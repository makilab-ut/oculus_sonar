#include "OculusNode.h"
#include "conversions.h"
#include <ros/ros.h>

ros::Time last_ping_time_;
int ping_count_;
double total_time_elapsed_;

OculusNode::OculusNode(const std::string& nodeName) :
    node_(nodeName),
    configServer_(node_),
    sonar_(service_.io_service())
{
    node_.param<std::string>("ping_topic",       pingTopic_,      "ping");
    node_.param<std::string>("status_topic",     statusTopic_,    "status");
    // node_.param<std::string>("raw_topic",        rawTopic_,       "raw");
    // node_.param<std::string>("ping_image_topic", pingImageTopic_, "ping_image");
    // node_.param<std::string>("ping_topic_deprecated",  pingTopicDeprecated_,  "ping_deprecated");

    pingPublisher_   = node_.advertise<oculus_sonar::Ping>        (pingTopic_,      1);
    statusPublisher_ = node_.advertise<oculus_sonar::OculusStatus>(statusTopic_,    1);
    // imagePublisher_  = node_.advertise<sensor_msgs::Image>        (pingImageTopic_, 100);
    // rawPublisher_    = node_.advertise<oculus_sonar::Raw>         (rawTopic_,       100);
    // pingPublisherDeprecated_ = node_.advertise<oculus_sonar::OculusPing>(pingTopicDeprecated_, 100);

    node_.param<bool>("publish_without_subs", publishWithoutSubs_, false);

    sonar_.add_ping_callback(std::bind(&OculusNode::ping_callback,this, std::placeholders::_1));
    sonar_.add_message_callback(std::bind(&OculusNode::message_callback,this, std::placeholders::_1));
    sonar_.add_status_callback(std::bind(&OculusNode::status_callback,this, std::placeholders::_1));
    //sonar_.add_dummy_callback(std::bind(&OculusNode::dummy_callback,this, std::placeholders::_1));
    this->start();

    configServer_.setCallback(std::bind(&OculusNode::reconfigure_callback, this,std::placeholders::_1, std::placeholders::_2));
}

OculusNode::~OculusNode()
{
    this->stop();
}

void OculusNode::start()
{
    service_.start();
    if(!sonar_.wait_next_message()) {
        throw std::runtime_error("Timeout reached while waiting for sonar. Is it plugged in ?");
    }
}

void OculusNode::stop()
{
    service_.stop();
}

void OculusNode::ping_callback(const oculus::PingMessage::ConstPtr& ping)
{
    if(!publishWithoutSubs_ && !this->has_ping_subscribers()) {
        std::cout << "Going to standby mode" << std::endl;
        sonar_.standby();
    }

    // Calculate the time since the last callback
    ros::Time now = ros::Time::now();
    if (!last_ping_time_.isZero()) { // Skip the first callback since there's no previous time
        ros::Duration time_diff = now - last_ping_time_;
        total_time_elapsed_ += time_diff.toSec();
        ping_count_++;
        if (ping_count_ >= 10) { // Log every 10 messages, for example
            double frequency = static_cast<double>(ping_count_) / total_time_elapsed_;
            ROS_INFO_STREAM("Ping publish frequency: " << frequency << " Hz");
            // Reset counters
            ping_count_ = 0;
            total_time_elapsed_ = 0.0;
        }
    }
    last_ping_time_ = now;

    oculus_sonar::Ping msg;
    oculus::copy_to_ros(msg, ping);
    pingPublisher_.publish(msg);


}

void OculusNode::publish_deprecated(const oculus::PingMessage::ConstPtr& ping)
{
    if(ping->message()->message_version() == 2) {
        // v2 oculus message not compatible with OculusStampedPing
        return;
    }
    oculus_sonar::OculusStampedPing msg;

    const std::vector<uint8_t>&   pingData = ping->message()->data();
    const OculusSimplePingResult& pingMetadata =
        *reinterpret_cast<const OculusSimplePingResult*>(pingData.data());
    
    oculus::copy_to_ros(msg.ping, pingMetadata);
    msg.ping.data.resize(pingData.size());
    for(int i = 0; i < msg.ping.data.size(); i++)
        msg.ping.data[i] = pingData[i];

    msg.header.stamp    = oculus::to_ros_stamp(ping->timestamp());
    msg.header.frame_id = "oculus_sonar";
    pingPublisherDeprecated_.publish(msg);
}

void OculusNode::status_callback(const OculusStatusMsg& status)
{
    oculus_sonar::OculusStatus msg;
    oculus::copy_to_ros(msg, status);
    statusPublisher_.publish(msg);
}

void OculusNode::message_callback(const oculus::Message::ConstPtr& msg)
{
    //oculus_sonar::Raw rosMsg;
    //oculus::copy_to_ros(rosMsg, msg);
    //rawPublisher_.publish(rosMsg);
}

void OculusNode::reconfigure_callback(oculus_sonar::OculusSonarConfig& config,
                                      int32_t level)
{
    oculus::SonarDriver::PingConfig currentConfig;
    std::memset(&currentConfig, 0, sizeof(currentConfig));


    currentConfig.masterMode = config.frequency_mode;
    int ping_rate;
    switch(config.ping_rate)
    {
        case 0: currentConfig.pingRate = pingRateNormal;  ping_rate = 10; break;
        case 1: currentConfig.pingRate = pingRateHigh;    ping_rate = 15; break;
        case 2: currentConfig.pingRate = pingRateHighest; ping_rate = 40; break;
        case 3: currentConfig.pingRate = pingRateLow;     ping_rate =  5; break;
        case 4: currentConfig.pingRate = pingRateLowest;  ping_rate =  2; break;
        case 5: currentConfig.pingRate = pingRateStandby; ping_rate =  0; break;
        default:break;
    }

    // flags
    //currentConfig.flags = 0x09; // always in meters, simple ping
    currentConfig.flags = 0x01  // always in meters
                        | 0x04  // force send gain to true
                        | 0x08; // use simple ping
    // switch(config.data_depth)
    // {
    //     case oculus_sonar::OculusSonar_8bits:
    //         break;
    //     case oculus_sonar::OculusSonar_16bits:
    //         currentConfig.flags |= 0x02;
    //         break;
    //     default:break;
    // }
    // switch(config.nbeams)
    // {
    //     case oculus_sonar::OculusSonar_256beams:
    //         break;
    //     case oculus_sonar::OculusSonar_512beams:
    //         currentConfig.flags |= 0x40;
    //         break;
    //     default:break;
    // }
    //if(config.send_gain)
    //    currentConfig.flags |= 0x04;
    if(config.gain_assist)
        currentConfig.flags |= 0x10;

    currentConfig.range           = config.range;
    currentConfig.gammaCorrection = config.gamma_correction;
    currentConfig.gainPercent     = config.gain_percent;

    if(config.use_salinity)
        currentConfig.speedOfSound = 0.0;
    else
        currentConfig.speedOfSound = config.sound_speed;
    currentConfig.salinity = config.salinity;
    
    //sonar_.send_ping_config(currentConfig);
    // // a timeout would be nice
    auto feedback = sonar_.request_ping_config(currentConfig);
    config.frequency_mode   = feedback.masterMode;
    //config.ping_rate      = feedback.pingRate // is broken (?) sonar side
    // config.data_depth       = (feedback.flags & 0x02) ? 1 : 0;
    config.send_gain        = (feedback.flags & 0x04) ? 1 : 0;
    //config.full_ping        = (feedback.flags & 0x08) ? 1 : 0;
    config.gain_assist      = (feedback.flags & 0x10) ? 1 : 0;
    // config.nbeams           = (feedback.flags & 0x40) ? 1 : 0;
    config.range            = feedback.range;
    config.gamma_correction = feedback.gammaCorrection;
    config.gain_percent     = feedback.gainPercent;
    config.sound_speed      = feedback.speedOfSound;
    config.salinity         = feedback.salinity;

    ROS_INFO("---Sonar Config Updated ---");
    ROS_INFO("Frequency: %d kHz", currentConfig.masterMode*450+300);
    ROS_INFO("Range    : %f m", currentConfig.range);
    ROS_INFO("PingRate : %d Hz", ping_rate);
}

bool OculusNode::has_ping_subscribers() const
{
    return pingPublisher_.getNumSubscribers() > 0
        || rawPublisher_.getNumSubscribers() > 0
        || imagePublisher_.getNumSubscribers() > 0
        || pingPublisherDeprecated_.getNumSubscribers() > 0;
}

void OculusNode::dummy_callback(const OculusMessageHeader& msg)
{
    if(publishWithoutSubs_ || this->has_ping_subscribers()) {
        std::cout << "Exiting standby mode" << std::endl;
        sonar_.resume();
    }
}