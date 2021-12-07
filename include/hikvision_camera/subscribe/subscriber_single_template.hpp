/*
 * @Description: ros SubscriberSingle template
 * @Author: Pi Fan
 * @Date: 2021-1-15
 */
#ifndef AGV_NAVIGATION_TEMPLATE_SUBSCRIBER_SINGLE_HPP_
#define AGV_NAVIGATION_TEMPLATE_SUBSCRIBER_SINGLE_HPP_

#include <ros/ros.h>
#include <mutex>

namespace vision_sensor_ns 
{
template<typename T>
class SubscriberSingle 
{
public:
    SubscriberSingle(ros::NodeHandle& nh, std::string topicName, size_t buffSize);
    SubscriberSingle() = default;
    bool parseData(T &data);

private:
    void msgCallback(const T &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    T newArrivedData_;
private:
    std::mutex  mutex_;
    bool isDataReady_;
};

template <typename T>
SubscriberSingle<T>::SubscriberSingle(ros::NodeHandle& nh, std::string topicName, size_t buffSize):nh_(nh),isDataReady_(false) 
{
    subscriber_ = nh_.subscribe(topicName, buffSize, &SubscriberSingle<T>::msgCallback, this);
}

template <typename T>
void SubscriberSingle<T>::msgCallback(const T &msg)
{

    std::unique_lock<std::mutex> lock(mutex_);
    newArrivedData_ = msg;
    isDataReady_ = true;
    ROS_INFO("subscribersingle  camera control msgCallback");
}

template <typename T>
bool SubscriberSingle<T>::parseData(T &data) 
{   
    bool re=false;
    std::unique_lock<std::mutex> lock(mutex_);
    if (isDataReady_ == true)
    {
        data = newArrivedData_;
        isDataReady_ = false;
        re=true;
    }
    return re;
}

}
#endif