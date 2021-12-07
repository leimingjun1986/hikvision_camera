#ifndef  HIKVISION_CAMERA_H
#define HIKVISION_CAMERA_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include  <thread>
#include <MvCameraControl.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include  <std_msgs/UInt32.h>
#include  <yaml-cpp/yaml.h>
#include  <csignal>
#include  <hikvision_camera/subscribe/subscriber_single_template.hpp>
#include  <vision_msgs/CameraControl.h>   //自定义消息头文件

using namespace std;

namespace   vision_sensor_ns    
{
   class  hikvision_camera
   {  
    public:  
      hikvision_camera();
      hikvision_camera(ros::NodeHandle&  nh_  ,  ros::NodeHandle& nh_priv_ );
      ~hikvision_camera();
      std::string hikvision_camera_config_file_path;
      std::string hikvision_camera_config_file_name;

      bool debug ;
       int   CAMERA_NUM  ;    //相机数目要改掉
      // bool stopSign ;     //中断函数控制  线程退出
       bool g_bExit  ;      // 线程意外 终止后  通知其他线程结束
       ros::NodeHandle   nh;
       ros::NodeHandle   nh_priv;
       std::vector<std::thread>  pids;    //多线程
      // 图像发送
      // 0: 右 1：左 2：后
       enum Camera{RIGHT=0, LEFT, REAR};
       //
       std::vector<image_transport::Publisher> cameraPub;     //多相机图像发布
       std::vector<std::string> cameraID;
       std::vector<std::string> cameraTopicName;

       vision_msgs::CameraControl   camera_control_;  //多相机控制消息
       std::string   camera_control_topic_name;

       ros::Publisher cameraUSBErrorStatusPub;
       ros::Publisher cameraUSBErrorVoicePub;
       ros::Publisher cameraUSBErrorAccumPub;   
       std::size_t cameraErrorCnt ;

       ros::Subscriber  cameraControlSub;

       std::size_t cameraErrorStatusPubCnt ;
       int  CAMERA_ERROR_THROTTLE ;
       int  CAMERA_ERROR_STATUS_PUB_THROTTLE ;
        bool hasCameraErrorHistory  ;

        //相机驱动部分  设备枚举查询    相机设备句柄创建    相机设备打开    相机网络包探测   开启线程循环采集图像

        MV_CC_DEVICE_INFO_LIST stDeviceList;  //接入的海康设备信息列表  通过枚举得到
        vector<void*>  handles ;       //相机句柄数组  与相机数目一致

        bool   hik_camera_init(); 
        bool   hik_camera_end(void* );
        bool    hik_camera_convert(); 
        bool    hik_camera_thread_start();    
/*
 * ＠brief:convert data stream in Mat format and send through ROS
 * @
 */
bool  init(ros::NodeHandle&  nh_  ,  ros::NodeHandle& nh_priv_ );

bool  import_config(string&   configFilePath_);
//导入参数文件中的参数
bool  import_config(ros::NodeHandle  node_priv);


bool convertAndSend(void* phandle,unsigned char * pData,unsigned char * pConvertData,  unsigned int  datalen,MV_FRAME_OUT_INFO_EX& stImageInfo, std::string& cameraIDInput);

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void  PressEnterToExit(void);
bool  PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

void  WorkThread(void* pUser);

double fps;  // 设置视频采集的帧率  ,相机sdk中也设置了帧率  , 最终取了较小的帧率

std::shared_ptr<SubscriberSingle<vision_msgs::CameraControl>> cameraControlSubPtr_;

//SubscriberSingle<vision_msgs::CameraControl>  cameraControlSub;

void cameraControlCallback(const vision_msgs::CameraControl &msg);

private:
    // virtual void onInit();    //此函数声明部分为固定格式，在nodelet加载此plugin会自动执行此函数
    //void onInit(ros::NodeHandle&   nh_,ros::NodeHandle& nh_priv_); 

};

}

#endif
