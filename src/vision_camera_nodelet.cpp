#include   "hikvision_camera/app/vision_camera_nodelet.hpp"
 #include   <pluginlib/class_list_macros.h>

 using  namespace   vision_sensor_ns;

vision_camera_nodelet::vision_camera_nodelet()
{
    ;
}

vision_camera_nodelet::~vision_camera_nodelet(){
    ;
}

//nodelet 动态加载入口
void vision_camera_nodelet::onInit()
{
  ros::NodeHandle nh=getNodeHandle();
  ros::NodeHandle nh_priv=getPrivateNodeHandle();
  hik_cmaera_ptr.reset(new  hikvision_camera(nh,nh_priv));    
}

PLUGINLIB_EXPORT_CLASS(vision_sensor_ns::vision_camera_nodelet, nodelet::Nodelet)