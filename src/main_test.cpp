#include   <hikvision_camera/app/hikvision_cameras.hpp>

using namespace vision_sensor_ns;   //直接调用相机类
int main(int argc, char **argv)
{
   
       ros::init(argc, argv, "hik_camera");
       ros::NodeHandle nh_priv("~"); 
       ros::NodeHandle nh;

       hikvision_camera  hikcam(nh,nh_priv);

       ROS_INFO("main  thread......");

     //相机采集线程处于join状态时 没有调用  ros::spinOnce    camera_control消息没有被成功接收(定阅)到
        /*
       ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();     
            loop_rate.sleep();
        }*/
      return 0;
}