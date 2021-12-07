#include  <nodelet/nodelet.h>
#include <hikvision_camera/app/hikvision_cameras.hpp>
 
 namespace  vision_sensor_ns{

        // nodelet  接口类
        class  vision_camera_nodelet:public nodelet::Nodelet      
        {
            public:
            vision_camera_nodelet();
            ~ vision_camera_nodelet();
            
            boost::shared_ptr<hikvision_camera>   hik_cmaera_ptr;

            private:
            virtual void onInit();    //此函数声明部分为固定格式，在nodelet加载此plugin会自动执行此函数

        };

 }