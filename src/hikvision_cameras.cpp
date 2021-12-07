 #include  "hikvision_camera/app/hikvision_cameras.hpp"


namespace   vision_sensor_ns    
{
 
       int  stopSign=false;
        //信号中断  手动结束线程
       void  sigintHandler(int sig)
        {
            if (sig == SIGINT)
            {
                ROS_INFO_STREAM( "signal  ctrl+c pressed, manual close  camera  capture  thread!" );
                stopSign = true;
                /*
                if(ros::ok())
               ros::shutdown();*/
            }
        }    
        //类构造函数
        hikvision_camera::hikvision_camera()
        {
            cameraErrorCnt = 0;
            cameraErrorStatusPubCnt = 0;
            CAMERA_ERROR_THROTTLE =300;
            CAMERA_ERROR_STATUS_PUB_THROTTLE= 10;
            hasCameraErrorHistory = false;
            g_bExit=false  ;
        } 
        
         hikvision_camera::hikvision_camera(ros::NodeHandle&  nh_  ,  ros::NodeHandle& nh_priv_)
        {
            cameraErrorCnt = 0;
            cameraErrorStatusPubCnt = 0;
            CAMERA_ERROR_THROTTLE =300;
            CAMERA_ERROR_STATUS_PUB_THROTTLE= 10;
            hasCameraErrorHistory = false;
            g_bExit=false  ;
            bool re;
            re=init(nh_, nh_priv_) ;
            if(!re)  return ;
            image_transport::ImageTransport it(nh_priv_);
            for(int cam_id=0;cam_id<CAMERA_NUM;cam_id++)
            {
                image_transport::Publisher  impub=it.advertise(cameraTopicName.at(cam_id), 1); 
                cameraPub.push_back(impub);
            }
            re=hik_camera_init();
            if(!re)  
            {   
                ROS_ERROR("hik_camera_init  erro,no  start thread.");
                return;
            }
            re=hik_camera_thread_start();
        } 
        
        hikvision_camera::~hikvision_camera()    
        {

            ROS_INFO("~hikvision_camera function  start ."); 
            for(int i=0;i<handles.size();i++)
                {
                    bool re=hik_camera_end(handles[i]);
                    if(re)
                    {
                        ROS_INFO("camera[%d]  close successful.",i);
                    }
                    else
                    {
                    ROS_INFO("camera[%d]  close successful.",i);
                    }
                }
            ROS_INFO("~hikvision_camera function  finish ."); 
             if(ros::ok())
                {
                 ROS_INFO("prepare for shutdown node.");
                  ros::shutdown();     //关闭node节点
                }
        }
        
        bool  vision_sensor_ns::hikvision_camera::init(ros::NodeHandle&   nh_,ros::NodeHandle& nh_priv_ ){

                string  configFilePath_;
                //nh=getNodeHandle();
               // nh_priv=getPrivateNodeHandle();
               nh=nh_;
               nh_priv=nh_priv_;
               
                nh.param<std::string>("hkivision_config_path",configFilePath_,"/data/vision_ws/config/hikvision_camera_config.yaml");

                ROS_INFO_STREAM("hkivision_config_path:  "<<configFilePath_) ;
                   
                bool re= import_config(configFilePath_);
                // 相机故障
                cameraUSBErrorStatusPub = nh.advertise<std_msgs::Bool>("/vision_error", 1, true);
                cameraUSBErrorVoicePub = nh.advertise<std_msgs::UInt8>("/voice", 1, true);
                cameraUSBErrorAccumPub = nh.advertise<std_msgs::UInt32>("/vision_error_accum", 1, true);
                //cameraControlSubPtr_.reset(new  SubscriberSingle<vision_msgs::CameraControl>(nh_, camera_control_topic_name,1));
                cameraControlSub  =nh.subscribe(camera_control_topic_name,1,&hikvision_camera::cameraControlCallback,this);
                 

                //cameraControlSub =SubscriberSingle<vision_msgs::CameraControl>(nh_, camera_control_topic_name,1);

                signal(SIGINT, sigintHandler);
                if(re)  { 
                    ROS_INFO("hikvision_camera read config ***.xml param finish.");
                    }
                return re;
        }

        //通过配置文件名路径导入参数
        bool vision_sensor_ns::hikvision_camera::import_config(string&  configFilePath_){
        try
        {
            CAMERA_NUM=0;
            YAML::Node  configNode_ = YAML::LoadFile(configFilePath_);   //  YAML::Node
            if(configNode_.IsNull())
                {
                    ROS_ERROR("hikvision_camera load YAML  File  error!!!!");
                    return false ;
                } 
            YAML::iterator  it;
            it=configNode_["camera_config"].begin();
            CAMERA_NUM=it->second["camera_num"].as<int>();
            if(CAMERA_NUM<=0)
                {
                    ROS_ERROR("CAMERA_NUM  set  0  or   CAMERA_NUM  load  error !!!!");
                    return false ;
                } 
            for(int cam_id=0;cam_id<CAMERA_NUM;cam_id++){
                    std::string sn=it->second["camera_sn"][cam_id].as<std::string>();
                    if(sn.empty())
                       {
                                ROS_ERROR("camera_sn  %d   is  empty      load  error !!!!",cam_id);
                                return false ;
                       }
                    else
                        {
                                cameraID.push_back(sn);
                                ROS_INFO_STREAM("camera_sn:"<<sn);

                        }
                    std::string  topic_name=it->second["camera_topic_name"][cam_id].as<std::string>();
                    if(topic_name.empty())
                       {
                                ROS_ERROR("camera_topic_name  %d   is  empty      load  error !!!!",cam_id);
                                return false ;
                       }
                    else
                      {
                             cameraTopicName.push_back(topic_name);
                            ROS_INFO_STREAM("topic_name:"<<topic_name);
                      }
            }
           
            camera_control_topic_name=it->second["camera_control_topic_name"].as<std::string>();
             ROS_INFO_STREAM("camera_control_topic_name:"<<camera_control_topic_name);
            camera_control_.controlByte=it->second["camera_control_value"].as<uint8_t>();
             ROS_INFO_STREAM("camera_control_.controlByte:"<<camera_control_.controlByte);
            fps=it->second["FPS"].as<double>();
            debug=it->second["debug"].as<bool>();
            ROS_INFO_STREAM("debug value:"<<debug);


            return true;
        }
        catch(std::exception& e){

                ROS_ERROR_STREAM("hikvision_camera  import_config  ERROR."<<e.what());
                return false;        }
            
        }

        bool  vision_sensor_ns::hikvision_camera::convertAndSend(void* phandle,unsigned char * pData, unsigned char * pConvertData, unsigned int  datalen,MV_FRAME_OUT_INFO_EX& stImageInfo, std::string& cameraIDInput)
        {
            cv::Mat srcImage;
           int nRet;
            bool isMono;//判断是否为黑白图像
            switch (stImageInfo.enPixelType)
            {
            case PixelType_Gvsp_Mono8:
            case PixelType_Gvsp_Mono10:
            case PixelType_Gvsp_Mono10_Packed:
            case PixelType_Gvsp_Mono12:
            case PixelType_Gvsp_Mono12_Packed:
                isMono = true;
                break;
            default:
                isMono = false;
                break;
            }

         if (isMono)
            {
                if(stImageInfo.enPixelType==PixelType_Gvsp_Mono8)
                    {
                        srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);
                        if(debug)
                        ROS_INFO_ONCE("PixelType_Gvsp_Mono8");     
                    }
                else
                 {
                       MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 };
                        memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
                        stConvertParam.nWidth = stImageInfo.nWidth;             
                        stConvertParam.nHeight = stImageInfo.nHeight;           
                        stConvertParam.pSrcData = pData;             
                        stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;   
                        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;  

                        stConvertParam.enDstPixelType = PixelType_Gvsp_Mono8;
                        stConvertParam.pDstBuffer = pConvertData;                
                        stConvertParam.nDstBufferSize = datalen;        
                        nRet=MV_CC_ConvertPixelType(phandle, &stConvertParam);
                        srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pConvertData); 
                         if(debug)
                        ROS_INFO_ONCE("isMon  but  no PixelType_Gvsp_Mono8");     //是黑白图像  但不是 mono8 像素格式
                 }
            }

        if (!isMono)
            {
                if(stImageInfo.enPixelType==PixelType_Gvsp_RGB8_Packed||stImageInfo.enPixelType==PixelType_Gvsp_BGR8_Packed)
                    { 
                        srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
                        if(debug)
                        ROS_INFO_ONCE("PixelType_Gvsp_RGB8_Packed");    //彩色图像 且为 RGB8格式
                    }
               else
                    {
                        //转换图像格式为BGR8
                        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 };
                        memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
                        stConvertParam.nWidth = stImageInfo.nWidth;             
                        stConvertParam.nHeight = stImageInfo.nHeight;           
                        stConvertParam.pSrcData = pData;             
                        stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;   
                        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;  

                        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                        //stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
                        stConvertParam.pDstBuffer = pConvertData;                
                        stConvertParam.nDstBufferSize = datalen;        
                        nRet=MV_CC_ConvertPixelType(phandle, &stConvertParam);
                        srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pConvertData);
                         if(debug)
                        ROS_INFO_ONCE("no PixelType_Gvsp_RGB8_Packed");   //彩色图像  非 RGB8格式
                    }
            }

            //　发布图片消息
            // 並不需要將rgb轉換成bgr,有rgb的格式
            sensor_msgs::ImagePtr msg;
            if (isMono)
                msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", srcImage).toImageMsg();//图像格式转换
            else
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();//图像格式转换
                
            msg->header.frame_id = cameraIDInput;
            msg->header.stamp = ros::Time::now();

            if (cameraIDInput ==  cameraID.at(RIGHT))
            {
                cameraPub.at(RIGHT).publish(msg);
            }
            else if (cameraIDInput == cameraID.at(LEFT))  
            {
                cameraPub.at(LEFT).publish(msg);
            }
            else if (cameraIDInput == cameraID.at(REAR))
            {
                cameraPub.at(REAR).publish(msg);
            }
            
            return true;
        }

        // 等待用户输入enter键来结束取流或结束程序  函数没有用到
        void vision_sensor_ns::hikvision_camera::PressEnterToExit(void)
        {
            int c;
            while ( (c = getchar()) != '\n' && c != EOF );
            fprintf( stderr, "\nPress enter to exit.\n");
            while( getchar() != '\n');
            g_bExit = true;
            sleep(1);
        }
        //打印相机设备信息  没有调用
        bool vision_sensor_ns::hikvision_camera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
        {
            if (NULL == pstMVDevInfo)
            {
                printf("The Pointer of pstMVDevInfo is NULL!\n");
                return false;
            }
            if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
            {
                int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

                // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
                printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
                printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
                printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
            }
            else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
            {
                printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
                printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            }
            else
            {
                printf("Not support.\n");
            }

            return true;
        }
       
       //循环采集图像线程函数  参数为相机句柄
        void  vision_sensor_ns::hikvision_camera::WorkThread(void* pUser)
        {
            int nRet = MV_OK;
            MVCC_STRINGVALUE stStringValue = {0};
            char camSerialNumber[256] = {0};
            nRet = MV_CC_GetStringValue(pUser, "DeviceSerialNumber", &stStringValue);   //获取串口号
            if (MV_OK == nRet)
            {
                memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
            }
            else
            {  
                if(debug)
                ROS_ERROR("Get camSerialNumber  error . nRet = [%x]\n", nRet);
                return ;
            }
            // ch:获取数据包大小
            MVCC_INTVALUE stParam;
            memset(&stParam, 0, sizeof(MVCC_INTVALUE));
            nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
            if (MV_OK != nRet)
            {
                ROS_ERROR("Get Data  package size error.nRet:[0x%x]\n", nRet);
                return ;
            }
            else{
                ROS_INFO_ONCE(" Get Data  package size:   %d. \n", stParam.nCurValue);
            }

            MV_FRAME_OUT_INFO_EX stImageInfo = {0};
            memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
            unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);  //存放元素数据
            
            MVCC_INTVALUE  Width,Height;
            nRet = MV_CC_GetIntValue(pUser, "Width",  &Width);
            nRet = MV_CC_GetIntValue(pUser, "Height", &Height);
            MVCC_ENUMVALUE   PixelFormat = {0}; 
            nRet = MV_CC_GetEnumValue(pUser, "PixelFormat", &PixelFormat);

            MVCC_ENUMVALUE PixelSize={0};
            nRet = MV_CC_GetEnumValue(pUser, "PixelSize", &PixelSize);
            ROS_INFO_STREAM("PixelFormat:"<<PixelFormat.nCurValue); 
            ROS_INFO_STREAM("PixelSize :"<<PixelSize.nCurValue);

            int   convert_data_len = Width.nCurValue * Height.nCurValue * 3 ;  
            ROS_INFO("Frame  width:%d  height: %d  len:%d",Width.nCurValue, Height.nCurValue,convert_data_len);
             unsigned char*  pConvertData = (unsigned char*)malloc(convert_data_len);    //为彩色图像时 将数据转换到此内存

            if (NULL == pData)
            { 
                ROS_INFO(" malloc memery faliture.\n");
                return ;
            }
            unsigned int nDataSize = stParam.nCurValue;
            
            //获取当前线程 对应哪个相机
            int camid=0;
            for(int i=0;i<cameraID.size();i++)
            {
                if(cameraID[i].compare(camSerialNumber)==0){
                    camid=i;
                }
            }
        
        ROS_INFO(" camera [%d] enter  loop  capture thread.\n",camid);
        ros::Rate loop_rate(fps);
         //while start
            while(!vision_sensor_ns::stopSign&&!g_bExit)
            {
                ros::Time   tt1=ros::Time::now();
                //cameraControlSubPtr_->parseData(camera_control_);
             //cameraControlSub.parseData(camera_control_);
             //ROS_INFO_STREAM("camera_control_.controlByte:"<<camera_control_.controlByte);
              //ROS_INFO("hikcamera %d, camera_control.controlByte %d" ,camid,camera_control_.controlByte);
                if((camera_control_.controlByte>>camid)&0x01?true:false)
                {
                        double  t10 =ros::Time::now().toSec();
                        double  ts =ros::Time::now().toSec();
                        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
                        double  te =ros::Time::now().toSec();
                        if(debug){
                        ROS_INFO("MV_CC_GetOneFrameTimeout cost time: %f",(te-ts)*1000);}
                        if (nRet == MV_OK)
                        {
                            /*
                            printf("Cam Serial Number[%s]:GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
                                camSerialNumber, stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
                            */
                            std::string cameraID = camSerialNumber;
                            double  t1 =ros::Time::now().toSec();
                            convertAndSend(pUser,pData, pConvertData,convert_data_len,stImageInfo, cameraID);
                        
                            cameraErrorCnt = 0;
                            if (hasCameraErrorHistory == true)
                            {
                                hasCameraErrorHistory = false;
                                // 视觉恢复，解除停车
                                std_msgs::Bool statusMsg;
                                statusMsg.data = false;
                                cameraUSBErrorStatusPub.publish(statusMsg);
                            }
                            double t2 =ros::Time::now().toSec() ;
                            if(debug){
                            ROS_INFO("convertAndSend cost time: %f",(t2-t1)*1000);}
                        }
                        else
                        {
                            double  t1 =ros::Time::now().toSec();
                            if(debug)
                            printf("cam[%s]:Get One Frame failed![%x]\n", camSerialNumber, nRet);
                            // exit(1);
                            cameraErrorCnt++;
                            if (cameraErrorCnt > CAMERA_ERROR_THROTTLE)  //连续  多次获取不到图像  视觉错误  发布topic
                            {
                                // 视觉故障，停车消息
                                std_msgs::Bool statusMsg;
                                statusMsg.data = true;
                                cameraUSBErrorStatusPub.publish(statusMsg);
                                // 视觉故障，语音播报
                                std_msgs::UInt8 voiceMsg;
                                voiceMsg.data = 5;
                                cameraUSBErrorVoicePub.publish(voiceMsg);
                                // 延时
                                hasCameraErrorHistory = true;
                                cameraErrorCnt = 0;
                                cameraErrorStatusPubCnt++;
                            }
                            // 发送多次，重启程序
                            if (cameraErrorStatusPubCnt > CAMERA_ERROR_STATUS_PUB_THROTTLE)
                            {
                                ROS_ERROR("cam[%s] :  %d  times  camera  loss %d frames ,thread  exit  ",camSerialNumber, CAMERA_ERROR_STATUS_PUB_THROTTLE,CAMERA_ERROR_THROTTLE);
                                // vision_sensor_ns::stopSign = true;
                                g_bExit=true;
                                //  exit(1);
                            }

                            if (debug)
                            {
                                std_msgs::UInt32 accumMsg;
                                accumMsg.data = cameraErrorStatusPubCnt;
                                cameraUSBErrorAccumPub.publish(accumMsg);
                            }

                        double t2 =ros::Time::now().toSec() ;
                        if(debug){
                            ROS_INFO(" One Frame failed cost time: %f",(t2-t1)*1000);}  
                        }
                    double  t20 =ros::Time::now().toSec();
                    if(debug){
                    ROS_INFO("while  cost time: %f",(t20-t10)*1000);}
                 }
                
                /*
                ros::Time   tt2 =ros::Time::now();
                ros::Duration during = tt2 - tt1;
                if(during.toSec() < 1.0/fps)
                {
                    ros::Duration(1.0/fps-during.toSec()).sleep();
                }   */
                ros::spinOnce();
                loop_rate.sleep();           
            }
            //while end
            if(pData!=NULL)
              {
                  free(pData);  
                  pData=NULL ;
              }           
            if(pConvertData!=NULL)
              {
                  free(pConvertData); 
                   pConvertData=NULL ;
               }
            g_bExit=true;
            ROS_INFO("cmare[%s] caputre thread exit.",camSerialNumber);        
        }
        
        bool   vision_sensor_ns::hikvision_camera::hik_camera_init(){
                    int  nRet;
                    //设置海康相机日志路径
                    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
                    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
                    if (nRet!=MV_OK)
                        {
                            ROS_ERROR("Enum hik  device error.   nRet:%x",nRet);
                            return  false;
                        }
                    unsigned int nIndex = 0;
                    if (stDeviceList.nDeviceNum > 0)
                    {
                        ROS_INFO_STREAM("Find  hik  cameras:  "<<std::to_string(stDeviceList.nDeviceNum)<<"  .");
                    } 
                    else
                    {
                        ROS_ERROR("No Find any  hik cameras");
                        return false;
                    }

                if(stDeviceList.nDeviceNum < CAMERA_NUM)
                    {
                        ROS_ERROR("set  %d   cameras,  actual find  %d  cameras.", CAMERA_NUM, stDeviceList.nDeviceNum);
                        return false;
                    }
            
                //相机句柄vector  
                handles.clear();

                // 循环完成相机设备初始化动作   start    设定了多少个相机  初始化多少个相机句柄
                for(int i = 0; i < CAMERA_NUM; i++)     
                    {
                        int nIndex=i ;   //设备号索引根据序列号从   设备列表中选取
                        //找到设定序列号 对应的DeviceList位置   要严格按照设定的序列号顺序初始化相机
                        int fid=-1;

                        for(int n=0;  n< stDeviceList.nDeviceNum; n++)
                        {
                             if (stDeviceList.pDeviceInfo[n]->nTLayerType == MV_GIGE_DEVICE)
	                        	{
		                  	       if (strcmp((const char*)stDeviceList.pDeviceInfo[n]->SpecialInfo.stGigEInfo.chSerialNumber, cameraID[i].c_str())==0)
		 	                           {
			                          	 fid = n;
		                            	}
	                        	}
                            else if (stDeviceList.pDeviceInfo[n]->nTLayerType == MV_USB_DEVICE)
                            {
                                if (strcmp((const char*)stDeviceList.pDeviceInfo[n]->SpecialInfo.stUsb3VInfo.chSerialNumber, cameraID[i].c_str())==0)
                                    {
                                        fid = n;
                                    }
                            }
                        }

                        if(fid<0)  
                          {
                              ROS_ERROR("camer[%d] in DeviceList  no  find  same  serial number",i);
                              return false;
                          }
                        // 选择设备并创建句柄
                        void*   handle = NULL ;
                        ROS_INFO("i:%d ,set sn %s, fid[%d], sn:%s ", i , (const char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber,fid,cameraID[i].c_str());

                        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[fid]);
                        handles.push_back(handle);     //保证相机句柄数组  与设置的序列号相机顺序一致
                        if (MV_OK != nRet)
                        {
                            ROS_ERROR("camera[%d] Handle  Create Error.  nRet:%x", i , nRet);   //打印的是  i 
                            MV_CC_DestroyHandle(handle);
                            return false;
                        }
                        // 打开设备
                        nRet = MV_CC_OpenDevice(handles[nIndex]);
                        if (MV_OK != nRet)
                        {
                            ROS_ERROR("camera [%d] open   failure.  nRet:%x\n", nIndex,nRet);
                            MV_CC_DestroyHandle(handle);
                            return false;
                        }
                        else{
                            ROS_INFO("camera[%d] open  successful\n", nIndex);
                        }
                        
                        // ch:探测网络最佳包大小(只对GigE相机有效) 
                        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
                        {
                            int nPacketSize = MV_CC_GetOptimalPacketSize(handles[nIndex]);
                            if (nPacketSize > 0)
                            {
                                nRet = MV_CC_SetIntValue(handles[nIndex],"GevSCPSPacketSize",nPacketSize);
                                if(nRet != MV_OK)
                                {
                                    ROS_WARN("warning: camera[%d]  survey  network  packet size  erroor.   nRet:[0x%x]!\n", nIndex,nRet);
                                    return false;
                                }
                            }
                            else
                            {
                                ROS_INFO("camera [%d] survey  network  packet size  successful:  packet size[0x%x]:\n",nIndex, nPacketSize);
                                return true ;
                            }
                        }

                        // 设置触发模式为off
                        nRet = MV_CC_SetEnumValue(handles[i], "TriggerMode", MV_TRIGGER_MODE_OFF);
                        if (MV_OK != nRet)
                        {
                            ROS_ERROR("camera [%d]: turn off TriggerMode  failure. nRet[%x]\n", nIndex, nRet);
                            return false ;
                        }       
                    }
                // 循环完成相机设备初始化动作   end

                // 循环开启相机视频流  尽量保证同时  start
                for(int i=0;i<CAMERA_NUM;i++)
                {
                        nRet = MV_CC_StartGrabbing(handles[i]);
                        if (MV_OK != nRet)
                        {
                            ROS_ERROR("camera[%d]: start grabbing  video stream error.   nRet [%x]\n",i, nRet);
                            return false;
                        }
                        else{            
                            ROS_INFO("camera[%d]: start grabbing  video stream successful.\n",i);
                        }
                }
                // 循环开启相机视频流    尽量保证同时  end
            
                return true;
        }

        bool   vision_sensor_ns::hikvision_camera::hik_camera_end(void*  handle)
        {
            int nRet;
            // 停止取流
            if(handle==NULL)
            return false;

            nRet = MV_CC_StopGrabbing(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
                return  false;
            }
            // 关闭设备
            nRet = MV_CC_CloseDevice(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
                return  false;
            }
            // 销毁句柄
            nRet = MV_CC_DestroyHandle(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
                return false;
            }
  
            return true;
    }

        bool vision_sensor_ns::hikvision_camera::hik_camera_thread_start()
        {
        //开启相机采集程序的多线程
        pids.clear();
        for (int i=0;i<CAMERA_NUM;i++) 
            {
                std::thread   Thred(&vision_sensor_ns::hikvision_camera::WorkThread, this, this->handles[i]) ;  //创建线程
                pids.push_back(std::move(Thred));  //不能直接拷贝THread  用线程转移
            }

            ROS_INFO("hik_camera_mult_thread_start  finish\n");
            //阻塞多线程
            for(int i=0;i<pids.size();i++){
                //pids[i].detach();
                pids[i].join();
            }
           //线程退出阻塞结束
            ROS_INFO("hik_camera_mult_thread_start   endh\n");
            return true;
        }    
 
        void vision_sensor_ns::hikvision_camera::cameraControlCallback(const vision_msgs::CameraControl &msg)
                {
                    camera_control_ =  msg ;
                    ROS_INFO("hikvision_camera::cameraControlCallback   byte:%d",camera_control_.controlByte);
                }

}
