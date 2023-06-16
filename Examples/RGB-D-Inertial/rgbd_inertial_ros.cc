//created by DXJ on 2023-0614

#include <ros/ros.h>//ROS的头文件
#include <sensor_msgs/Image.h>//ROS图像类型
#include <sensor_msgs/image_encodings.h>//ROS图像类型的编码函数
#include <sensor_msgs/PointCloud.h>//ROS点云类型
#include <sensor_msgs/Imu.h>//ROS IMU类型
#include <cv_bridge/cv_bridge.h>//ROS图像类型和OpenCV图像类型转换的函数
#include <message_filters/subscriber.h>//消息过滤器
#include <message_filters/time_synchronizer.h>//时间同步器
#include <message_filters/sync_policies/approximate_time.h>//时间同步器的策略
#include <message_filters/synchronizer.h>//时间同步器
#include <std_msgs/Bool.h>//ROS布尔类型

#include <iostream>//C++标准输入输出库
#include <fstream>//C++文件输入输出库
#include <algorithm>//C++算法库
#include <chrono>//C++时间库
#include <thread>//C++线程库
#include <queue>//C++队列库
#include <mutex>//C++互斥锁库

#include <opencv2/opencv.hpp>//OpenCV头文件
#include <opencv2/core/core.hpp>//OpenCV核心头文件

#include <System.h>//ORB-SLAM的头文件
#include <ImuTypes.h>//ORB-SLAM的IMU类型头文件
#include <glog/logging.h>//Google日志库
#include <gflags/gflags.h>//Google命令行参数库

using namespace std;//使用标准命名空间

template<typename T>//模板类
T readParam(ros::NodeHandle &n, string name)//读取参数
{
    T ans;//定义变量
    if (n.getParam(name, ans))//如果读取成功
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);//输出读取的参数
    } else//如果读取失败
    {
        ROS_ERROR_STREAM("Failed to load " << name);//输出读取失败的信息
        n.shutdown();//关闭节点
    }
    return ans;//返回读取的参数
}


class ImuGrabber//IMU数据获取类
{
public:
    ImuGrabber(){};//构造函数
    void GrabImu(const sensor_msgs::ImuConstPtr &msg);//IMU数据获取函数

    queue<sensor_msgs::ImuConstPtr> imu0buf;//IMU数据队列
    std::mutex m_buf;//互斥锁
};

class ImageGrabber//图像数据获取类
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,ImuGrabber *pImuGb):mpSLAM(pSLAM),mpImuGb(pImuGb){}//构造函数
    ORB_SLAM3::System* mpSLAM;//ORB-SLAM系统指针
    ImuGrabber* mpImuGb;//IMU数据获取类指针
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);//图像数据获取函数
    void SyncWithImu();//图像和IMU数据同步函数
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr& msgRGB);//获取图像函数
    cv::Mat GetDepth(const sensor_msgs::ImageConstPtr& msgD);//获取深度图函数

    queue<sensor_msgs::ImageConstPtr> img0buf;//图像数据队列
    queue<sensor_msgs::ImageConstPtr> dep0buf;//深度图数据队列
    std::mutex m_buf;//互斥锁

};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"rgbd_inertial_ros");//初始化ROS节点
    ros::start();//启动ROS节点

    google::InitGoogleLogging(argv[0]);//初始化Google日志库
    google::SetLogDestination(google::GLOG_INFO, "./log/ORB_SLAM3_");//设置日志输出目录
    google::SetStderrLogging(google::GLOG_INFO);//设置日志输出级别
    google::SetLogFilenameExtension("log");//设置日志文件后缀名
    google::EnableLogCleaner(5);//设置日志清理时间间隔

    ros::NodeHandle nh;//定义ROS节点句柄

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);//设置ROS日志级别

    string strVocFile = readParam<string>(nh, "Vocabulary");//读取ORB词典文件路径
    string strConfig = readParam<string>(nh, "config_file");//读取配置文件路径

    ORB_SLAM3::System SLAM(strVocFile, strConfig, ORB_SLAM3::System::IMU_RGBD,nh,true);//创建ORB-SLAM系统

    // ORB_SLAM3::Config::setParameterFile(strConfig);//设置ORB-SLAM配置文件路径

    ImuGrabber igb;//创建IMU数据获取类
    ImageGrabber igb0(&SLAM,&igb);//创建图像数据获取类

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);//定义图像话题订阅者
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);//定义深度图话题订阅者

    ros::Subscriber imu_sub = nh.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &igb);//定义IMU话题订阅者
    ros::Publisher Save_map_ = nh.advertise<std_msgs::Bool>("/save_map_cmd", 10);//定义保存地图话题发布者

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;//定义时间同步策略
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);//定义时间同步器

    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb0, _1, _2));//注册回调函数

    thread sync_thread(&ImageGrabber::SyncWithImu, &igb0);//创建图像和IMU数据同步线程

     while(1)
    {
        char key = getchar();
        ros::spinOnce();
        if(key == 's')
            // Stop all threads
            SLAM.Shutdown();
            // Save camera trajectory
//            SLAM.SaveTrajectoryTUM(string("/media/wangwen/01D747F7BEB117101/DataSets/Science_Corridor/room_4_full") + string("/CameraTrajectory_DR.txt"));
//            SLAM.SaveTrajectoryManhattan(string("/media/wangwen/01D747F7BEB117101/DataSets/Science_Corridor/room_4_full")+string("/CameraTrajectory_DRManhattan.txt"));
            break;
    }
    ros::shutdown();
    google::ShutdownGoogleLogging();

    return 0;

}

//RGBD数据获取函数
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{

    m_buf.lock();
    if (!img0buf.empty())
        img0buf.pop();
    if (!dep0buf.empty())
        dep0buf.pop();

    img0buf.push(msgRGB);
    dep0buf.push(msgD);
    m_buf.unlock();
}

//图像和IMU数据同步函数
void ImageGrabber::SyncWithImu(){
    while(1)
    {
        cv::Mat im;
        cv::Mat dep;
        double tIm = 0;
        if (!img0buf.empty()&& !dep0buf.empty() &&!mpImuGb->imu0buf.empty())
        {
            tIm = img0buf.front()->header.stamp.toSec();
            if(tIm>mpImuGb->imu0buf.back()->header.stamp.toSec())
                continue;
            {
                this->m_buf.lock();
                im = GetImage(img0buf.front());
                img0buf.pop();
                dep = GetDepth(dep0buf.front());
                dep0buf.pop();
                this->m_buf.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->m_buf.lock();
            while(!mpImuGb->imu0buf.empty() && mpImuGb->imu0buf.front()->header.stamp.toSec()<=tIm)
            {
                double t = mpImuGb->imu0buf.front()->header.stamp.toSec();
                cv::Point3f acc(mpImuGb->imu0buf.front()->linear_acceleration.x, mpImuGb->imu0buf.front()->linear_acceleration.y, mpImuGb->imu0buf.front()->linear_acceleration.z);
                cv::Point3f gyr(mpImuGb->imu0buf.front()->angular_velocity.x, mpImuGb->imu0buf.front()->angular_velocity.y, mpImuGb->imu0buf.front()->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                mpImuGb->imu0buf.pop();
            }
            mpImuGb->m_buf.unlock();

            Eigen::Quaterniond a(1,0,0,0);
            mpSLAM->TrackRGBD(im,dep,tIm,vImuMeas);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

//获取图像
cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &msgRGB)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return cv_ptrRGB->image.clone();

}

//获取深度图
cv::Mat ImageGrabber::GetDepth(const sensor_msgs::ImageConstPtr &msgD)
{

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return cv_ptrD->image.clone();

}

//IMU数据获取类
void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &msgImu)
{
    m_buf.lock();
    imu0buf.push(msgImu);
    m_buf.unlock();
    return;
}