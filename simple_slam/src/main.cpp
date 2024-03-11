#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <mutex>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/OccupancyGrid.h>  
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

std::string frame_id_="scan";
//把点云转成scan时的最大最小角度和角度增量
float angle_min_=0;
float angle_max_=6.2831853;
float angle_increment_= 0.0034906585;
//把点云转成scan时的单帧时长
float scan_time_=0.1;
//把点云转成scan时的最大最小距离
float range_min_=0.7;
float range_max_=30;
//把点云转成scan时的距离最大值增加量，代表无效数据
float inf_epsilon_=1.0;
//把点云转成scan时的切取高度值
float  z_min_= -0.2;
float  z_max_= 0.0;
//栅格地图的分辨率
float  map_resolution_=0.1;
//栅格地图的初始大小
int  init_map_size_=400;
//栅格地图的扩大时的增长量
int  map_expand_size_=200;
//栅格地图的初始概率值
float  occupancy_center_add_=0.55;
float  occupancy_center_sub_=0.45;
//栅格地图的概率更新系数
float  occupancy_constant_add_=1.25;
float  occupancy_constant_sub_=0.75;

ros::Subscriber sublidrCloud;
ros::Publisher  pubLaserScan;
ros::Publisher  pubGridMap;

std::unique_ptr<tf::TransformListener> tfPrt;

int map_center_;
int map_size_;
std::vector<std::vector<float> > map_vector_(init_map_size_,std::vector<float>(init_map_size_,-1));

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msgIn)
{
    clock_t start_callback=clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msgIn, *lidarCloud);  

    sensor_msgs::LaserScan output;
    output.header = msgIn->header;
    output.header.frame_id = "body";
    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    int ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    std::vector<float> angel_range_vector(ranges_size,output.range_max + inf_epsilon_);
    std::vector<float> point_x_vector(ranges_size,output.range_max + inf_epsilon_);
    std::vector<float> point_y_vector(ranges_size,output.range_max + inf_epsilon_);
    for(auto& point:*lidarCloud)
    {
        float point_range=sqrt(pow(point.x,2)+pow(point.y,2));
        if(point.z>z_min_ && point.z<z_max_ && point_range>range_min_ && point_range<range_max_)
        {
            float angle = atan2(point.y, point.x);
            if(angle<0)
            {
                angle=angle+2*M_PI;
            }
            int angle_index=angle/angle_increment_;
            if(angle_index>=0 && angle_index<ranges_size)
            {
                if(point_range<angel_range_vector[angle_index])
                {
                    angel_range_vector[angle_index]=point_range;
                    point_x_vector[angle_index]=point.x;
                    point_y_vector[angle_index]=point.y;
                }
            }
        }
    }
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
    for(int i=0;i<angel_range_vector.size();i++)
    {
        output.ranges[i] = angel_range_vector[i];
    }

    pubLaserScan.publish(output);

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarScan(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i<ranges_size;i++)
    {
        pcl::PointXYZ pointxyz;
        pointxyz.x=point_x_vector[i];
        pointxyz.y=point_y_vector[i];
        pointxyz.z=0;
        (*lidarScan).push_back(pointxyz);
    }

    tf::Stamped<tf::Pose> poseident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),tf::Vector3(0,0,0)), msgIn->header.stamp, "body");
    tf::Stamped<tf::Pose> odom_pose;  
    try
    {
        if (!tfPrt->waitForTransform("camera_init", "body", msgIn->header.stamp,ros::Duration(0.5)))
        {
        }
        tfPrt->transformPose("camera_init", poseident, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("WARN, Failed to compute odom pose, lidarCallback skipping scan (%s)", e.what());
        return ;
    }    
    
    Eigen::Quaternionf pose_orientation;
    pose_orientation.w()=odom_pose.getRotation().w();
    pose_orientation.x()=odom_pose.getRotation().x();
    pose_orientation.y()=odom_pose.getRotation().y();
    pose_orientation.z()=odom_pose.getRotation().z();
    Eigen::Matrix3f pose_rotation = pose_orientation.toRotationMatrix();
    Eigen::Matrix4f pose_transformation= Eigen::Matrix4f::Identity();
    pose_transformation(0, 0) = pose_rotation(0, 0);
    pose_transformation(0, 1) = pose_rotation(0, 1);
    pose_transformation(0, 2) = pose_rotation(0, 2);
    pose_transformation(0, 3) =(odom_pose.getOrigin())[0];
    pose_transformation(1, 0) = pose_rotation(1, 0);
    pose_transformation(1, 1) = pose_rotation(1, 1);
    pose_transformation(1, 2) = pose_rotation(1, 2);
    pose_transformation(1, 3) = (odom_pose.getOrigin())[1];
    pose_transformation(2, 0) = pose_rotation(2, 0);
    pose_transformation(2, 1) = pose_rotation(2, 1);
    pose_transformation(2, 2) = pose_rotation(2, 2);
    pose_transformation(2, 3) = (odom_pose.getOrigin())[2];

    pcl::transformPointCloud (*lidarScan, *lidarScan, pose_transformation);    

    for(int i=0;i<ranges_size;i++)
    {
        if(angel_range_vector[i]<range_max_)
        {
            int index_x=((*lidarScan)[i].x-(-map_center_*map_resolution_))/map_resolution_;
            int index_y=((*lidarScan)[i].y-(-map_center_*map_resolution_))/map_resolution_;
            if(index_x<0 || index_x>=(2*map_center_) || index_y<0 || index_y>=(2*map_center_))
            {
                map_size_=map_size_+map_expand_size_*2;
                map_center_=map_size_/2;
                std::vector<std::vector<float> > new_map_vector_(map_size_,std::vector<float>(map_size_,-1));
                for(int j=0;j<map_vector_.size();j++)
                {
                    for(int k=0;k<map_vector_[j].size();k++)
                    {
                        new_map_vector_[j+map_expand_size_][k+map_expand_size_]=map_vector_[j][k];
                    }                    
                }
                int new_index_x=((*lidarScan)[i].x-(-map_center_*map_resolution_))/map_resolution_;
                int new_index_y=((*lidarScan)[i].y-(-map_center_*map_resolution_))/map_resolution_;   
                if(new_index_x>=0 && new_index_x<map_size_ && new_index_y>=0 && new_index_y<map_size_)        
                {
                    if(new_map_vector_[new_index_y][new_index_x]==-1)
                    {
                        new_map_vector_[new_index_y][new_index_x]=occupancy_center_add_;
                    }else
                    {
                        new_map_vector_[new_index_y][new_index_x]=new_map_vector_[new_index_y][new_index_x]*occupancy_constant_add_;
                    }
                    new_map_vector_[new_index_y][new_index_x]=new_map_vector_[new_index_y][new_index_x]<1?new_map_vector_[new_index_y][new_index_x]:1;
                }    
                new_map_vector_.swap(map_vector_);
                int index_center_x=((odom_pose.getOrigin())[0]-(-map_center_*map_resolution_))/map_resolution_;
                int index_center_y=((odom_pose.getOrigin())[1]-(-map_center_*map_resolution_))/map_resolution_;
                int numberX=abs(new_index_x-index_center_x);
                int numberY=abs(new_index_y-index_center_y);
                int numberXY=numberX>numberY?numberX:numberY;
                float distanceX=(*lidarScan)[i].x-(odom_pose.getOrigin())[0];
                float distanceY=(*lidarScan)[i].y-(odom_pose.getOrigin())[1];
                Eigen::Vector2f standVector=Eigen::Vector2f(distanceX,distanceY);
                standVector.normalize();
                if(distanceX!=0)
                {
                    if(numberX>numberY)
                    {
                        float standDistance=sqrt(pow(map_resolution_,2)+pow(map_resolution_*distanceY/distanceX,2));
                        standVector=standDistance*standVector;
                    }else
                    {
                        float standDistance=sqrt(pow(map_resolution_,2)+pow(map_resolution_*distanceX/distanceY,2)); 
                        standVector=standDistance*standVector;                       
                    }
                }else
                {
                    standVector=map_resolution_*standVector; 
                }
                for(int j=0;j<numberXY;j++)
                {
                    Eigen::Vector2f pointVector=j*standVector;
                    //std::cout<<standVector[0]<<' '<<standVector[1]<<' '<<pointVector[0]<<' '<<pointVector[1]<<std::endl;
                    int index_point_x=((odom_pose.getOrigin())[0]+pointVector[0]-(-map_center_*map_resolution_))/map_resolution_;
                    int index_point_y=((odom_pose.getOrigin())[1]+pointVector[1]-(-map_center_*map_resolution_))/map_resolution_;
                    if(index_point_x>=0 && index_point_x<map_size_ && index_point_y>=0 && index_point_y<map_size_ )
                    {
                        if(map_vector_[index_point_y][index_point_x]==-1)
                        {
                            map_vector_[index_point_y][index_point_x]=occupancy_center_sub_;
                        }else
                        {
                            map_vector_[index_point_y][index_point_x]=map_vector_[index_point_y][index_point_x]*occupancy_constant_sub_;
                        }
                        map_vector_[index_point_y][index_point_x]=map_vector_[index_point_y][index_point_x]>0?map_vector_[index_point_y][index_point_x]:0;
                    }
                }
            }else
            {
                if(index_x>=0 && index_x<map_size_ && index_y>=0 && index_y<map_size_)        
                {
                    if(map_vector_[index_y][index_x]==-1)
                    {
                        map_vector_[index_y][index_x]=occupancy_center_add_;
                    }else
                    {
                        map_vector_[index_y][index_x]=map_vector_[index_y][index_x]*occupancy_constant_add_;
                    }
                    map_vector_[index_y][index_x]=map_vector_[index_y][index_x]<1?map_vector_[index_y][index_x]:1;
                }    
                int index_center_x=((odom_pose.getOrigin())[0]-(-map_center_*map_resolution_))/map_resolution_;
                int index_center_y=((odom_pose.getOrigin())[1]-(-map_center_*map_resolution_))/map_resolution_;
                int numberX=abs(index_x-index_center_x);
                int numberY=abs(index_y-index_center_y);
                int numberXY=numberX>numberY?numberX:numberY;
                float distanceX=(*lidarScan)[i].x-(odom_pose.getOrigin())[0];
                float distanceY=(*lidarScan)[i].y-(odom_pose.getOrigin())[1];
                Eigen::Vector2f standVector=Eigen::Vector2f(distanceX,distanceY);
                standVector.normalize();
                if(distanceX!=0)
                {
                    if(numberX>numberY)
                    {
                        float standDistance=sqrt(pow(map_resolution_,2)+pow(map_resolution_*distanceY/distanceX,2));
                        standVector=standDistance*standVector;
                    }else
                    {
                        float standDistance=sqrt(pow(map_resolution_,2)+pow(map_resolution_*distanceX/distanceY,2)); 
                        standVector=standDistance*standVector;                       
                    }
                }else
                {
                    standVector=map_resolution_*standVector; 
                }
                for(int j=0;j<numberXY;j++)
                {
                    Eigen::Vector2f pointVector=j*standVector;
                    //std::cout<<"standVector: "<<standVector[0]<<' '<<standVector[1]<<" pointVector: "<<pointVector[0]<<' '<<pointVector[1]<<" odom_pose: "<<(odom_pose.getOrigin())[0]<<' '<<(odom_pose.getOrigin())[1]<<" lidarScan: "<<(*lidarScan)[i].x<<' '<<(*lidarScan)[i].y<<" pointNow"<<(odom_pose.getOrigin())[0]+pointVector[0]<<' '<<(odom_pose.getOrigin())[1]+pointVector[1]<<std::endl;
                    int index_point_x=((odom_pose.getOrigin())[0]+pointVector[0]-(-map_center_*map_resolution_))/map_resolution_;
                    int index_point_y=((odom_pose.getOrigin())[1]+pointVector[1]-(-map_center_*map_resolution_))/map_resolution_;
                    if(index_point_x>=0 && index_point_x<map_size_ && index_point_y>=0 && index_point_y<map_size_ )
                    {
                        if(map_vector_[index_point_y][index_point_x]==-1)
                        {
                            map_vector_[index_point_y][index_point_x]=occupancy_center_sub_;
                        }else
                        {
                            map_vector_[index_point_y][index_point_x]=map_vector_[index_point_y][index_point_x]*occupancy_constant_sub_;
                        }
                        map_vector_[index_point_y][index_point_x]=map_vector_[index_point_y][index_point_x]>0?map_vector_[index_point_y][index_point_x]:0;
                    }
                }
            }
        }
    }

    nav_msgs::OccupancyGrid occupancyGridMsg;
    occupancyGridMsg.info.resolution = map_resolution_;
    occupancyGridMsg.info.origin.position.x = -(map_size_/2)*map_resolution_;
    occupancyGridMsg.info.origin.position.y = -(map_size_/2)*map_resolution_;
    occupancyGridMsg.info.origin.position.z = 0.0;
    occupancyGridMsg.info.origin.orientation.x = 0.0;
    occupancyGridMsg.info.origin.orientation.y = 0.0;
    occupancyGridMsg.info.origin.orientation.z = 0.0;
    occupancyGridMsg.info.origin.orientation.w = 1.0;
    occupancyGridMsg.info.width = map_size_;
    occupancyGridMsg.info.height = map_size_;
    occupancyGridMsg.data.resize(occupancyGridMsg.info.width * occupancyGridMsg.info.height);
    for (int i = 0; i < occupancyGridMsg.info.width * occupancyGridMsg.info.height; i++)
    {
        int indexi=i%map_size_;
        int indexj=i/map_size_;
        if(map_vector_[indexj][indexi]>occupancy_center_add_)
        {
            occupancyGridMsg.data[i] =100;
        }else if(map_vector_[indexj][indexi]>=0 && map_vector_[indexj][indexi]<occupancy_center_sub_)
        {
            occupancyGridMsg.data[i] =0;
        }else
        {
            occupancyGridMsg.data[i] =-1;
        }
    }
    occupancyGridMsg.header.stamp = ros::Time::now();
    occupancyGridMsg.header.frame_id = "camera_init";
    pubGridMap.publish(occupancyGridMsg);     

    clock_t end_callback=clock();
    double callback_time=((double)(end_callback-start_callback)/(double)CLOCKS_PER_SEC);
    ROS_INFO("callback_time: %f ",callback_time);
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"simple_slam");
    ros::NodeHandle nh;

    nh.param<std::string>("simple_slam/frame_id_", frame_id_, "scan"); 
    nh.param<float>("simple_slam/angle_min_", angle_min_, 0); 
    nh.param<float>("simple_slam/angle_max_", angle_max_, 6.2831853); 
    nh.param<float>("simple_slam/angle_increment_", angle_increment_, 0.0034906585); 
    nh.param<float>("simple_slam/scan_time_", scan_time_, 0.1); 
    nh.param<float>("simple_slam/range_min_", range_min_,0.7); 
    nh.param<float>("simple_slam/range_max_", range_max_, 30); 
    nh.param<float>("simple_slam/inf_epsilon_", inf_epsilon_, 1.0); 
    nh.param<float>("simple_slam/z_min_", z_min_, -0.2); 
    nh.param<float>("simple_slam/z_max_", z_max_, 0.0); 
    nh.param<float>("simple_slam/map_resolution_", map_resolution_, 0.1); 
    nh.param<int>("simple_slam/init_map_size_", init_map_size_, 400); 
    nh.param<int>("simple_slam/map_expand_size_", map_expand_size_, 200); 
    nh.param<float>("simple_slam/occupancy_center_add_", occupancy_center_add_, 0.55);
    nh.param<float>("simple_slam/occupancy_center_sub_", occupancy_center_sub_, 0.45); 
    nh.param<float>("simple_slam/occupancy_constant_add_", occupancy_constant_add_, 1.25);
    nh.param<float>("simple_slam/occupancy_constant_sub_", occupancy_constant_sub_, 0.85); 

    std::cout<<"frame_id_: "<<frame_id_<<std::endl;    
    std::cout<<"angle_min_: "<<angle_min_<<std::endl;    
    std::cout<<"angle_max_: "<<angle_max_<<std::endl;    
    std::cout<<"angle_increment_: "<<angle_increment_<<std::endl;    
    std::cout<<"scan_time_: "<<scan_time_<<std::endl;   
    std::cout<<"range_min_: "<<range_min_<<std::endl;    
    std::cout<<"range_max_: "<<range_max_<<std::endl;   
    std::cout<<"inf_epsilon_: "<<inf_epsilon_<<std::endl;  
    std::cout<<"z_min_: "<<z_min_<<std::endl;   
    std::cout<<"z_max_: "<<z_max_<<std::endl;  
    std::cout<<"map_resolution_: "<<map_resolution_<<std::endl;  
    std::cout<<"init_map_size_: "<<init_map_size_<<std::endl;   
    std::cout<<"map_expand_size_: "<<map_expand_size_<<std::endl;
    std::cout<<"occupancy_center_add_: "<<occupancy_center_add_<<std::endl;  
    std::cout<<"occupancy_center_sub_: "<<occupancy_center_sub_<<std::endl;
    std::cout<<"occupancy_constant_add_: "<<occupancy_constant_add_<<std::endl;  
    std::cout<<"occupancy_constant_sub_: "<<occupancy_constant_sub_<<std::endl;

    map_center_=init_map_size_/2;;
    map_size_=init_map_size_;

    tfPrt=std::make_unique<tf::TransformListener>();

	sublidrCloud= nh.subscribe<sensor_msgs::PointCloud2>("/livox_pcl0", 100, lidarCallback);
    pubLaserScan = nh.advertise<sensor_msgs::LaserScan>("/scan", 1,true);
    pubGridMap = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    ros::spin();
    return 0;
}
