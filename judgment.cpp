#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <iterator>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

using std::string;
using std::vector;

ros::Publisher pub;
ros::Publisher pub_pose;
ros::Publisher pub_dist;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    auto cl = (*cloud);
    
    pcl::PointCloud<pcl::PointXYZ> test_cloud;
    string obj_position[cloud->points.size()];
    float obj_distance[cloud->points.size()];
    //auto pose = 'N';
    auto pose = 0;
    auto dist = 100;
    //std_msgs::String pose_array;
    std_msgs::UInt16MultiArray pose_array;
    std_msgs::Float32MultiArray dist_array;
    for(int i = 0; i < cloud->points.size(); i++)
    {
        //std::cout << "i = " << i << std::endl;
        if((cl[i].y > -1.8) && (cl[i].y < 1.8) && (cl[i].x > 0) && (cl[i].x < 30))
        {
            std::cout << "F" << std::endl;
            pose = 2;//'F';
            test_cloud.push_back(cl[i]);
        }
        else if((cl[i].y > 2) && (cl[i].y < 6) && (cl[i].x < 0) && (cl[i].x > -20))
        {
            std::cout << "BL" << std::endl;
            pose = 1;//'BL';
            test_cloud.push_back(cl[i]);
        }
        else if((cl[i].y < -2) && (cl[i].y > -6) && (cl[i].x < 0) && (cl[i].x > -20))
        {
            std::cout << "BR" << std::endl;
            pose = 3;//'BR';
            test_cloud.push_back(cl[i]);
        }
        else pose = 0;//'N';
        dist = sqrt(pow(cl[i].x, 2) + pow(cl[i].y, 2));
        pose_array.data.push_back(pose);
        obj_position[i] = pose;
        //
        dist_array.data.push_back(dist);
        obj_distance[i] = dist;
    }
    /*std::cout << "******************************************************" << std::endl;
    for(int i = 0; i < cloud->points.size(); i++)
    {
        std::cout << obj_position[i] << std::endl;
    }
    for(int j = 0; j < cloud->points.size(); j++)
    {
        std::cout << obj_distance[j] << std::endl;
    }
    std::cout << "******************************************************" << std::endl;*/
    pcl::PCLPointCloud2 center;
    pcl::toPCLPointCloud2(test_cloud, center);
    sensor_msgs::PointCloud2 output; 
    pcl_conversions::fromPCL(center, output);
    output.header.frame_id = "/velodyne"; 
    pub_pose.publish(pose_array);
    pub_dist.publish(dist_array);
    pub.publish(output); 

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_position_distance");
    ros::NodeHandle nh;
   
    ros::Subscriber sub = nh.subscribe("clustered_center_points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/object_judgment", 1);
    pub_pose = nh.advertise<std_msgs::UInt16MultiArray>("/obj_pose", 1);
    //pub_pose = nh.advertise<std_msgs::String>("/obj_pose", 1);
    pub_dist = nh.advertise<std_msgs::Float32MultiArray>("/obj_dist", 1);
    ros::spin();
}