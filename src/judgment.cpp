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
#include "std_msgs/Float32.h"

using std::string;
using std::vector;

ros::Publisher pub;
ros::Publisher pub_dist;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    auto cl = (*cloud);
    
    pcl::PointCloud<pcl::PointXYZ> test_cloud;

    auto obj_dist = 999;

    std_msgs::Float32 obj_state;
    for(int i = 0; i < cloud->points.size(); i++)
    {
    	
        //std::cout << "i = " << i << std::endl;
        if((cl[i].y > -1.8) && (cl[i].y < 1.8) && (cl[i].x > 0) && (cl[i].x < 30))
        {
            std::cout << "F" << std::endl;
            obj_dist = sqrt(pow(cl[i].x, 2) + pow(cl[i].y, 2));
            test_cloud.push_back(cl[i]);
        }
       
    }
    obj_state.data = obj_dist;
    /*std::cout << "******************************************************" << std::endl;
    std::cout << "******************************************************" << std::endl;*/
    pcl::PCLPointCloud2 center;
    pcl::toPCLPointCloud2(test_cloud, center);
    sensor_msgs::PointCloud2 output; 
    pcl_conversions::fromPCL(center, output);
    output.header.frame_id = "/velodyne"; 
    pub_dist.publish(obj_state);
    pub.publish(output); 

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object__distance");
    ros::NodeHandle nh;
   
    ros::Subscriber sub = nh.subscribe("clustered_center_points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/object_judgment", 1);
    pub_dist = nh.advertise<std_msgs::Float32>("/obj_state", 1);
    ros::spin();
}
