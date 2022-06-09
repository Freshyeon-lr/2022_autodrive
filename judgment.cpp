#include <iostream>
#include <cmath>
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

ros::Publisher pub;
auto distance = 1000;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    auto cl = (*cloud);
    
    pcl::PointCloud<pcl::PointXYZ> test_cloud;
    for(int i = 0; i < cloud->points.size(); i++)
    {
        std::cout << "sub msg x = " << cl[i].x << std::endl;
        if((cl[i].y > -1.8) && (cl[i].y < 1.8) && (cl[i].x > 0) && (cl[i].x < 30))
        {
            std::cout << "F" << std::endl;
            test_cloud.push_back(cl[i]);
            distance = sqrt(pow(cl[i].x, 2) + pow(cl[i].y, 2));
            std::cout << "distance = " << distance << std::endl;
        }
        else if((cl[i].y > 2) && (cl[i].y < 6) && (cl[i].x < 0) && (cl[i].x > -20))
        {
            std::cout << "BL" << std::endl;
            test_cloud.push_back(cl[i]);
            distance = sqrt(pow(cl[i].x, 2) + pow(cl[i].y, 2));
            std::cout << "distance = " << distance << std::endl;
        }
        else if((cl[i].y < -2) && (cl[i].y > -6) && (cl[i].x < 0) && (cl[i].x > -20))
        {
            std::cout << "BR" << std::endl;
            test_cloud.push_back(cl[i]);
            distance = sqrt(pow(cl[i].x, 2) + pow(cl[i].y, 2));
            std::cout << "distance = " << distance << std::endl;
        }
    }
    pcl::PCLPointCloud2 center;
    pcl::toPCLPointCloud2(test_cloud, center);
    sensor_msgs::PointCloud2 output; 
    pcl_conversions::fromPCL(center, output);
    output.header.frame_id = "/velodyne"; 
    pub.publish(output); 

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_position_distance");
    ros::NodeHandle nh;
   
    ros::Subscriber sub = nh.subscribe("clustered_center_points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/object_judgment", 1);
    ros::spin();
}