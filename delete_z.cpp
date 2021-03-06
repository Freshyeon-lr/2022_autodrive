#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

ros::Publisher pub;

void
callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::fromROSMsg(*msg, *cloud_filtered);
	pcl::fromROSMsg(*msg, *cloud);
	  // 오브젝트 생성 
	pcl::PassThrough<pcl::PointXYZ> pass;
	pcl::PassThrough<pcl::PointXYZ> passy;
	pcl::PassThrough<pcl::PointXYZ> passx;
	pass.setInputCloud(cloud);//입력
	passy.setInputCloud(cloud);//입력
	passx.setInputCloud(cloud);
	pass.setFilterFieldName("z");//적용할 좌표 축 (eg. Z축)
	pass.setFilterLimits(-1.3, 50.0);//적용할 값 (최소, 최대 값)
	passy.setFilterFieldName("y");//적용할 좌표 축 (eg. Z축)
	passy.setFilterLimits(-20.0, 20.0);//적용할 값 (최소, 최대 값)
	passx.setFilterFieldName("x");
	passx.setFilterLimits(-30, 50);

//	pass.setFilterFieldName("y");//적용할 좌표 축 (eg. Z축)
//	pass.setFilterLimits(-100.0, 100.0);//적용할 값 (최소, 최대 값)
//	pass.setFilterFieldName("x");//적용할 좌표 축 (eg. Z축)
//	pass.setFilterLimits(-100.0, 100.0);//적용할 값 (최소, 최대 값)
	pass.filter(*cloud);//필터 적용
	passy.filter(*cloud);//필터 적용
	passx.filter(*cloud);
	//for(size_t i = 0; i<cloud->points.size(); ++i)
	//{
	//	std::cout << "x: " << cloud -> points[i].x ;
	//	std::cout << "  y:" << cloud -> points[i].y ;
	//	std::cout << "  z:" << cloud -> points[i].z << std::endl;
	//}
	//c = pass.filter(cloud_filtered);
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud, output);
	pub.publish(output);
}

int
 main (int argc, char** argv)
{
	ros::init(argc, argv, "delete_ground");
	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::PointCloud2>("delete_ground_combined",1);
	ros::Subscriber sub = nh.subscribe("Combined_velo", 1, callback);
	ros::spin();
}
