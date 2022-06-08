#include <ros/ros.h>
#include <iterator>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Euclidean Cluster Extraction
// http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

ros::Publisher pub;
using std::copy; using std::ostream_iterator;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

  // 탐색을 위한 KdTree 오브젝트 생성 //Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);  //KdTree 생성 


  std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
  // 군집화 오브젝트 생성  
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud (cloud);       // 입력   
  ec.setClusterTolerance (2);  // 2cm  0.02
  ec.setMinClusterSize (100);     // 최소 포인트 수 
  ec.setMaxClusterSize (25000);   // 최대 포인트 수
  ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정 
  ec.extract (cluster_indices);   // 군집화 적용 
  
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
  copy(cluster_indices.begin(), cluster_indices.end(), ostream_iterator<pcl::PointIndices>(std::cout, "; "));
  std::cout << std::endl;
  std::cout << "cluster_indices" << cluster_indices.data () << std::endl;
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  
  // 클러스터별 정보 수집, 출력, 저장 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      pcl::PointXYZ pt = cloud->points[*pit];
      pcl::PointXYZI pt2;
      pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
      pt2.intensity = (float)(j + 1);

      TotalCloud.push_back(pt2);
    } 
    j++;
  }

  // 포인트수 출력
  std::cout << "PointCloud representing the Cluster: " << cloud->points.size () << " data points." << std::endl;
  
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "/velodyne";   
  pub.publish(output); 
 
  ROS_INFO("published it.");
    //sensor_msgs::PointCloud2 cluster_output;
    //pcl::toROSMsg(*cloud_cluster, cluster_output);
    //pub.publish(cluster_output);

    // 클러스터별 이름 생성 및 저장 
    //std::stringstream ss;
    //ss << "cloud_cluster_" << j << ".pcd";
    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //j++;
  
}


int 
main (int argc, char** argv)
{
  ros::init(argc, argv, "cluster");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/delete_ground_combined", 1, callback);
  ////ros::Subscriber sub = nh.subscribe("/Combined_velo", 1, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_points",1);
  
  ros::spin();
}
/*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("RANSAC_plane_true.pcd", *cloud);

  // 포인트수 출력
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // 탐색을 위한 KdTree 오브젝트 생성 //Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);  //KdTree 생성 


  std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
  // 군집화 오브젝트 생성  
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setInputCloud (cloud);       // 입력   
  ec.setClusterTolerance (0.02);  // 2cm  
  ec.setMinClusterSize (100);     // 최소 포인트 수 
  ec.setMaxClusterSize (25000);   // 최대 포인트 수
  ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정 
  ec.extract (cluster_indices);   // 군집화 적용 

  // 클러스터별 정보 수집, 출력, 저장 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
     cloud_cluster->points.push_back (cloud->points[*pit]); 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // 포인트수 출력
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    // 클러스터별 이름 생성 및 저장 
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);*/
