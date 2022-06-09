#include <ros/ros.h>
#include <iterator>
#include <typeinfo>
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
#include <pcl/common/centroid.h>

// Euclidean Cluster Extraction
// http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

ros::Publisher pub;
ros::Publisher pub2;
using std::copy; using std::ostream_iterator;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

  // 탐색을 위한 KdTree 오브젝트 생성 //Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);  //KdTree 생성 


  std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
  // 군집화 오브젝트 생성  
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud (cloud);       // 입력   
  ec.setClusterTolerance (2);  // 2cm  0.02
  ec.setMinClusterSize (50);     // 최소 포인트 수 100
  ec.setMaxClusterSize (20000);   // 최대 포인트 수 25000
  ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정 
  ec.extract (cluster_indices);   // 군집화 적용 
  
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;///1
  //copy(cluster_indices.begin(), cluster_indices.end(), ostream_iterator<pcl::PointIndices>(std::cout, "; "));
  //std::cout << std::endl;
  //std::cout << "cluster_indices.size 11111 ====>   " << cluster_indices.size() << std::endl;
  //std::cout << "cluster_indices" << cluster_indices.data () << std::endl;//0x559d14587c60
  //std::cout << typeid(cluster_indices.data ()).name() << std::endl;//PN3pcl12PointIndicesE
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  
  // 클러스터별 정보 수집, 출력, 저장 
  int j = 0;
  auto x_sum = 0;
  auto y_sum = 0;
  auto z_sum = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud_center;
  pcl::PointXYZ c1;
  //cluster
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    
    //pcl::PointXYZ c1;
    //std::cout << "```````````````````````````````````````````````````````````````" << std::endl;///1234567
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //points in each cluster
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      //std::cout << "pit = " << *pit << std::endl;
      cloud_cluster->points.push_back (cloud->points[*pit]);
      //std::cout << "wowowowowowowowo" << cloud->points[*pit] << std::endl;//(25.112,38.0353,12.6797)
      //std::cout << "cloud == " << *cloud << std::endl;
      pcl::PointXYZ pt = cloud->points[*pit];
      pcl::PointXYZI pt2;
      pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
      pt2.intensity = (float)(j + 1);

      TotalCloud.push_back(pt2);
      //std::cout << "11111111111111111111111111111111111111111111111111111111" << std::endl;//1234567
      //std::cout << "cluster_indices.size ====>   " << cluster_indices.size() << std::endl;  
      
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    
    for(int i = 0; i < cloud_cluster->points.size(); i++)
    {
      centroid.add(pcl::PointXYZ ((*cloud_cluster)[i].x, (*cloud_cluster)[i].y, (*cloud_cluster)[i].z));
      /*std::cout << "x ************** " << (*cloud_cluster)[i].x << std::endl;
      std::cout << "y ************** " << (*cloud_cluster)[i].y << std::endl;
      std::cout << "z ************** " << (*cloud_cluster)[i].z << std::endl;*/
    }
    
    centroid.get(c1);
    //std::cout << "===========================================================" << std::endl;
    //std::cout << c1.x << "     "<< c1.y << "      " << c1.z << std::endl;
    //std::cout << "===========================================================" << std::endl;
    cloud_center.push_back(c1);

    //std::cout << "cloud_cluster->width ---> " << (*cloud_cluster)[40].y << std::endl;
    //std::cout << "cloud_cluster->width(cluster size) ---> " << cloud_cluster->points.size() << std::endl;
    //std::cout << "cloud_cluster print ---> " << *(cloud_cluster)->points.data() << std::endl;
    //std::cout << "cluster_indices.size 2222222 ====>   " << cluster_indices.size() << std::endl;
    //std::stringstream ss;
    //ss << "cloud_cluster_" << j << ".pcd";
    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
    j++;
    //std::cout << "PointCloud representing the Cluster: " << cloud->points.size () << " data points." << std::endl;
    //std::cout << "----------------------finish for loop--------------------------------------" << std::endl;//1234567
  }

  // 포인트수 출력
  //std::cout << "PointCloud representing the Cluster: " << cloud->points.size () << " data points." << std::endl;
  //std::cout << "**************************aaaaaaaaaaaaaaaa***************************" << std::endl;//1
  pcl::PCLPointCloud2 cloud_p;
  pcl::PCLPointCloud2 center;
  pcl::toPCLPointCloud2(cloud_center, center);
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);//transform to PointCloud2 of sensor_msgs 
  //std::cout << "TotalCloud = " << TotalCloud.data () << std::endl;
  sensor_msgs::PointCloud2 output; 
  sensor_msgs::PointCloud2 output_center;
  pcl_conversions::fromPCL(center, output_center);
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "/velodyne";  
  output_center.header.frame_id = "/velodyne"; 
  pub.publish(output_center);
  pub2.publish(output); 
 
  //ROS_INFO("published it.");
}


int 
main (int argc, char** argv)
{
  ros::init(argc, argv, "cluster");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  ros::Subscriber sub = nh.subscribe("/delete_ground_combined", 1, callback);
  ////ros::Subscriber sub = nh.subscribe("/Combined_velo", 1, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_center_points",1);
  pub2 = nh2.advertise<sensor_msgs::PointCloud2>("/clustered_points",1);
  
  ros::spin();
}

