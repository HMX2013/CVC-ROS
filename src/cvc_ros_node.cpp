#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include "obsdet_msgs/CloudCluster.h"
#include "obsdet_msgs/CloudClusterArray.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION == 3)
  #include <opencv/cv.h>
#else
  #include <opencv2/imgproc.hpp>
#endif

#include <dynamic_reconfigure/server.h>
#include <cvc_ros/cvc_ros_Config.h>
#include "cvc.hpp"
#include "tools/utils.hpp"
#include "ground_truth.hpp"

// using namespace cv;
using PointType = PointXYZIRLICO;

static ros::Publisher pub_jskrviz_time_;
static std_msgs::Float32 time_spent;

std::string output_frame_;
std::string non_ground_cloud_;
std_msgs::Header rosmsg_header_;

// Pointcloud Filtering Parameters
float DELTA_A, DELTA_P, DELTA_R;


boost::shared_ptr<groundtruth::DepthCluster<PointType>> gt_verify;

class cloud_segmentation
{
 private:
  ros::NodeHandle nh;

  pcl::PointCloud<PointType>::Ptr laserCloudIn;

  dynamic_reconfigure::Server<cvc_ros::cvc_ros_Config> server;
  dynamic_reconfigure::Server<cvc_ros::cvc_ros_Config>::CallbackType f;

  pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloudColor;

  ros::Subscriber sub_lidar_points_;

  ros::Publisher pub_obsdet_clusters_;
  ros::Publisher pubSegmentedCloudColor_;
  ros::Publisher pubgt_pc_;

  std::vector<std::vector<int> > clusterIndices;
  std::vector<std::vector<int> > objectIndices;

  std::vector<std::vector<int> > gt_clusterIndices;

  std::vector<int> runtime_vector;
  std::vector<double> use_vector;
  std::vector<double> ose_vector;

  std::shared_ptr<CVC> cvc;

  void preprocessing(const sensor_msgs::PointCloud2::ConstPtr& laserRosCloudMsg);
  void MainLoop(const sensor_msgs::PointCloud2::ConstPtr& lidar_points);
  void eval_running_time(int running_time);
  void eval_USE();
  void eval_OSE();

  void copyPointCloudCEC(pcl::PointCloud<PointType> &src, pcl::PointCloud<pcl::PointXYZINormal> &dst);

  void CVC_CLUSTER(const pcl::PointCloud<PointType>::Ptr in_cloud_ptr,
                   const float deltaA, const float deltaR, const float deltaP);
  void postSegment(obsdet_msgs::CloudClusterArray &in_out_cluster_array);

 public:
  cloud_segmentation();
  ~cloud_segmentation() {};

  void allocateMemory(){
    segmentedCloudColor.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudIn.reset(new pcl::PointCloud<PointType>());
  }

  void resetParameters(){
    segmentedCloudColor->clear();
    laserCloudIn->clear();
    clusterIndices.clear();
    gt_clusterIndices.clear();
    objectIndices.clear();
  }

};

// Dynamic parameter server callback function
void dynamicParamCallback(cvc_ros::cvc_ros_Config& config, uint32_t level)
{
  // Pointcloud Filtering Parameters
  DELTA_A=config.delta_a;
  DELTA_P=config.delta_p;
  DELTA_R=config.delta_r;
}


cloud_segmentation::cloud_segmentation()
{
  ros::NodeHandle private_nh("~");

  runtime_vector.clear();
  use_vector.clear();
  ose_vector.clear();

  std::string cloud_ground_topic;
  std::string cloud_clusters_topic;
  std::string jsk_bboxes_topic;

  private_nh.param<std::string>("non_ground_cloud", non_ground_cloud_, "/points_no_ground");
  private_nh.param<std::string>("output_frame", output_frame_, "velodyne");

  sub_lidar_points_ = nh.subscribe(non_ground_cloud_, 1, &cloud_segmentation::MainLoop, this);
  pubSegmentedCloudColor_ = nh.advertise<sensor_msgs::PointCloud2> ("/segmentation/segmented_cloud_color", 1);

  pub_obsdet_clusters_ = nh.advertise<obsdet_msgs::CloudClusterArray>("/clustering/cluster_array", 1);

  pubgt_pc_= nh.advertise<sensor_msgs::PointCloud2> ("/segmentation/gt_pc", 1);

  pub_jskrviz_time_ = nh.advertise<std_msgs::Float32>("/cvc/time_cvc", 1);

  // Dynamic Parameter Server & Function
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  // Create point processor
  cvc = std::make_shared<CVC>();

  allocateMemory();
  resetParameters();
}

void cloud_segmentation::CVC_CLUSTER(const pcl::PointCloud<PointType>::Ptr in_cloud_ptr,
                                     const float deltaA, const float deltaR, const float deltaP)
{
  std::vector<pcl::PointCloud<PointType>::Ptr> clusters;

  cvc->set_deltaA(deltaA);
  cvc->set_deltaR(deltaR);
  cvc->set_deltaP(deltaP);
  std::vector<PointAPR> capr;
  cvc->calculateAPR(*in_cloud_ptr, capr);

	std::unordered_map<int, Voxel> hash_table;
	cvc->build_hash_table(capr, hash_table);
	std::vector<int> cluster_indices;
	cluster_indices = cvc->cluster(hash_table, capr);
	std::vector<int> cluster_id;
	cvc->most_frequent_value(cluster_indices, cluster_id);

  uint16_t cid_index = 0;

  for (int j = 0; j < cluster_id.size(); ++j)
  {
    std::vector<int> clusterIndice;
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
      if (cluster_indices[i] == cluster_id[j])
      {
        clusterIndice.push_back(i);
        laserCloudIn->points[i].cid = cid_index;
      }
    }
    clusterIndices.push_back(clusterIndice);
    cid_index++;
  }
}


void cloud_segmentation::eval_USE()
{
  std::vector<int> cluater_label;
  std::vector<std::vector<int> > cluaters_label;

  int label[34] = {0, 1, 10, 11, 13, 15, 16, 18, 20, 30, 31, 32, 40, 44, 48, 49, 50, 51,
                   52, 60, 70, 71, 72, 80, 81, 99, 252, 253, 254, 255, 256, 257, 258, 259};

  double use_i_sum = 0;

  for (auto& getIndices : clusterIndices)
  {
    int cluster_label[34] = {0};
    for (auto& index : getIndices)
    {
      for (size_t i = 0; i < 34; i++)
      {
        if (laserCloudIn->points[index].label == label[i])
          cluster_label[i]++;
      }
    }

    int M = getIndices.size();

    for (size_t i = 0; i < 34; i++)
    {
      if (cluster_label[i] == 0)
        continue;

      double use_i = -(cluster_label[i] / (M * 1.0)) * log(cluster_label[i] / (M * 1.0));
      use_i_sum += use_i;
    }
  }

  use_vector.push_back(use_i_sum);  

  double use_total_v = 0.0;
  double use_sqr_sum = 0.0;
  double use_mean;
  double use_std;

  for (size_t i = 0; i < use_vector.size(); i++)
  {
    use_total_v += use_vector[i];
  }
  use_mean = use_total_v / use_vector.size();

  for (size_t i = 0; i < use_vector.size(); i++)
  {
    use_sqr_sum += (use_vector[i] - use_mean) * (use_vector[i] - use_mean);
  }

  use_std = sqrt(use_sqr_sum / use_vector.size());

  std::cout << "current use_i is = " << use_i_sum << std::endl;
  std::cout << "\033[1;32muse_mean is = " << use_mean << "\033[0m" << std::endl;
  std::cout << "use_std is = " << use_std << std::endl;
}


void cloud_segmentation::eval_OSE()
{
  double ose_i = 0.0;

  int cluster_id = 0;
  for (auto &getIndices : clusterIndices)
  {
    for (auto &index : getIndices)
    {
      laserCloudIn->points[index].cid = cluster_id;
    }
    cluster_id++;
  }

  for (auto &getIndices : gt_clusterIndices)
  {
    int object_cluster[clusterIndices.size()] = {0};
    int N = getIndices.size();

    for (auto &index : getIndices)
    {
      if (laserCloudIn->points[index].cid != 9999)
        object_cluster[laserCloudIn->points[index].cid]++;
    }

    for (size_t i = 0; i < clusterIndices.size(); i++)
    {
      if (object_cluster[i] == 0)
        continue;

      double ose_ii = -(object_cluster[i] / (1.0*N)) * log(object_cluster[i] / (1.0*N));

      ose_i += ose_ii;
    }
  }

  ose_vector.push_back(ose_i);

  std::cout << "ose_vector.size() is = " << ose_vector.size() << std::endl;

  double ose_total_v = 0.0;
  double ose_sqr_sum = 0.0;
  double ose_mean;
  double ose_std;

  for (size_t i = 0; i < ose_vector.size(); i++)
  {
    ose_total_v += ose_vector[i];
  }
  ose_mean = ose_total_v / ose_vector.size();

  for (size_t i = 0; i < ose_vector.size(); i++)
  {
    ose_sqr_sum += (ose_vector[i] - ose_mean) * (ose_vector[i] - ose_mean);
  }

  ose_std = sqrt(ose_sqr_sum / ose_vector.size());

  std::cout << "current ose_i is = " << ose_i << std::endl;
  std::cout << "\033[1;34mose_mean is = " << ose_mean << "\033[0m" << std::endl;
  std::cout << "ose_std is = " << ose_std << std::endl;
}


void cloud_segmentation::eval_running_time(int running_time)
{
  double runtime_std;
  double runtime_aver;
  double runtime_total_v = 0.0;
  double runtime_sqr_sum = 0.0;

  runtime_vector.push_back(running_time);

  for (size_t i = 0; i < runtime_vector.size(); i++)
  {
    runtime_total_v += runtime_vector[i];
  }

  runtime_aver = runtime_total_v / runtime_vector.size();

  for (size_t i = 0; i < runtime_vector.size(); i++)
  {
    runtime_sqr_sum += (runtime_vector[i] - runtime_aver) * (runtime_vector[i] - runtime_aver);
  }

  runtime_std = sqrt(runtime_sqr_sum / runtime_vector.size());

  std::cout << "current running_time is = " << running_time << "ms" << std::endl;
  std::cout << "\033[1;36mruntime_aver is = " << runtime_aver << "ms"
            << "\033[0m" << std::endl;
  std::cout << "runtime_std is = " << runtime_std << "ms" << std::endl;
}

void cloud_segmentation::preprocessing(const sensor_msgs::PointCloud2::ConstPtr& laserRosCloudMsg){
  rosmsg_header_ = laserRosCloudMsg->header;
  rosmsg_header_.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line

  pcl::PointCloud<PointType>::Ptr raw_pcl(new pcl::PointCloud<PointType>());

  pcl::fromROSMsg(*laserRosCloudMsg, *raw_pcl);

  size_t cloudSize = raw_pcl->points.size();
  PointType this_pt;
  laserCloudIn->clear();
  
  int scan_num=0;
  for (size_t i = 0; i < cloudSize; ++i)
  {
    this_pt.x = raw_pcl->points[i].x;
    this_pt.y = raw_pcl->points[i].y;
    this_pt.z = raw_pcl->points[i].z;
    this_pt.intensity = raw_pcl->points[i].intensity;
    this_pt.id = raw_pcl->points[i].id;
    this_pt.label = raw_pcl->points[i].label;
    this_pt.cid = 9999;

    bool is_nan = std::isnan(this_pt.x) || std::isnan(this_pt.y) || std::isnan(this_pt.z);
    if (is_nan)
      continue;
    laserCloudIn->points.push_back(this_pt);
  }
}

void cloud_segmentation::postSegment(obsdet_msgs::CloudClusterArray &in_out_cluster_array)
{
  unsigned int intensity_mark = 1;
  
  pcl::PointXYZI cluster_color;
  sensor_msgs::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2 colored_cluster_ros;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_pcl(new pcl::PointCloud<pcl::PointXYZI>);

  // clusterIndices  objectIndices

  for (auto& getIndices : clusterIndices)
  {
    cluster_pcl->clear();
    for (auto& index : getIndices){
      cluster_color.x = laserCloudIn->points[index].x;
      cluster_color.y = laserCloudIn->points[index].y;
      cluster_color.z = laserCloudIn->points[index].z;
      cluster_color.intensity = intensity_mark;
      cluster_pcl->push_back(cluster_color);
      segmentedCloudColor->push_back(cluster_color);
    }

    intensity_mark++;
    cloud_msg.header = rosmsg_header_;
    pcl::toROSMsg(*cluster_pcl, cloud_msg);

    obsdet_msgs::CloudCluster cluster_;
    cluster_.header = rosmsg_header_;
    cluster_.cloud = cloud_msg;
    in_out_cluster_array.clusters.push_back(cluster_);
  }

  in_out_cluster_array.header = rosmsg_header_;
  pub_obsdet_clusters_.publish(in_out_cluster_array);

  pcl::toROSMsg(*segmentedCloudColor, colored_cluster_ros);
  colored_cluster_ros.header.frame_id = rosmsg_header_.frame_id;
  colored_cluster_ros.header.stamp = ros::Time::now();
  pubSegmentedCloudColor_.publish(colored_cluster_ros);
}


void cloud_segmentation::MainLoop(const sensor_msgs::PointCloud2::ConstPtr& lidar_points)
{
  gt_verify.reset(new groundtruth::DepthCluster<PointType>());

  obsdet_msgs::CloudClusterArray cluster_array;

  preprocessing(lidar_points);

  gt_verify->GTV(laserCloudIn, gt_clusterIndices);

  std::vector<int> cluster_indices;

  const auto start_time = std::chrono::steady_clock::now();

  CVC_CLUSTER(laserCloudIn, DELTA_A, DELTA_R, DELTA_P);

  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  eval_running_time(elapsed_time.count());

  eval_USE();
  eval_OSE();

  // extract segmented cloud for visualization
  postSegment(cluster_array);

  time_spent.data = elapsed_time.count();
  pub_jskrviz_time_.publish(time_spent);

  resetParameters();
  std::cout << "--------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cvc_ros_node");

  cloud_segmentation cloud_segmentation_node;

  ros::spin();

  return 0;
}