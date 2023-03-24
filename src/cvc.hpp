#ifndef CVC_CLUSTER_H
#define CVC_CLUSTER_H

#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include "tools/utils.hpp"
using PointType = PointXYZIRLICO;

template <typename T>
std::string toString(const T &t)
{
   std::ostringstream oss;
   oss << t;
   return oss.str();
}

struct PointAPR
{
   float azimuth;
   float polar_angle;
   float range;
   float range_horizon;
};

struct Voxel
{
   bool haspoint = false;
   int cluster = -1;
   std::vector<int> index;
};

class CVC
{
public:

   void set_deltaA(float deltaA)
   {
      deltaA_ = deltaA;
   }

   void set_deltaR(float deltaR)
   {
      deltaR_ = deltaR;
   }

   void set_deltaP(float deltaP)
   {
      deltaP_ = deltaP;
   }

   void calculateAPR(const pcl::PointCloud<PointType> &cloud_in, std::vector<PointAPR> &vapr);
   void build_hash_table(const std::vector<PointAPR> &vapr, std::unordered_map<int, Voxel> &map_out);
   void find_neighbors(int azimuth, int range, int polar, std::vector<int> &neighborindex);
   bool most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index);
   void mergeClusters(std::vector<int> &cluster_indices, int idx1, int idx2);
   std::vector<int> cluster(std::unordered_map<int, Voxel> &map_in, const std::vector<PointAPR> &vapr);
   void process();

private:
   //azimuth angle
   float deltaA_ = 0.4;

   //cvc range
   float deltaR_ = 0.3;

   //polar angle
   float deltaP_ = 4.0;

   float min_range_ = std::numeric_limits<float>::max();
   float max_range_ = std::numeric_limits<float>::min();
   float min_polar_ = -25.0 * M_PI / 180;
   float max_polar_ = 3.0 * M_PI / 180;

   int length_ = 0;
   int width_ = 0;
   int height_ = 0;
};


bool compare_cluster(std::pair<int,int> a,std::pair<int,int> b){
    return a.second>b.second;
}//upper sort

float azimuth_angle_cal(float x, float y)
{
   float temp_tangle = 0;
   if (x == 0 && y == 0)
   {
      temp_tangle = 0;
   }
   else if (y >= 0)
   {
      temp_tangle = (float)atan2(y, x);
   }
   else if (y < 0)
   {
      temp_tangle = (float)atan2(y, x) + 2 * M_PI;
   }
   return temp_tangle;
}

void CVC::calculateAPR(const pcl::PointCloud<PointType>& cloud_in, std::vector<PointAPR>& vapr){
   for (int i =0; i<cloud_in.points.size(); ++i){
      PointAPR parp;
      parp.azimuth = azimuth_angle_cal(cloud_in.points[i].x, cloud_in.points[i].y);
      parp.range = sqrt(cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z);
      parp.range_horizon = sqrt(cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y);
      parp.polar_angle = (float)atan2(cloud_in.points[i].z, parp.range_horizon);

      if(parp.range < min_range_){
         min_range_ = parp.range;
      }
      if(parp.range > max_range_){
         max_range_ = parp.range;
      }
      vapr.push_back(parp);
   }

	length_ = int((max_range_ - min_range_)/deltaR_)+1;
	width_  = round(360/deltaA_);
	height_ = int(((max_polar_ - min_polar_)*180/M_PI)/deltaP_)+1;
}

void CVC::build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out)
{
   for(int i =0; i< vapr.size(); ++i){
      int azimuth_index = int(vapr[i].azimuth*180/M_PI/deltaA_);      
      int range_index = int((vapr[i].range - min_range_)/deltaR_);
      int polar_index = int(((vapr[i].polar_angle-min_polar_)*180/M_PI)/deltaP_);
      int voxel_index = (azimuth_index*(length_)+range_index)+polar_index*(length_)*(width_);

      std::unordered_map<int, Voxel>::iterator it_find;
      it_find = map_out.find(voxel_index);
      if (it_find != map_out.end())
      {
         it_find->second.index.push_back(i);
      }
      else
      {
         Voxel vox;
         vox.haspoint = true;
         vox.index.push_back(i);
         vox.index.swap(vox.index);
         map_out.insert(std::make_pair(voxel_index, vox));
      }
   }
}

void CVC::find_neighbors(int azimuth, int range, int polar, std::vector<int> &neighborindex)
{
   for (int z = polar - 1; z <= polar + 1; z++)
   {
      if (z < 0 || z > (height_ - 1))
      {
         continue;
      }

      for (int y = range - 1; y <= range + 1; y++)
      {
         if (y < 0 || y > (length_ - 1))
         {
            continue;
         }
         for (int x = azimuth - 1; x <= azimuth + 1; x++)
         {
            int px = x;
            if (x < 0)
            {
               px = width_ - 1;
            }
            if (x > (width_ - 1))
            {
               px = 0;
            }
            neighborindex.push_back((px * (length_) + y) + z * (length_) * (width_));
         }
      }
   }
}

std::vector<int> CVC::cluster(std::unordered_map<int, Voxel> &map_in, const std::vector<PointAPR> &vapr)
{
   int current_cluster = 0;

   std::vector<int> cluster_indices = std::vector<int>(vapr.size(), -1);

   for (int i = 0; i < vapr.size(); ++i)
   {
      if (cluster_indices[i] != -1)
         continue;

      int azimuth_index = int(vapr[i].azimuth * 180 / M_PI / deltaA_);
      int range_index = int((vapr[i].range - min_range_) / deltaR_);
      int polar_index = int(((vapr[i].polar_angle - min_polar_) * 180 / M_PI) / deltaP_);
      int voxel_index = (azimuth_index * (length_) + range_index) + polar_index * (length_) * (width_);

      std::unordered_map<int, Voxel>::iterator it_find;
      std::unordered_map<int, Voxel>::iterator it_find2;

      it_find = map_in.find(voxel_index);
      std::vector<int> neightbors;

      if (it_find != map_in.end())
      {
         std::vector<int> neighborid;
         find_neighbors(azimuth_index, range_index, polar_index, neighborid);
         for (int k = 0; k < neighborid.size(); ++k)
         {
            it_find2 = map_in.find(neighborid[k]);

            if (it_find2 != map_in.end())
            {
               for (int j = 0; j < it_find2->second.index.size(); ++j)
               {
                  neightbors.push_back(it_find2->second.index[j]);
               }
            }
         }
      }

      neightbors.swap(neightbors);

      if (neightbors.size() > 0)
      {
         for (int j = 0; j < neightbors.size(); ++j)
         {
            int oc = cluster_indices[i];
            int nc = cluster_indices[neightbors[j]];
            if (oc != -1 && nc != -1)
            {
               if (oc != nc)
                  mergeClusters(cluster_indices, oc, nc);
            }
            else
            {
               if (nc != -1)
               {
                  cluster_indices[i] = nc;
               }
               else
               {
                  if (oc != -1)
                  {
                     cluster_indices[neightbors[j]] = oc;
                  }
               }
            }
         }
      }

      if (cluster_indices[i] == -1)
      {
         current_cluster++;
         cluster_indices[i] = current_cluster;
         for (int s = 0; s < neightbors.size(); ++s)
         {
            cluster_indices[neightbors[s]] = current_cluster;
         }
      }
   }
   return cluster_indices;
}

void CVC::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2)
{
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}

bool CVC::most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index)
{
   std::unordered_map<int, int> histcounts;
   for (int i = 0; i < values.size(); i++)
   {
      if (histcounts.find(values[i]) == histcounts.end())
      {
         histcounts[values[i]] = 1;
      }
      else
      {
         histcounts[values[i]] += 1;
      }
   }
   int max = 0, maxi;
   std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
   sort(tr.begin(), tr.end(), compare_cluster);
   for (int i = 0; i < tr.size(); ++i)
   {
      if (tr[i].second > 10)
      {
         cluster_index.push_back(tr[i].first);
      }
   }
   return true;
}

#endif