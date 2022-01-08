/*
 * Original work Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * Modified work Copyright 2020 Dan Riley
 * Modified work Copyright 2021 Daniel Torres
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MARBLE_MAPPING_MARBLEMAPPING_H
#define MARBLE_MAPPING_MARBLEMAPPING_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <marble_mapping/MarbleMappingConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations" // pcl::SAC_SAMPLE_SIZE is protected since PCL 1.8.0
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <rough_octomap/RoughOcTree.h>
#include <rough_octomap/conversions.h>
#include <omp.h>
#include <unordered_set>

#include "marble_mapping/OctomapArray.h"
#include "marble_mapping/OctomapNeighbors.h"

namespace marble_mapping {
class MarbleMapping {

public:
  typedef pcl::PointXYZI PCLPoint;
  typedef pcl::PointXYZRGB PCLPointRGB;
  typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
  typedef pcl::PointCloud<PCLPointRGB> PCLPointCloudRGB;
  typedef pcl::PointXYZ icpPoint;
  typedef pcl::PointCloud<icpPoint> icpPointCloud;
  typedef octomap::OcTree OcTreeT;
  typedef octomap::RoughOcTree RoughOcTreeT;
  typedef octomap_msgs::GetOctomap OctomapSrv;
  ros::CallbackQueue pub_queue;

  MarbleMapping(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle());
  virtual ~MarbleMapping();
  virtual bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  virtual bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  virtual void neighborMapsCallback(const marble_mapping::OctomapNeighborsConstPtr& msg);
  virtual void octomapCallback(const octomap_msgs::OctomapConstPtr& msg);
  virtual void octomapDiffsCallback(const octomap_msgs::OctomapConstPtr& msg, const std::string owner);
  virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  virtual void insertStairCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  virtual bool openFile(const std::string& filename);

protected:
  void reconfigureCallback(marble_mapping::MarbleMappingConfig& config, uint32_t level);
  void publishBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishMergedBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishMergedFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishCameraOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishNeighborMaps(const ros::Time& rostime = ros::Time::now());
  void publishOctoMaps(const ros::TimerEvent& event);
  void publishOptionalMaps(const ros::TimerEvent& event);

  /**
  * @brief update occupancy map with a scan labeled as ground and nonground.
  * The scans should be in the global map frame.
  *
  * @param sensorOrigin origin of the measurements for raycasting
  * @param ground scan endpoints on the ground plane (only clear space)
  * @param nonground all other endpoints (clear up to occupied endpoint)
  */
  virtual double insertScan(const tf::StampedTransform& sensorToWorldTf, const PCLPointCloud& ground, const PCLPointCloud& nonground);

  // Check the changes since last run to publish for sharing to other agents
  template <class OcTreeMT>
  int updateDiffTree(OcTreeMT* tree, PCLPointCloudRGB& pclDiffCloud);
  void updateDiff(const ros::TimerEvent& event);

  // Merge all neighbor maps
  void parseIDXandAgent(std::string& cidx_str);
  void parseColorPairs(std::string& color_pairs_str);
  char nidToIDX(std::string& nid);
  void mergeNeighbors();

  // ICP supporting functions
  void tree2PointCloud(const RoughOcTreeT* tree, icpPointCloud& cloud);
  void updateNeighborMaps();
  void mergeNeighbors(const ros::TimerEvent& event);
  Eigen::Matrix4f alignMap(const RoughOcTreeT* tree);
  void mergeMap(const RoughOcTreeT* tree);
  void transformMap(RoughOcTreeT* tree, const Eigen::Matrix4f& tf);
  Eigen::Matrix4f getGICPTransform(icpPointCloud& source, icpPointCloud& target);
  double getSign(double x) { if (x < 0) return -1; else return 1; }

  struct PointHash {
    size_t operator()(const octomap::point3d& point) const {
      size_t xHash = std::hash<int>()(point.x());
      size_t yHash = std::hash<int>()(point.y());
      size_t zHash = std::hash<int>()(point.z());
      return xHash ^ yHash ^ zHash;
    }
  };

  /// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
  void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const;

  /**
  * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
  * @param key
  * @return
  */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  static std_msgs::ColorRGBA heightMapColor(double h);

  void buildView(const tf::Matrix3x3& rotation, const octomap::point3d& position);
  bool pointInsideView(octomap::point3d& point);
  bool isCeiling(const octomap::OcTreeKey& curKey);

  ros::NodeHandle m_nh;
  ros::NodeHandle m_nh_private;
  ros::Publisher  m_markerPub, m_binaryMapPub, m_fullMapPub, m_mergedBinaryMapPub, m_mergedFullMapPub, m_diffMapPub, m_diffsMapPub, m_cameraMapPub, m_cameraViewPub, m_pointCloudPub, m_pointCloudDiffPub, m_fmarkerPub;
  ros::Subscriber m_neighborsSub;
  ros::Subscriber m_octomapSub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
  ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_resetService;
  ros::Timer diff_timer;
  ros::Timer pub_timer;
  ros::Timer pub_opt_timer;
  ros::Timer map_merge_timer;
  tf::TransformListener m_tfListener;
  mutable boost::mutex m_mtx;
  boost::recursive_mutex m_config_mutex;
  dynamic_reconfigure::Server<MarbleMappingConfig> m_reconfigureServer;

  RoughOcTreeT* m_octree;
  RoughOcTreeT* m_diff_tree;
  OcTreeT* m_camera_tree;
  RoughOcTreeT* m_merged_tree;
  std::vector<octomap::KeyRay> keyrays;
  RoughOcTreeT* temp_tree; // for neighbor diff concatenation

  marble_mapping::OctomapArray mapdiffs;
  marble_mapping::OctomapNeighbors neighbors;
  std::map<std::string, RoughOcTreeT*> neighbor_maps;
  std::map<std::string, bool> neighbor_updated;
  std::map<std::string, ros::Publisher> neighbor_pubs;

  int m_input;
  double m_maxRange;
  double m_minRange;
  std::string m_worldFrameId; // the map frame
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;
  double m_colorFactor;

  double pub_duration;
  double pub_opt_duration;
  bool m_octree_updated;
  bool m_mtree_updated;
  bool m_mtree_markers_updated;
  bool m_mergeMaps;
  bool m_latchedTopics;
  bool m_publishDiffs;
  bool m_publishMarkerArray;
  bool m_publishFreeSpace;
  bool m_publishPointCloud;
  bool m_publishPointCloudDiff;
  bool m_publishMergedBinaryMap;
  bool m_publishMergedFullMap;
  bool m_publishCameraMap;
  bool m_publishCameraView;
  bool m_publishNeighborMaps;
  bool m_adjustAgent;
  int m_lastSub;
  int m_lastMarkerSub;
  int m_displayColor;

  bool m_removeCeiling;
  int m_removeCeilingDepth;

  double m_res;
  double m_mres;
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;

  // traversability vars
  ros::Publisher m_travMarkerPub;
  bool m_enableTraversability;
  bool m_enableTraversabilitySharing;
  bool m_publishTravMarkerArray;
  int m_travMarkerDensity;

  // stair vars
  ros::Publisher m_stairMarkerPub, m_stairPointsPub, m_stairEdgePub, m_nearStairIndicatorPub, m_slowdownForStairsIndicatorPub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* m_stairPointCloudSub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* m_stairTfPointCloudSub;
  ros::Timer stairs_timer, is_near_stair_timer;
  std::unordered_set<octomap::point3d, PointHash> m_stairPoints;
  bool m_enableStairs;
  float m_stairProcessRate, m_isNearStairPubRate;
  bool m_publishStairMarkerArray;
  int m_stairMarkerDensity;
  bool m_publishStairPointsArray;
  bool m_publishStairEdgeArray;
  bool m_publishNearStairIndicator;
  bool m_isNearStairs, m_slowdownForStairs;
  float m_isNearStairInnerRad, m_isNearStairOuterRad, m_slowdownForStairsRad, m_isNearStairZThresh;
  pcl::PointCloud<pcl::PointXYZ> m_stairCloud;
  void processStairsLoop(const ros::TimerEvent& event);
  void pubNearStairIndicatorLoop(const ros::TimerEvent& event);
  void procAndPubNearStairIndicator();
  void setStairCloud(pcl::PointCloud<pcl::PointXYZ>& stairCloud);
  virtual void insertStairScan(const tf::StampedTransform& sensorToWorldTf, const PCLPointCloud& pc);

  // Diff parameters
  int diff_threshold;
  double diff_duration;
  unsigned num_diffs;
  bool m_diffMerged;
  char next_idx;
  std::map<std::string, std::vector<int>> seqs;
  std::map<std::string, char> idx;
  std::map<std::string, char> color_pairs;

  double m_pointcloudMinX;
  double m_pointcloudMaxX;
  double m_pointcloudMinY;
  double m_pointcloudMaxY;
  double m_pointcloudMinZ;
  double m_pointcloudMaxZ;
  double m_occupancyMinZ;
  double m_occupancyMaxZ;

  // Performance Tuning
  bool m_compressMaps;
  bool m_threadPublishing;
  bool m_passthroughFilter;
  bool m_enableRadiusOutlierRemoval;
  double m_downsampleSize;
  double m_pclTimeLimit;
  int m_numThreads;

  // Track dropped PCL
  ros::Duration pclTimeLimit;
  ros::Duration longTimeDiff;
  unsigned int pclCount;
  unsigned int pclCountProcessed;
  unsigned int pclDropped;
  double pclTime;

  bool m_filterSpeckles;

  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;

  bool m_initConfig;

  // Secondary camera variables for restricted view map
  bool m_buildCameraMap;
  double camera_range, camera_h, camera_w;
  Eigen::Vector4f pl_f, pl_t, pl_b, pl_r, pl_l;
  visualization_msgs::Marker m_cameraView;

  // Diff merging variables
  std::string m_agents;
  std::string m_diff_pre;
  std::string m_diff_post;
  std::map<std::string, ros::Subscriber> m_diffSubs;

  // Map alignment parameters
  std::string m_inputTopic;
  std::string m_neighborsInputTopic;
  int m_mapMergeTimer;
  int m_leafScale;
  bool m_neighborMapsMerged = false; // default init to false
  std::map<std::string, bool> neighbor_aligned;
  std::map<std::string, Eigen::Matrix4f> neighbor_tf;
  // ICP parameters
  int meanK;
  double stdDevMulThresh;
  int leafScale;
  int icpIterations;
  int icpOptimizerIterations;
  int icpMaxCorrespDistScale;
  double icpTFEpsScale;
  int ransacNumIterations;
  double ransacOutlierRejecThresh;
  double euclideanFitnessEps;
  double fitnessScoreThreshold;
};
}

#endif