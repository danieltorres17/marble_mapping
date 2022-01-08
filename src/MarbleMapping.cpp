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

#include <MarbleMapping.h>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace marble_mapping{

MarbleMapping::MarbleMapping(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_pointCloudSub(NULL), m_stairPointCloudSub(NULL),
  m_tfPointCloudSub(NULL), m_stairTfPointCloudSub(NULL),
  m_reconfigureServer(m_config_mutex, private_nh_),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_colorFactor(0.8),
  m_latchedTopics(true),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_pointcloudMinX(-std::numeric_limits<double>::max()),
  m_pointcloudMaxX(std::numeric_limits<double>::max()),
  m_pointcloudMinY(-std::numeric_limits<double>::max()),
  m_pointcloudMaxY(std::numeric_limits<double>::max()),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_filterSpeckles(false), m_filterGroundPlane(false),
  m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
  m_isNearStairs(false),
  m_initConfig(true)
{
  double probHit, probMiss, thresMin, thresMax;
  double stairsProbHit, stairsProbMiss, stairsProbMin, stairsProbMax, stairsProbThres;

  m_nh_private.param("input", m_input, 1);
  m_nh_private.param("frame_id", m_worldFrameId, m_worldFrameId);
  m_nh_private.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  m_nh_private.param("color_factor", m_colorFactor, m_colorFactor);

  m_nh_private.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  m_nh_private.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  m_nh_private.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  m_nh_private.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  m_nh_private.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  m_nh_private.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  m_nh_private.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  m_nh_private.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);

  m_nh_private.param("compress_maps", m_compressMaps, true);
  m_nh_private.param("thread_publishing", m_threadPublishing, true);
  m_nh_private.param("passthrough_filter", m_passthroughFilter, true);
  m_nh_private.param("enable_radius_outlier_removal", m_enableRadiusOutlierRemoval, false);
  m_nh_private.param("num_threads", m_numThreads, 1);
  m_nh_private.param("downsample_size", m_downsampleSize, 0.0);
  m_nh_private.param("pcl_time_limit", m_pclTimeLimit, 100.0);

  m_nh_private.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  m_nh_private.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  // distance of points from plane for RANSAC
  m_nh_private.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  // angular derivation of found plane:
  m_nh_private.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  m_nh_private.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

  m_nh_private.param("sensor_model/max_range", m_maxRange, m_maxRange);
  m_nh_private.param("sensor_model/min_range", m_minRange, -1.0);

  m_nh_private.param("resolution", m_res, m_res);
  m_nh_private.param("merged_resolution", m_mres, 0.2);
  m_nh_private.param("sensor_model/hit", probHit, 0.7);
  m_nh_private.param("sensor_model/miss", probMiss, 0.4);
  m_nh_private.param("sensor_model/min", thresMin, 0.12);
  m_nh_private.param("sensor_model/max", thresMax, 0.97);
  m_nh_private.param("publish_duration", pub_duration, 0.1);
  m_nh_private.param("publish_merged_binary", m_publishMergedBinaryMap, false);
  m_nh_private.param("publish_merged_full", m_publishMergedFullMap, false);

  // Enable multiagent functions such as diff creation and merging
  m_nh_private.param("merge_maps", m_mergeMaps, true);
  // Enable diff publishing (diffs are still created)
  m_nh_private.param("publish_diffs", m_publishDiffs, false);
  // Threshold for when to add map diffs
  m_nh_private.param("diff_threshold", diff_threshold, 1000);
  // How many seconds to check/publish diffs
  m_nh_private.param("diff_duration", diff_duration, 10.0);
  m_nh_private.param("diff_merged", m_diffMerged, false);

  // traversability
  m_nh_private.param("enable_traversability", m_enableTraversability, false);
  m_nh_private.param("enable_traversability_sharing", m_enableTraversabilitySharing, false);
  m_nh_private.param("trav_marker_density", m_travMarkerDensity, 0);

  if (!m_enableTraversability && m_enableTraversabilitySharing) {
    ROS_WARN("Traversability sharing enabled but traversability overall disabled. Disabling sharing.");
    m_enableTraversabilitySharing = false;
  }

  m_publishTravMarkerArray = (m_travMarkerDensity>0 && m_enableTraversability);
  if (m_publishTravMarkerArray)
    m_travMarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("trav_text_markers", 1, m_latchedTopics);

  // stairs
  m_nh_private.param("enable_stairs", m_enableStairs, false);
  m_nh_private.param("process_stairs_rate", m_stairProcessRate, 1.0f);
  m_nh_private.param("stair_marker_density", m_stairMarkerDensity, 0);
  m_nh_private.param("publish_stair_points_array", m_publishStairPointsArray, false);
  m_nh_private.param("publish_stair_edge_array", m_publishStairEdgeArray, false);
  m_nh_private.param("publish_near_stair_indicator", m_publishNearStairIndicator, false);
  m_nh_private.param("is_near_stair_pub_rate", m_isNearStairPubRate, 2.0f);
  m_nh_private.param("is_near_stair_inner_rad", m_isNearStairInnerRad, 1.0f);
  m_nh_private.param("is_near_stair_outer_rad", m_isNearStairOuterRad, 1.5f);
  m_nh_private.param("slow_down_for_rad", m_slowdownForStairsRad, 3.0f);
  m_nh_private.param("is_near_stair_z_thresh", m_isNearStairZThresh, 3.0f);
  m_nh_private.param("stairs_prob_hit", stairsProbHit, 0.99);
  m_nh_private.param("stairs_prob_miss", stairsProbMiss, 0.49);
  m_nh_private.param("stairs_prob_thres", stairsProbThres, 0.5);
  m_nh_private.param("stairs_prob_max", stairsProbMax, 0.97);
  m_nh_private.param("stairs_prob_min", stairsProbMin, 0.12);

  if (m_enableStairs) {
    stairs_timer = m_nh.createTimer(ros::Duration(1.f/m_stairProcessRate), &MarbleMapping::processStairsLoop, this);
    is_near_stair_timer = m_nh.createTimer(ros::Duration(1.f/m_isNearStairPubRate), &MarbleMapping::pubNearStairIndicatorLoop, this);
    m_publishStairMarkerArray = m_stairMarkerDensity > 0;
  } else {
    m_publishStairMarkerArray = false;
    m_publishStairPointsArray = false;
    m_publishStairEdgeArray = false;
    m_publishNearStairIndicator = false;
  }

  if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)){
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<m_pointcloudMinZ <<", "<< m_pointcloudMaxZ << "], excluding the ground level z=0. "
              << "This will not work.");
  }

  // initialize octomap object & params
  m_octree = new RoughOcTreeT(m_res);
  m_octree->setRoughEnabled(m_enableTraversability);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  m_octree->setStairsEnabled(m_enableStairs);
  m_octree->setStairsProbHit(stairsProbHit);
  m_octree->setStairsProbMiss(stairsProbMiss);
  m_octree->setStairsClampingThresMin(stairsProbMin);
  m_octree->setStairsClampingThresMax(stairsProbMax);

  #pragma omp parallel
  #pragma omp master
  {
    // Set the number of threads, limited to maximum threads allowed by OMP_NUM_THREADS
    if ((m_numThreads == 0) || (m_numThreads > omp_get_max_threads()))
      m_numThreads = omp_get_num_threads();
    ROS_INFO("Using %d threads of %d available", m_numThreads, omp_get_max_threads());
    keyrays.resize(m_numThreads);
  }
  omp_set_num_threads(m_numThreads);

  // Merged OcTree
  m_merged_tree = new RoughOcTreeT(m_mres);
  m_merged_tree->setRoughEnabled(m_enableTraversability);
  m_merged_tree->setProbHit(probHit);
  m_merged_tree->setProbMiss(probMiss);
  m_merged_tree->setClampingThresMin(thresMin);
  m_merged_tree->setClampingThresMax(thresMax);
  m_merged_tree->setStairsEnabled(m_enableStairs);
  m_merged_tree->setStairsProbHit(stairsProbHit);
  m_merged_tree->setStairsProbMiss(stairsProbMiss);
  m_merged_tree->setStairsClampingThresMin(stairsProbMin);
  m_merged_tree->setStairsClampingThresMax(stairsProbMax);
  m_treeDepth = m_merged_tree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

  // Use change detection to build diff tree
  if (m_diffMerged)
    m_merged_tree->enableChangeDetection(true);
  else
    m_octree->enableChangeDetection(true);

  // Differences OcTree, for sharing between agents
  m_diff_tree = new RoughOcTreeT(m_mres);
  m_diff_tree->setRoughEnabled(m_enableTraversabilitySharing);
  m_diff_tree->setProbHit(probHit);
  m_diff_tree->setProbMiss(probMiss);
  m_diff_tree->setClampingThresMin(thresMin);
  m_diff_tree->setClampingThresMax(thresMax);

  // Camera View OcTree, to show what has been seen by secondary sensor(s)
  m_camera_tree = new OcTreeT(m_mres);
  m_camera_tree->setProbHit(probHit);
  m_camera_tree->setProbMiss(probMiss);
  m_camera_tree->setClampingThresMin(thresMin);
  m_camera_tree->setClampingThresMax(thresMax);

  num_diffs = 0;
  next_idx = 2;

  double r, g, b, a;
  m_nh_private.param("color/r", r, 0.0);
  m_nh_private.param("color/g", g, 0.0);
  m_nh_private.param("color/b", b, 1.0);
  m_nh_private.param("color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  m_nh_private.param("color_free/r", r, 0.0);
  m_nh_private.param("color_free/g", g, 1.0);
  m_nh_private.param("color_free/b", b, 0.0);
  m_nh_private.param("color_free/a", a, 1.0);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  // Get secondary camera view's height, width and range and whether to publish
  double vfov_angle, hfov_angle;
  m_nh_private.param("build_camera_map", m_buildCameraMap, false);
  m_nh_private.param("publish_camera_map", m_publishCameraMap, false);
  m_nh_private.param("publish_camera_view", m_publishCameraView, false);
  m_nh_private.param("camera_range", camera_range, 5.0);
  m_nh_private.param("camera_horizontal_fov", hfov_angle, 60.0);
  m_nh_private.param("camera_vertical_fov", vfov_angle, 20.0);
  double hfov_rad = hfov_angle * M_PI / 180.0;
  double vfov_rad = vfov_angle * M_PI / 180.0;
  camera_h = 2.0 * tan(vfov_rad / 2.0) * camera_range;
  camera_w = 2.0 * tan(hfov_rad / 2.0) * camera_range;

  m_nh_private.param("publish_optional_duration", pub_opt_duration, 0.2);
  m_nh_private.param("publish_marker_array", m_publishMarkerArray, false);
  m_nh_private.param("publish_free_space", m_publishFreeSpace, false);
  m_nh_private.param("publish_point_cloud", m_publishPointCloud, false);
  m_nh_private.param("publish_point_cloud_diff", m_publishPointCloudDiff, false);
  m_nh_private.param("publish_neighbor_maps", m_publishNeighborMaps, false);
  m_nh_private.param("display_color", m_displayColor, 1);

  m_nh_private.param("latch", m_latchedTopics, m_latchedTopics);

  m_nh_private.param("remove_ceiling", m_removeCeiling, false);
  m_nh_private.param("remove_ceiling_depth", m_removeCeilingDepth, 4);

  // Octomap in parameters
  m_nh_private.param("input_topic", m_inputTopic, m_inputTopic);
  m_nh_private.param("neighbors_input_topic", m_neighborsInputTopic, m_neighborsInputTopic);

  // Diff merging parameters
  m_nh_private.param("agents", m_agents, std::string("X1,X2"));
  m_nh_private.param("diff_pre", m_diff_pre, std::string("/robot_data/"));
  m_nh_private.param("diff_post", m_diff_post, std::string("/map_diff"));
  m_nh_private.param("map_merge_timer", m_mapMergeTimer, 300);

  bool pclInput = false;
  bool octomapInput = false;
  bool diffInput = false;

  if (m_input == 1) pclInput = true;
  else if (m_input == 2) octomapInput = true;
  else if (m_input == 3) diffInput = true;
  // To get full marker colors need to reduce the agent by one if we don't have a self map
  if ((m_input == 0) || (m_input == 3)) m_adjustAgent = true;
  else m_adjustAgent = false;

  // Normal octomap format map publishers
  if (pclInput) {
    m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, m_latchedTopics);
    m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, m_latchedTopics);
  }

  // Merged map publishers
  if (m_publishMergedBinaryMap)
    m_mergedBinaryMapPub = m_nh.advertise<Octomap>("merged_map", 1, m_latchedTopics);
  if (m_publishMergedFullMap)
    m_mergedFullMapPub = m_nh.advertise<Octomap>("merged_map_full", 1, m_latchedTopics);

  // Diff map publishers
  if (m_publishDiffs) {
    m_diffMapPub = m_nh.advertise<Octomap>("map_diff", 1, m_latchedTopics);
    m_diffsMapPub = m_nh.advertise<marble_mapping::OctomapArray>("map_diffs", 1, m_latchedTopics);
  }

  if (m_publishCameraMap)
    m_cameraMapPub = m_nh.advertise<Octomap>("camera_map", 1, m_latchedTopics);

  // Optional marker array and point cloud publishers
  if (m_publishMarkerArray)
    m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("merged_map_markers", 1, m_latchedTopics);
  if (m_publishFreeSpace)
    m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("merged_map_free", 1, m_latchedTopics);
  if (m_publishPointCloud)
    m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("merged_map_pc", 1, m_latchedTopics);
  if (m_publishPointCloudDiff)
    m_pointCloudDiffPub = m_nh.advertise<sensor_msgs::PointCloud2>("pc_diff", 1, m_latchedTopics);

  if (m_publishCameraView) {
    m_cameraViewPub = m_nh.advertise<visualization_msgs::Marker>("camera_view", 1, m_latchedTopics);
    m_cameraView.header.frame_id = "world";
    m_cameraView.id = 0;
    m_cameraView.scale.x = 1;
    m_cameraView.scale.y = 1;
    m_cameraView.scale.z = 1;
    m_cameraView.pose.position.x = 0.0;
    m_cameraView.pose.position.y = 0.0;
    m_cameraView.pose.position.z = 0.0;
    m_cameraView.pose.orientation.x = 0.0;
    m_cameraView.pose.orientation.y = 0.0;
    m_cameraView.pose.orientation.z = 0.0;
    m_cameraView.pose.orientation.w = 1.0;
    m_cameraView.color.a = 0.85;
    m_cameraView.color.r = 0.0;
    m_cameraView.color.g = 191.0/255.0;
    m_cameraView.color.b = 1.0;
    m_cameraView.type = 11;
    m_cameraView.action = visualization_msgs::Marker::ADD;
  }

  if (m_publishStairMarkerArray)
    m_stairMarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("stair_text_markers", 1, m_latchedTopics);
  if (m_publishStairPointsArray)
    m_stairPointsPub = m_nh.advertise<visualization_msgs::MarkerArray>("stair_points_array", 1, m_latchedTopics);
  if (m_publishStairEdgeArray)
    m_stairEdgePub = m_nh.advertise<visualization_msgs::MarkerArray>("stair_edge_array", 1, m_latchedTopics);
  if (m_publishNearStairIndicator) {
    m_nearStairIndicatorPub = m_nh.advertise<std_msgs::Bool>("is_near_stair", 1, m_latchedTopics);
    m_slowdownForStairsIndicatorPub = m_nh.advertise<std_msgs::Bool>("slowdown_for_stairs", 1, m_latchedTopics);
  }

  // Subscribers
  if (pclInput) {
    // Suppress insane PCL warnings
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
    // m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "lio_sam/mapping/cloud_registered", 5);
    m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSub->registerCallback(boost::bind(&MarbleMapping::insertCloudCallback, this, _1));
    m_stairPointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "stair_cloud_in", 5);
    m_stairTfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_stairPointCloudSub, m_tfListener, m_worldFrameId, 5);
    m_stairTfPointCloudSub->registerCallback(boost::bind(&MarbleMapping::insertStairCloudCallback, this, _1));
    // Services
    m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &MarbleMapping::octomapBinarySrv, this);
    m_octomapFullService = m_nh.advertiseService("octomap_full", &MarbleMapping::octomapFullSrv, this);
    m_resetService = m_nh_private.advertiseService("reset", &MarbleMapping::resetSrv, this);
  } else if (octomapInput) {
    m_octomapSub = m_nh.subscribe(m_inputTopic, 100, &MarbleMapping::octomapCallback, this);
  } else if (diffInput) {
    // Get the agents from the launch parameters
    std::vector<std::string> agents_vec;
    std::replace(m_agents.begin(), m_agents.end(), ',', ' ');
    std::stringstream ss(m_agents);
    std::string temp;
    while (ss >> temp)
      agents_vec.push_back(temp);

    // Create a subscriber for each agent
    for (auto agent : agents_vec) {
      std::string topic = m_diff_pre + agent + m_diff_post;
      m_diffSubs[agent] = m_nh.subscribe<octomap_msgs::Octomap>(topic, 100, boost::bind(&MarbleMapping::octomapDiffsCallback, this, _1, agent));
    }

    // Populate the neighbors message so mergeNeighbors will work
    neighbors.num_neighbors = 1;
    neighbors.neighbors.push_back(marble_mapping::OctomapArray());
    neighbors.neighbors[0].num_octomaps = 1;
    neighbors.neighbors[0].octomaps.push_back(octomap_msgs::Octomap());
  }

  m_neighborsSub = m_nh.subscribe(m_neighborsInputTopic, 100, &MarbleMapping::neighborMapsCallback, this);
  // Diff map merge timer 
  // map_merge_timer = m_nh.createTimer(ros::Duration(m_mapMergeTimer), &MarbleMapping::mergeNeighbors, this);

  // Timer for updating the local map diff that will be broadcast
  diff_timer = m_nh.createTimer(ros::Duration(diff_duration), &MarbleMapping::updateDiff, this);

  // Timer for publishing the OctoMaps
  ros::TimerOptions ops_pub;
  ops_pub.period = ros::Duration(pub_duration);
  ops_pub.callback = boost::bind(&MarbleMapping::publishOctoMaps, this, _1);
  if (m_threadPublishing && !octomapInput) ops_pub.callback_queue = &pub_queue;
  pub_timer = m_nh.createTimer(ops_pub);

  // Timer to publish the optional maps, on a separate thread
  if (m_publishMarkerArray || m_publishFreeSpace || m_publishPointCloud || m_publishTravMarkerArray) {
    ros::TimerOptions ops;
    ops.period = ros::Duration(pub_opt_duration);
    ops.callback = boost::bind(&MarbleMapping::publishOptionalMaps, this, _1);
    // Assign to the publishing thread, unless input is an octomap
    // If input is octomap the map is deleted so need to run single threaded
    if (!octomapInput) ops.callback_queue = &pub_queue;
    pub_opt_timer = m_nh.createTimer(ops);
  }

  pclTimeLimit = ros::Duration(0.0);
  longTimeDiff = ros::Duration(0.0);
  pclCount = 0;
  pclCountProcessed = 0;
  pclDropped = 0;
  pclTime = 0;

  std::string color_pairs_str;
  m_nh_private.param("color_pairs", color_pairs_str, std::string(""));
  parseColorPairs(color_pairs_str);

  dynamic_reconfigure::Server<MarbleMappingConfig>::CallbackType f;
  f = boost::bind(&MarbleMapping::reconfigureCallback, this, _1, _2);
  m_reconfigureServer.setCallback(f);
}

MarbleMapping::~MarbleMapping(){
  if (m_tfPointCloudSub){
    delete m_tfPointCloudSub;
    m_tfPointCloudSub = NULL;
  }

  if (m_pointCloudSub){
    delete m_pointCloudSub;
    m_pointCloudSub = NULL;
  }

  if (m_octree){
    delete m_octree;
    m_octree = NULL;
  }

  if (m_merged_tree){
    delete m_merged_tree;
    m_merged_tree = NULL;
  }

  if (m_diff_tree){
    delete m_diff_tree;
    m_diff_tree = NULL;
  }
}

bool MarbleMapping::openFile(const std::string& filename){
  if (filename.length() <= 3)
    return false;

  std::string suffix = filename.substr(filename.length()-3, 3);
  if (suffix== ".bt"){
    if (!m_octree->readBinary(filename)){
      return false;
    }
  } else if (suffix == ".ot"){
    AbstractOcTree* tree = AbstractOcTree::read(filename);
    if (!tree){
      return false;
    }
    if (m_octree){
      delete m_octree;
      m_octree = NULL;
    }
    m_octree = dynamic_cast<RoughOcTreeT*>(tree);
    if (!m_octree){
      ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }

  } else{
    return false;
  }

  ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octree->size());

  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_res = m_octree->getResolution();

  return true;
}

void MarbleMapping::neighborMapsCallback(const marble_mapping::OctomapNeighborsConstPtr& msg) {
  neighbors = *msg;

  // Neighbor Maps array gets corrupted occasionally during publishing for an unknown reason
  // If that happens, just ignore any reset or clear commands from this message
  bool invalidMessage = (neighbors.num_neighbors != neighbors.neighbors.size());
  if (!invalidMessage) {
    for (int i = 0; i < msg->neighbors.size(); i++) {
      if (neighbors.neighbors[i].num_octomaps != neighbors.neighbors[i].octomaps.size())
        invalidMessage = true;
    }
  }

  if (invalidMessage) {
    ROS_WARN("Received invalid neighbor maps!");
    neighbors.hardReset = false;
    neighbors.clear = false;
  }

  // If hard reset passed, start the self and merged map over!
  if (neighbors.hardReset) {
    ROS_INFO("Hard resetting map due to request.");
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->clear();
    m_merged_tree->clear();
    if (m_diffMerged)
      m_merged_tree->resetChangeDetection();
    else
      m_octree->resetChangeDetection();
    mapdiffs = marble_mapping::OctomapArray();
    num_diffs = 0;
  }

  // Better to put this somewhere else, but the callback should be infrequent enough this is ok
  try {
    if (m_mergeMaps)
      updateNeighborMaps();
  } catch (const std::exception& e) {
    ROS_ERROR("merging error! %s", e.what());
  }

  if (m_compressMaps) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->prune();
    m_merged_tree->prune();
  }
}

void MarbleMapping::octomapCallback(const octomap_msgs::OctomapConstPtr& msg) {
  boost::mutex::scoped_lock lock(m_mtx);
  delete m_merged_tree;
  m_merged_tree = (RoughOcTreeT*)octomap_msgs::binaryMsgToMap(*msg);
  m_mtree_updated = true;
  m_mtree_markers_updated = true;
}

void MarbleMapping::octomapDiffsCallback(const octomap_msgs::OctomapConstPtr& msg, const std::string owner) {
  // Update the neighbors message with current info
  neighbors.neighbors[0].owner = owner;
  neighbors.neighbors[0].octomaps[0] = *msg;

  // Merge the diff
  try {
    updateNeighborMaps();
  } catch (const std::exception& e) {
    ROS_ERROR("merging error! %s", e.what());
  }

  if (m_compressMaps) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->prune();
    m_merged_tree->prune();
  }
}

void MarbleMapping::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  // Get the initial time offset for the sensor
  // This might not work well if multiple sensors publish to the same topic, instead of using
  // a topic_tools to merge them, where the timestamps get rewritten!
  if (pclTimeLimit == ros::Duration(0.0)) {
    pclTimeLimit = ros::Time().now() - cloud->header.stamp + ros::Duration(m_pclTimeLimit);
  }

  pclCount++;
  // Drop any "old" pointclouds
  // For multiple sensors, might want a way to ensure we get a good mix of each!
  if (cloud->header.stamp + pclTimeLimit < ros::Time().now()) {
    pclDropped++;
    ros::Duration timediff = ros::Time::now() - cloud->header.stamp - pclTimeLimit + ros::Duration(m_pclTimeLimit);
    if (timediff > longTimeDiff) longTimeDiff = timediff;

    return;
  }

  ros::WallTime startTime = ros::WallTime::now();

  //
  // ground filtering in base frame
  //
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  if (m_downsampleSize > 0.0) {
    pcl::VoxelGrid<PCLPoint> voxel_filter;
    voxel_filter.setInputCloud (pc.makeShared());
    voxel_filter.setFilterFieldName("x");
    voxel_filter.setLeafSize (m_downsampleSize, m_downsampleSize, m_downsampleSize);
    voxel_filter.filter (pc);
  }

  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


  // set up filter for height range, also removes NANs:
  pcl::PassThrough<PCLPoint> pass_x;
  pcl::PassThrough<PCLPoint> pass_y;
  pcl::PassThrough<PCLPoint> pass_z;
  if (m_passthroughFilter) {
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
  }

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  if (m_filterGroundPlane){
    tf::StampedTransform sensorToBaseTf, baseToWorldTf;
    try{
      m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
      m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);
    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                        "You need to set the base_frame_id or disable filter_ground.");
    }

    Eigen::Matrix4f sensorToBase, baseToWorld;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToBase);

    if (m_passthroughFilter) {
      pass_x.setInputCloud(pc.makeShared());
      pass_x.filter(pc);
      pass_y.setInputCloud(pc.makeShared());
      pass_y.filter(pc);
      pass_z.setInputCloud(pc.makeShared());
      pass_z.filter(pc);
    }

    filterGroundPlane(pc, pc_ground, pc_nonground);

    // transform clouds to world frame for insertion
    pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
    pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
  } else {
    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    // just filter height range:
    if (m_passthroughFilter) {
      pass_x.setInputCloud(pc.makeShared());
      pass_x.filter(pc);
      pass_y.setInputCloud(pc.makeShared());
      pass_y.filter(pc);
      pass_z.setInputCloud(pc.makeShared());
      pass_z.filter(pc);
    }

    // Remove any spurious points that still exist if we're filtering minrange
    if (m_enableRadiusOutlierRemoval) {
      pcl::RadiusOutlierRemoval<PCLPoint> outrem;
      // build the filter
      outrem.setInputCloud(pc.makeShared());
      outrem.setRadiusSearch(0.2);
      outrem.setMinNeighborsInRadius(5);
      // apply filter
      outrem.filter(pc);
    }

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }


  double delayTime = insertScan(sensorToWorldTf, pc_ground, pc_nonground);

  if (m_compressMaps) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->prune();
    m_merged_tree->prune();
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in MarbleMapping done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

  pclCountProcessed++;
  pclTime += total_elapsed - delayTime;
  m_octree_updated = true;
  m_mtree_updated = true;
  m_mtree_markers_updated = true;
}

double MarbleMapping::insertScan(const tf::StampedTransform& sensorToWorldTf, const PCLPointCloud& ground, const PCLPointCloud& nonground){
  // Get the rotation matrix and the position, and build camera view if enabled
  tf::Matrix3x3 rotation = sensorToWorldTf.getBasis();
  point3d sensorOrigin = pointTfToOctomap(sensorToWorldTf.getOrigin());
  if (m_buildCameraMap)
    buildView(rotation, sensorOrigin);

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;

  // Profile our processing time
  ros::WallTime startTime = ros::WallTime::now();
  // Need to lock here due to rough integration during raycasting
  boost::mutex::scoped_lock lock(m_mtx);
  // Account for the delay in waiting for lock release in our overall PCL profiling
  double delayTime = (ros::WallTime::now() - startTime).toSec();

  // insert ground points only as free:
  #pragma omp parallel for schedule(guided)
  for (PCLPointCloud::const_iterator it = ground.begin(); it < ground.end(); ++it) {
    point3d point(it->x, it->y, it->z);
    unsigned threadIdx = omp_get_thread_num();
    KeyRay* keyRay = &(keyrays.at(threadIdx));

    // maxrange check
    if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
      point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
    }

    // only clear space (ground points)
    if (m_octree->computeRayKeys(sensorOrigin, point, *keyRay)) {
      #pragma omp critical(free_insert)
      {
        free_cells.insert(keyRay->begin(), keyRay->end());
      }
    }
  }

  // all other points: free on ray, occupied on endpoint:
  #pragma omp parallel for schedule(guided)
  for (PCLPointCloud::const_iterator it = nonground.begin(); it < nonground.end(); ++it) {
    point3d point(it->x, it->y, it->z);
    unsigned threadIdx = omp_get_thread_num();
    KeyRay* keyRay = &(keyrays.at(threadIdx));

    // minrange / maxrange check
    if (((m_minRange <= 0.0) || ((point - sensorOrigin).norm() >= m_minRange)) &&
        ((m_maxRange <= 0.0) || ((point - sensorOrigin).norm() <= m_maxRange))) {

      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, *keyRay)) {
        #pragma omp critical(free_insert)
        {
          free_cells.insert(keyRay->begin(), keyRay->end());
        }
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)) {
        #pragma omp critical(occupied_insert)
        {
          occupied_cells.insert(key);
          if (m_enableTraversability) {
            m_octree->integrateNodeRough(key, it->intensity);
          }
        }
      }
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, *keyRay)) {
        #pragma omp critical(free_insert)
        {
          free_cells.insert(keyRay->begin(), keyRay->end());
        }

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)) {
          #pragma omp critical(free_insert)
          {
            // This clears out previously occupied cells that were at the edge of the range
            free_cells.insert(endKey);
          }
        } else {
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }
      }
    }
  }

  // Find the total time for ray casting
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("The %zu points took %f seconds)", nonground.size(), total_elapsed);

  // mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      // Voxels are based on the main octree, so need the actual coordinate for other res trees
      point3d point = m_octree->keyToCoord(*it);
      RoughOcTreeNode *oNode = m_octree->updateNode(*it, false);
      // Update the merged map.  Using the coordinate allows for different resolutions
      RoughOcTreeNode *newNode = m_merged_tree->setNodeValue(point, oNode->getValue());
      newNode->setAgent(1);
      if (m_enableTraversability) {
        newNode->setRough(oNode->getRough());
      }
      // Build the map of what the secondary sensor can see
      if (m_buildCameraMap && pointInsideView(point)) {
        m_camera_tree->updateNode(point, false);
      }
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    point3d point = m_octree->keyToCoord(*it);
    RoughOcTreeNode *oNode = m_octree->updateNode(*it, true);
    RoughOcTreeNode *newNode = m_merged_tree->setNodeValue(point, oNode->getValue());
    newNode->setAgent(1);
    if (m_enableTraversability) {
      newNode->setRough(oNode->getRough());
    }
    if (m_buildCameraMap && pointInsideView(point)) {
      m_camera_tree->updateNode(point, true);
    }
  }

  return delayTime;
}

void MarbleMapping::insertStairCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  if (m_downsampleSize > 0.0) {
    pcl::VoxelGrid<PCLPoint> voxel_filter;
    voxel_filter.setInputCloud (pc.makeShared());
    voxel_filter.setFilterFieldName("x");
    voxel_filter.setLeafSize (m_downsampleSize, m_downsampleSize, m_downsampleSize);
    voxel_filter.filter (pc);
  }

  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  // directly transform to map frame:
  pcl::transformPointCloud(pc, pc, sensorToWorld);

  if (m_passthroughFilter) {
    // set up filter for height range, also removes NANs:
    pcl::PassThrough<PCLPoint> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pcl::PassThrough<PCLPoint> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pcl::PassThrough<PCLPoint> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);
  }

  insertStairScan(sensorToWorldTf, pc);
}

void MarbleMapping::insertStairScan(const tf::StampedTransform& sensorToWorldTf, const PCLPointCloud& pc){
  point3d sensorOrigin = pointTfToOctomap(sensorToWorldTf.getOrigin());

  // Profile our processing time
  ros::WallTime startTime = ros::WallTime::now();

  boost::mutex::scoped_lock lock(m_mtx);
  for (PCLPointCloud::const_iterator it = pc.begin(); it < pc.end(); ++it) {
    point3d point(it->x, it->y, it->z);

    // minrange / maxrange check
    if (((m_minRange <= 0.0) || ((point - sensorOrigin).norm() >= m_minRange)) &&
        ((m_maxRange <= 0.0) || ((point - sensorOrigin).norm() <= m_maxRange))) {

      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)) {
        RoughOcTreeNode *oNode = m_octree->integrateNodeStairs(key, it->intensity > 0.5);
        // The key might be different if the resolutions are different
        key = m_merged_tree->coordToKey(point);
        // Check if it was a stair node before the update
        RoughOcTreeNode *newNode = m_merged_tree->search(key);
        if (newNode != NULL) {
          bool was_stairs = m_merged_tree->isNodeStairs(newNode);
          // Set the new value
          newNode->setStairLogOdds(oNode->getStairLogOdds());
          // Convert the key back to the coordinate this is actually at
          point3d mpoint = m_merged_tree->keyToCoord(key);
          // If it's a stair point, add to the stair set
          // If it was a stair point, but now isn't, remove it
          if (m_merged_tree->isNodeStairs(newNode)) {
            m_stairPoints.insert(mpoint);
          } else if (was_stairs) {
            m_stairPoints.erase(mpoint);
          }
        }
      }
    }
  }
  lock.unlock();

  // Find the total time for stair processing
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("The %zu stair points took %f seconds)", pc.size(), total_elapsed);
}

void MarbleMapping::processStairsLoop(const ros::TimerEvent& event) {

  bool publishStairPointsArray = m_publishStairPointsArray && (m_stairPointsPub.getNumSubscribers() > 0);
  bool publishStairEdgeArray = m_publishStairEdgeArray && (m_stairEdgePub.getNumSubscribers() > 0);

  pcl::PointCloud<pcl::PointXYZ> stairCloud;
  // Convert the point3d set into a point cloud
  for (auto& point : m_stairPoints) {
    pcl::PointXYZ spoint;
    spoint.x = point.x(); spoint.y = point.y(); spoint.z = point.z();
    stairCloud.push_back(spoint);
  }

  // abort if no stair points
  if (stairCloud.height * stairCloud.width == 0) {
    ROS_DEBUG("No stairs mapped. Skipping stair processing...");
    return;
  }

  ROS_DEBUG("Extracting stair clusters.");
  // extract clusters
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointsVec;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr stairCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(stairCloud));
  kdtree->setInputCloud(stairCloudPtr);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extractor;
  euclidean_cluster_extractor.setClusterTolerance(1.8*m_res);
  euclidean_cluster_extractor.setMinClusterSize(15); // Cluster must be at least 15 voxels in size
  euclidean_cluster_extractor.setSearchMethod(kdtree);
  euclidean_cluster_extractor.setInputCloud(stairCloudPtr);
  euclidean_cluster_extractor.extract(cluster_indices);

  for (uint i=0; i < cluster_indices.size(); i++) {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices(cluster_indices[i]));
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(stairCloudPtr);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud);
    pointsVec.push_back(cloud);
  }

  // pub stair points marker array
  if (publishStairPointsArray) {
    ROS_DEBUG("Publishing stair voxels.");
    visualization_msgs::MarkerArray stairPointsArray;
    visualization_msgs::Marker stairPointsMarker;
    stairPointsMarker.action = visualization_msgs::Marker::DELETEALL;
    stairPointsArray.markers.push_back(stairPointsMarker);
    m_stairPointsPub.publish(stairPointsArray);

    float size = m_res;
    for (uint i=0; i < pointsVec.size(); i++) {
      visualization_msgs::Marker stairPointsMarker;
      stairPointsMarker.header.frame_id = m_worldFrameId;
      stairPointsMarker.header.stamp = ros::Time::now();
      stairPointsMarker.ns = "map";
      stairPointsMarker.id = i;
      stairPointsMarker.type = visualization_msgs::Marker::CUBE_LIST;
      stairPointsMarker.scale.x = size;
      stairPointsMarker.scale.y = size;
      stairPointsMarker.scale.z = size;
      RGBColor color = ratioToRGB((float)(i+1) / (float)pointsVec.size());
      stairPointsMarker.color.r = color.r;
      stairPointsMarker.color.g = color.g;
      stairPointsMarker.color.b = color.b;
      stairPointsMarker.color.a = 1.0;

      for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pointsVec[i]->begin(); it < pointsVec[i]->end(); ++it) {
        geometry_msgs::Point pt;
        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;
        stairPointsMarker.points.push_back(pt);
      }

      stairPointsArray.markers.push_back(stairPointsMarker);
    }
    m_stairPointsPub.publish(stairPointsArray);
  }

  ROS_DEBUG("Extracting stair edges.");
  // extract stair edges
  std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ>> stair_edges;
  stair_edges.resize(pointsVec.size());
  {
    for (uint i=0; i < pointsVec.size(); i++) {
      Eigen::Vector3f upVec; upVec << 0, 0, 1;

      pcl::PCA<pcl::PointXYZ> pca;
      pca.setInputCloud (pointsVec[i]);
      Eigen::Vector3f eigenVals = pca.getEigenValues();
      Eigen::Matrix3f eigenVecs = pca.getEigenVectors();

      // always point up
      Eigen::Vector3f eigenDotProds;
      for (uint j=0; j < 3; j++) {
        eigenDotProds(j) = upVec.dot(eigenVecs.col(j));
        if (eigenDotProds(j) < 0.f) {
          eigenVecs.col(j) *= -1.f;
          eigenDotProds(j) *= -1.f;
        }
      }

      float optPitchDeg = 90.f-35.f;
      float optPitchDot = cos(DEG2RAD(optPitchDeg));
      float pitchTolDeg = 5.f;
      float minPitchDot = cos(DEG2RAD(optPitchDeg+pitchTolDeg));
      float maxPitchDot = cos(DEG2RAD(optPitchDeg-pitchTolDeg));
      int optPitchEigIdx = -1;
      float minPitchErr = 1.f;
      for (uint j=0; j < 2; j++) {
        float thisMinPitchErr = abs(optPitchDot-eigenDotProds(j));
        if (thisMinPitchErr < minPitchErr) {
          minPitchErr = thisMinPitchErr;
          optPitchEigIdx = j;
        }
      }
      if (optPitchEigIdx==-1)
        continue;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected (new pcl::PointCloud<pcl::PointXYZ>);
      pca.project(*pointsVec[i], *cloudProjected);
      pcl::PointXYZ minPointProjected, maxPointProjected, minPoint, maxPoint;
      pcl::getMinMax3D(*cloudProjected, minPointProjected, maxPointProjected);
      switch (optPitchEigIdx) {
        case 0:
          minPointProjected.y = 0;
          minPointProjected.z = 0;
          maxPointProjected.y = 0;
          maxPointProjected.z = 0;
          break;
        case 1:
          minPointProjected.x = 0;
          minPointProjected.z = 0;
          maxPointProjected.x = 0;
          maxPointProjected.z = 0;
          break;
        case 2:
          minPointProjected.y = 0;
          minPointProjected.x = 0;
          maxPointProjected.y = 0;
          maxPointProjected.x = 0;
          break;
        default:
          break;
      }
      pca.reconstruct(minPointProjected, minPoint);
      pca.reconstruct(maxPointProjected, maxPoint);

      stair_edges[i].first = minPoint;
      stair_edges[i].second = maxPoint;
    }
  }

  // pub stair edge array
  if (publishStairEdgeArray) {
    ROS_DEBUG("Publishing stair edges.");
    visualization_msgs::MarkerArray stairEdgeArray;
    visualization_msgs::Marker stairEdgeMarker;
    stairEdgeMarker.action = visualization_msgs::Marker::DELETEALL;
    stairEdgeArray.markers.push_back(stairEdgeMarker);
    m_stairEdgePub.publish(stairEdgeArray);

    for (uint i=0; i < pointsVec.size(); i++) {
      if (stair_edges[i].first.x==0.f &&
          stair_edges[i].first.y==0.f &&
          stair_edges[i].first.z==0.f &&
          stair_edges[i].second.x==0.f &&
          stair_edges[i].second.y==0.f &&
          stair_edges[i].second.z==0.f) {
        continue;
      }

      visualization_msgs::Marker stairEdgeMarker;
      stairEdgeMarker.header.frame_id = m_worldFrameId;
      stairEdgeMarker.header.stamp = ros::Time::now();
      stairEdgeMarker.ns = "map";
      stairEdgeMarker.id = i;
      stairEdgeMarker.type = visualization_msgs::Marker::LINE_LIST;
      stairEdgeMarker.action = visualization_msgs::Marker::ADD;
      stairEdgeMarker.scale.x = 0.1;
      stairEdgeMarker.scale.y = 0.1;
      stairEdgeMarker.scale.z = 0.1;
      RGBColor color = ratioToRGB((float)(i+1) / (float)pointsVec.size());
      stairEdgeMarker.color.r = color.r;
      stairEdgeMarker.color.g = color.g;
      stairEdgeMarker.color.b = color.b;
      stairEdgeMarker.color.a = 1.0;

      geometry_msgs::Point minPointMsg, maxPointMsg;
      minPointMsg.x = stair_edges[i].first.x;
      minPointMsg.y = stair_edges[i].first.y;
      minPointMsg.z = stair_edges[i].first.z;
      stairEdgeMarker.points.push_back(minPointMsg);
      maxPointMsg.x = stair_edges[i].second.x;
      maxPointMsg.y = stair_edges[i].second.y;
      maxPointMsg.z = stair_edges[i].second.z;
      stairEdgeMarker.points.push_back(maxPointMsg);

      stairEdgeArray.markers.push_back(stairEdgeMarker);
    }
    m_stairEdgePub.publish(stairEdgeArray);
  }

  setStairCloud(stairCloud);
}

void MarbleMapping::pubNearStairIndicatorLoop(const ros::TimerEvent& event) {
  procAndPubNearStairIndicator();
}

void MarbleMapping::setStairCloud(pcl::PointCloud<pcl::PointXYZ>& stairCloud) {
  m_stairCloud = stairCloud;
}

void MarbleMapping::procAndPubNearStairIndicator() {
  bool publishNearStairIndicator = m_publishNearStairIndicator && ((m_nearStairIndicatorPub.getNumSubscribers() > 0) || (m_slowdownForStairsIndicatorPub.getNumSubscribers() > 0));
  int stairCloudSize = m_stairCloud.width*m_stairCloud.height;
  if (publishNearStairIndicator) {
    if (stairCloudSize > 0) {
      ROS_DEBUG("Publishing near stair indicator.");
      // get current position
      tf::StampedTransform baseToWorldTf;
      try {
        m_tfListener.waitForTransform(m_worldFrameId, m_baseFrameId, ros::Time::now(), ros::Duration(0.2));
        m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, ros::Time(0), baseToWorldTf);
      } catch (tf::TransformException& ex) {
        ROS_ERROR("near stair transform error %s", ex.what());
        return;
      }
      pcl::PointXYZ pos(baseToWorldTf.getOrigin().getX(), baseToWorldTf.getOrigin().getY(), baseToWorldTf.getOrigin().getZ());

      // get nearest stair position
      pcl::PointXYZ nearest_stair_pos;
      float nearest_stair_dist = INFINITY;
      for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = m_stairCloud.begin(); it < m_stairCloud.end(); ++it) {
        float stair_dist = sqrt(pow(it->x-pos.x,2)+pow(it->y-pos.y,2));
        if (stair_dist < nearest_stair_dist && abs(it->z-pos.z)<m_isNearStairZThresh) {
          nearest_stair_pos = *it;
          nearest_stair_dist = stair_dist;
        }
      }

      // abort if no nearby stair points (shouldn't be necessary bc stairCloud should not be empty but just in case)
      if (nearest_stair_pos.x == 0.f && nearest_stair_pos.y == 0.f && nearest_stair_pos.z == 0.f)
        return;

      // use nearest stair position to determine if near stairs (ie should trigger stair mode)
      if (!m_isNearStairs && (pcl::euclideanDistance(nearest_stair_pos,pos) < m_isNearStairInnerRad)) {
        // was not prev near stairs but am now
        m_isNearStairs = true;
      } else if (m_isNearStairs && (pcl::euclideanDistance(nearest_stair_pos,pos) > m_isNearStairOuterRad)) {
        // was prev near stairs but no longer am
        m_isNearStairs = false;
      }

      m_slowdownForStairs = false;
      if (pcl::euclideanDistance(nearest_stair_pos,pos) < m_slowdownForStairsRad)
        m_slowdownForStairs = true;

      // publish near stair indicator
      std_msgs::Bool isNearStairMsg;
      isNearStairMsg.data = m_isNearStairs;
      m_nearStairIndicatorPub.publish(isNearStairMsg);
      std_msgs::Bool slowdownForStairsMsg;
      slowdownForStairsMsg.data = m_slowdownForStairs;
      m_slowdownForStairsIndicatorPub.publish(slowdownForStairsMsg);
    } else {
      m_isNearStairs = false;
      std_msgs::Bool isNearStairMsg;
      isNearStairMsg.data = m_isNearStairs;
      m_nearStairIndicatorPub.publish(isNearStairMsg);
      m_slowdownForStairs = false;
      std_msgs::Bool slowdownForStairsMsg;
      isNearStairMsg.data = m_slowdownForStairs;
      m_slowdownForStairsIndicatorPub.publish(slowdownForStairsMsg);
    }
  }
}

template <class OcTreeMT>
int MarbleMapping::updateDiffTree(OcTreeMT* tree, PCLPointCloudRGB& pclDiffCloud) {
  // Find all changed nodes since last update and create a new octomap
  KeyBoolMap::const_iterator startPnt = tree->changedKeysBegin();
  KeyBoolMap::const_iterator endPnt = tree->changedKeysEnd();

  double minZ, maxZ;
  if (m_publishPointCloudDiff) {
    double minX, minY, maxX, maxY;
    tree->getMetricMin(minX, minY, minZ);
    tree->getMetricMax(maxX, maxY, maxZ);
    minZ = std::min(-1.0, minZ);
  }

  int num_nodes = 0;
  for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
    // Copy the value for each node
    RoughOcTreeNode* node = tree->search(iter->first);

    // We should never have a NULL node, but just in case...
    if (node != NULL) {
      RoughOcTreeNode *newNode = m_diff_tree->setNodeValue(iter->first, node->getLogOdds());
      if (m_enableTraversabilitySharing) {
        newNode->setRough(node->getRough());
      }
      num_nodes++;

      // Create a point cloud diff if enabled
      if ((m_publishPointCloudDiff) && (node->getLogOdds() >= 0)) {
        point3d point = tree->keyToCoord(iter->first);
        PCLPointRGB pcpoint;
        pcpoint.x = point.x();
        pcpoint.y = point.y();
        pcpoint.z = point.z();
        RGBColor _color = node->getAgentColor(pcpoint.z, minZ, maxZ, m_adjustAgent);
        pcpoint.r = _color.r * 255;
        pcpoint.g = _color.g * 255;
        pcpoint.b = _color.b * 255;

        pclDiffCloud.push_back(pcpoint);
      }
    }
  }

  return num_nodes;
}

void MarbleMapping::updateDiff(const ros::TimerEvent& event) {
  // TODO Add a check to see if we're in comm with anyone, and if not, don't update the diff
  PCLPointCloudRGB pclDiffCloud;
  int num_nodes;

  // Update the diff tree from either the merged map or self map
  boost::mutex::scoped_lock lock(m_mtx);
  m_diff_tree->clear();
  if (m_diffMerged)
    num_nodes = updateDiffTree(m_merged_tree, pclDiffCloud);
  else
    num_nodes = updateDiffTree(m_octree, pclDiffCloud);
  lock.unlock();

  // Only update the diff map if enough has changed
  if (num_nodes > diff_threshold) {
    if (m_publishDiffs) {
      num_diffs++;
      m_diff_tree->prune();

      // Create and publish message for this diff
      octomap_msgs::Octomap msg;
      octomap_msgs::binaryMapToMsg(*m_diff_tree, msg);
      msg.header.frame_id = m_worldFrameId;
      msg.header.stamp = ros::Time().now();
      msg.header.seq = num_diffs - 1;
      m_diffMapPub.publish(msg);

      // Add the diff to the map diffs array
      mapdiffs.octomaps.push_back(msg);
      mapdiffs.num_octomaps = num_diffs;
      m_diffsMapPub.publish(mapdiffs);
    }

    // Publish the diff point cloud if enabled
    if (m_publishPointCloudDiff) {
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg (pclDiffCloud, cloud);
      cloud.header.frame_id = m_worldFrameId;
      cloud.header.stamp = ros::Time().now();
      m_pointCloudDiffPub.publish(cloud);
    }

    boost::mutex::scoped_lock lock(m_mtx);
    if (m_diffMerged)
      m_merged_tree->resetChangeDetection();
    else
      m_octree->resetChangeDetection();
  }

  // Stats on dropped clouds
  if (pclDropped > 0) {
    double dropped = (double)pclDropped / (double)pclCount * 100.0;
    ROS_INFO("Dropped %.1f%% of clouds.  Largest delay was %.2f seconds.", dropped, longTimeDiff.toSec());
    pclDropped = 0;
    longTimeDiff = ros::Duration(0.0);
  }

  if (pclCount > 0) {
    double pcls = pclCountProcessed / (ros::Time::now() - event.last_real).toSec();
    ROS_INFO("Processed %d clouds, %0.2f pcl/sec, %0.4f average", pclCountProcessed, pcls, pclTime / pclCountProcessed);
    pclCount = 0;
    pclCountProcessed = 0;
    pclTime = 0;
  }
}

void MarbleMapping::parseIDXandAgent(std::string& cidx_str) {
  std::string agent;

  // Get the idx (number before colon) and adjust the next_idx if bigger
  auto cidx_end = cidx_str.find(":");
  int cidx = std::stoi(cidx_str.substr(0, cidx_end));
  if (cidx > next_idx) next_idx = cidx + 1;

  // Parse the rest of the string into each agent and store
  auto start = cidx_end + 1;
  auto end = 0;
  while ((end = cidx_str.find(",", start)) != std::string::npos) {
    agent = cidx_str.substr(start, end-start);
    // Store in our map for later use
    color_pairs[agent.data()] = cidx;
    start = end + 1;
  }
  // Store the last agent as well
  agent = cidx_str.substr(start, end);
  color_pairs[agent.data()] = cidx;
}

void MarbleMapping::parseColorPairs(std::string& color_pairs_str) {
  auto start = 0;
  auto end = 0;
  std::string cidx_str;
  // Get each idx/agent group
  while ((end = color_pairs_str.find(";", start)) != std::string::npos) {
    cidx_str = color_pairs_str.substr(start, end-start);
    // Find the idx and agents and store
    parseIDXandAgent(cidx_str);
    start = end + 1;
  }
  // Get the last set
  cidx_str = color_pairs_str.substr(start, end);
  parseIDXandAgent(cidx_str);
}

char MarbleMapping::nidToIDX(std::string& nid) {
  char nidx = 127;
  // See if there's already an idx configured for this nid
  if (color_pairs.count(nid.data()))
    nidx = color_pairs[nid.data()];

  // If not, use the next available
  if (nidx == 127) {
    nidx = next_idx;
    next_idx++;
  }

  return nidx;
}

void MarbleMapping::updateNeighborMaps() {
  // Merge neighbor maps with local map
  std::shared_ptr<RoughOcTreeT> ntree(nullptr);
  bool remerge;
  char agent;

  for (int i=0; i < neighbors.neighbors.size(); i++) {
    std::string nid = neighbors.neighbors[i].owner;
    // ignore self maps - should be able to remove this?
    if (nid == "H01") {
      // ROS_INFO("Won't merge self maps");
      continue;
    }
    ROS_INFO("Adding %s maps \n", nid.c_str());
    // If clear passed then re-merge everything
    if (neighbors.clear) {
      ROS_INFO("Clearing data for %s", nid.data());
      if (i == 0) {
        boost::mutex::scoped_lock lock(m_mtx);
        // First time through clear the merged tree, and reset change detection if necessary
        m_merged_tree->clear();

        // Copy the self map to merged map
        for (RoughOcTreeT::iterator it = m_octree->begin(), end = m_octree->end();
            it != end; ++it) {
          point3d point = it.getCoordinate();
          RoughOcTreeNode *newNode = m_merged_tree->setNodeValue(point, it->getLogOdds());
          newNode->setAgent(1);
          if (m_enableTraversability) {
            newNode->setRough(it->getRough());
          }
        }

        if (m_diffMerged)
          m_merged_tree->resetChangeDetection();
      }
      // For each neighbor, clear the diffs
      seqs[nid.data()].clear();
      if (m_publishNeighborMaps && neighbor_maps.count(nid.data()))
          neighbor_maps[nid.data()]->clear();
    }

    // Initialize neighbor map data if enabled
    if (m_publishNeighborMaps && !neighbor_maps.count(nid.data()) &&
        neighbors.neighbors[i].num_octomaps > 0 &&
        neighbors.neighbors[i].num_octomaps == neighbors.neighbors[i].octomaps.size()) {
      ROS_INFO("Adding neighbor map %s", nid.data());
      neighbor_maps[nid.data()] = new RoughOcTreeT(m_mres);
      neighbor_maps[nid.data()]->setRoughEnabled(m_enableTraversabilitySharing);
      neighbor_updated[nid.data()] = false;
      neighbor_pubs[nid.data()] = m_nh.advertise<Octomap>("neighbors/"+nid+"/map", 1, m_latchedTopics);
    }

    remerge = false;
    // Check each diff for new ones to merge
    // Array should be in sequence order to allow re-merging if a diff comes out of sequence
    // In theory should be able just skip nodes in out-of-sequence diffs,
    // but doesn't always work as expected due to various node sizes
    for (int j=0; j < neighbors.neighbors[i].octomaps.size(); j++) {
      uint32_t cur_seq = neighbors.neighbors[i].octomaps[j].header.seq;
      bool exists = std::count(seqs[nid.data()].cbegin(), seqs[nid.data()].cend(), cur_seq);

      // Only merge if we haven't already merged this sequence, or we got an out-of-sequence diff
      if (!exists || remerge) {
        // Add to our list of sequences, and get a unique agent id for this neighbor
        if (!exists) 
          seqs[nid.data()].push_back(cur_seq);
        if (idx[nid.data()])
          agent = idx[nid.data()];
        else {
          agent = nidToIDX(nid);
          idx[nid.data()] = agent;
          next_idx++;
        }

        // Check if this is an out-of-sequence diff, and force remerge of later sequences we have
        remerge = cur_seq < *std::max_element(seqs[nid.data()].cbegin(), seqs[nid.data()].cend());

        if (neighbors.neighbors[i].octomaps[j].binary) {
          // Backwards compatibility for regular OcTrees
          if (neighbors.neighbors[i].octomaps[j].id.find("RoughOcTree-") == std::string::npos)
            neighbors.neighbors[i].octomaps[j].id = "RoughOcTree-0";
          ntree = std::shared_ptr<RoughOcTreeT>(dynamic_cast<RoughOcTreeT*>
                    (octomap_msgs::binaryMsgToMap(neighbors.neighbors[i].octomaps[j])));
        } else
          ntree = std::shared_ptr<RoughOcTreeT>(dynamic_cast<RoughOcTreeT*>
                    (octomap_msgs::fullMsgToMap(neighbors.neighbors[i].octomaps[j])));

        // Iterate through the diff tree and merge
        ntree->expand();
        boost::mutex::scoped_lock lock(m_mtx);
        for (RoughOcTreeT::iterator it = ntree->begin(), end = ntree->end(); it != end; ++it) {
          OcTreeKey nodeKey = it.getKey();
          RoughOcTreeNode *nodeM = m_merged_tree->search(nodeKey);
          if (nodeM != NULL) {
            // Ignore any nodes that are self nodes
            if (nodeM->getAgent() != 1) {
              if (nodeM->getAgent() == agent) {
                // If the diff is newer, and the merged map node came from this neighbor
                // replace the value and maintain the agent id
                m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
                if (m_enableTraversabilitySharing) {
                  nodeM->setRough(it->getRough());
                }
              } else {
                // If there's already a node in the merged map, but it's from another neighbor,
                // or it's been previously merged, merge the value, and set agent to merged
                m_merged_tree->updateNode(nodeKey, it->getLogOdds());
                nodeM->setAgent(0);
                if (m_enableTraversabilitySharing) {
                  m_merged_tree->integrateNodeRough(nodeKey, it->getRough());
                }
              }
            }
          } else {
            // If the node doesn't exist in the merged map, add it with the value
            RoughOcTreeNode *newNode = m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
            newNode->setAgent(agent);
            if (m_enableTraversabilitySharing) {
              newNode->setRough(it->getRough());
            }
          }

          // If neighbor maps enabled, add the node if it's new (or needs remerge)
          if (m_publishNeighborMaps && neighbor_maps.count(nid.data())) {
            neighbor_updated[nid.data()] = true;
            RoughOcTreeNode *newNode = neighbor_maps[nid.data()]->setNodeValue(nodeKey, it->getLogOdds());
            if (m_enableTraversabilitySharing) {
              newNode->setRough(it->getRough());
            }
          }
        }
      }
    }
  }
  m_mtree_updated = true;
  m_mtree_markers_updated = true;
}

void MarbleMapping::mergeNeighbors(const ros::TimerEvent& event)
{
  // change neighbor maps aligned bool status
  if (!m_neighborMapsMerged) {
    ROS_INFO("Running merge procedure");
    // add self map to merged map
    m_octree->expand();
    for (RoughOcTreeT::iterator it = m_octree->begin(), end=m_octree->end();
         it != end; ++it) {
      point3d pt = it.getCoordinate();
      RoughOcTreeNode* newNode = m_merged_tree->setNodeValue(pt, it->getLogOdds());
      newNode->setAgent(1);
      if (m_enableTraversability)
        newNode->setRough(it->getRough());
    } 
    // loop through stored neighbor maps, transform and merge
    for (const auto& n : neighbor_maps) {
      // store neighbor name
      std::string nid = n.first.data();
      // update neighbor updated status
      neighbor_aligned[nid.data()] = true;
      RoughOcTreeT* neighbor_map = neighbor_maps[nid.data()];

      // align map
      neighbor_map->prune();
      Eigen::Matrix4f est_tf = alignMap(neighbor_map);
      neighbor_tf[nid.data()] = est_tf; // store neighbor map tf
      ROS_INFO("Aligning maps for %s", nid.c_str());

      // transform map
      transformMap(neighbor_map, est_tf);
      ROS_INFO("Transforming maps for %s", nid.c_str());

      // merge map
      mergeMap(neighbor_map);
      ROS_INFO("Merging maps for %s \n", nid.c_str());
    }
    m_neighborMapsMerged = true;
  }
  else {
    ROS_INFO("Already aligned");
  }
}

Eigen::Matrix4f MarbleMapping::alignMap(const RoughOcTreeT* tree)
{
  // convert tree to point cloud
  icpPointCloud tree_pc; // pruned in 
  tree2PointCloud(tree, tree_pc);

  // convert merged map to point cloud
  icpPointCloud merged_pc;
  m_merged_tree->prune();
  tree2PointCloud(m_merged_tree, merged_pc);

  // get estimated transform from GICP
  Eigen::Matrix4f est_tf = getGICPTransform(tree_pc, merged_pc);

  return est_tf;
}

void MarbleMapping::mergeMap(const RoughOcTreeT* tree)
{
  // iterate through tree and merge
  for (RoughOcTreeT::iterator it = tree->begin(), end = tree->end(); it != end; ++it) {
    OcTreeKey nodeKey = it.getKey();
    RoughOcTreeNode* nodeM = m_merged_tree->search(nodeKey);
    RoughOcTreeNode* nodeCurr = tree->search(nodeKey);
    char agent = nodeCurr->getAgent();
    if (nodeM != NULL) {
      // ignore any nodes that are self nodes
      if (nodeM->getAgent() != 1) {
        if (nodeM->getAgent() == agent) {
          // if the diff is newer, and the merged map node came from this neighbor
          // replace the value and maintain the agent id
          m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
          if (m_enableTraversabilitySharing)
            nodeM->setRough(it->getRough());
        }
        else {
          // if there's already a node in the merged map, but it's from another neighbor,
          // or it's been previously merged, merge the value, and set agent to merged
          m_merged_tree->updateNode(nodeKey, it->getLogOdds());
          nodeM->setAgent(0);
          if (m_enableTraversabilitySharing)
            m_merged_tree->integrateNodeRough(nodeKey, it->getRough());
        }
      }
    }
    else {
      // if the node doesn't exist in the merged map, add it with the value
      RoughOcTreeNode* newNode = m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
      newNode->setAgent(agent);
      if (m_enableTraversabilitySharing)
        newNode->setRough(it->getRough());
    }
  }
}

void MarbleMapping::tree2PointCloud(const RoughOcTreeT* tree, icpPointCloud& cloud)
{
  for (RoughOcTreeT::iterator it = tree->begin(), end = tree->end(); it != end; ++it) {
    if (tree->isNodeOccupied(*it))
      cloud.push_back(icpPoint(it.getX(), it.getY(), it.getZ()));
  }
}

Eigen::Matrix4f MarbleMapping::getGICPTransform(icpPointCloud& source, icpPointCloud& target)
{
  // find min and max points of src point cloud
  icpPoint srcMin, srcMax;
  pcl::getMinMax3D(source, srcMin, srcMax);
  icpPointCloud::Ptr src(new icpPointCloud);

  // find min and max points of trg point cloud
  icpPoint trgMin, trgMax;
  pcl::getMinMax3D(target, trgMin, trgMax);
  icpPointCloud::Ptr trg(new icpPointCloud);

  // find all points in src that are within trg bounds
  for (icpPointCloud::iterator it = source.begin(); it != source.end(); ++it) {
    if ((trgMin.x < it->x && trgMin.y < it->y && trgMin.z < it->z) && 
        (trgMax.x > it->x && trgMax.y > it->y && trgMax.z > it->z))
          src->push_back(*it);
  }

  // find all points in trg that are within src bounds
  for (icpPointCloud::iterator it = target.begin(); it != target.end(); ++it) {
    if ((srcMin.x < it->x && srcMin.y < it->y && srcMin.z < it->z) && 
        (srcMax.x > it->x && srcMax.y > it->y && srcMax.z > it->z))
          trg->push_back(*it);
  }

  // Downsample
  pcl::VoxelGrid<icpPoint> grid;
  grid.setLeafSize(m_leafScale * m_res, m_leafScale * m_res, m_leafScale * m_res);
  // downsample src cloud
  grid.setInputCloud(src);
  grid.filter(*src);
  // downsample trg cloud
  grid.setInputCloud(trg);
  grid.filter(*trg);

  // Statistical outlier filter to remove spurious points in PCs
  pcl::StatisticalOutlierRemoval<icpPoint> sor;
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stdDevMulThresh);
  sor.setInputCloud(src);
  sor.filter(*src);
  sor.setInputCloud(trg);
  sor.filter(*trg);

  // begin registration
  pcl::GeneralizedIterativeClosestPoint<icpPoint, icpPoint> gicp;
  gicp.setMaximumIterations(icpIterations);
  gicp.setMaximumOptimizerIterations(icpOptimizerIterations);
  gicp.setMaxCorrespondenceDistance(icpMaxCorrespDistScale * m_res);
  gicp.setTransformationEpsilon(icpTFEpsScale);
  gicp.setEuclideanFitnessEpsilon(euclideanFitnessEps);
  gicp.setRANSACOutlierRejectionThreshold(ransacOutlierRejecThresh * m_res);
  gicp.setRANSACIterations(ransacNumIterations);

  // setup output tf and cloud  
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev;
  Eigen::Matrix4f trgToSrcTF;
  icpPointCloud::Ptr gicp_result;

  if (src->size() < trg->size()) 
  {
    gicp.setInputSource(src);
    gicp.setInputTarget(trg);
    gicp_result = src;
    gicp.align(*gicp_result);
    Ti = gicp.getFinalTransformation();
  }
  else
  {
    gicp.setInputSource(trg);
    gicp.setInputTarget(src);
    gicp_result = trg;
    gicp.align(*gicp_result);
    Ti = gicp.getFinalTransformation();
    Ti = Ti.inverse();
  }
  // get the transformation from target to source
  trgToSrcTF = Ti;
  // get transformation fitness score
  double fs = gicp.getFitnessScore(0.05);

  ROS_INFO("GICP fitness score: %f", fs);
  if (fs > fitnessScoreThreshold)
    ROS_INFO("GICP Fitness Score above threshold. Storing map and will reattempt merge next time around.");
  // check for GICP convergence 
  if (gicp.hasConverged())
    ROS_INFO("GICP converged");
  else
    ROS_INFO("GICP did not converge");

  // clear filtered PCs
  src->clear();
  trg->clear();
  
  return trgToSrcTF;
}

void MarbleMapping::transformMap(RoughOcTreeT* tree, const Eigen::Matrix4f& tf)
{
  double res = m_res;
  RoughOcTreeT* transformed = new RoughOcTreeT(res);

  // build inverse transform
  Eigen::Matrix3f rotation = tf.block<3,3> (0,0); // extract rotation
  Eigen::Matrix3f invRotation = rotation.inverse();
  Eigen::Matrix4f invTransform = tf.inverse();

  // size in each coordinate of each axis
  double minX, maxX, minY, maxY, minZ, maxZ;

  // get the min and max in so we can step along each row
  // boost::mutex::scoped_lock lock(m_mtx);
  tree->getMetricMin(minX, minY, minZ);
  tree->getMetricMax(maxX, maxY, maxZ);
  // lock.unlock();

  // allocate a vector of points
  std::vector<octomap::point3d> points;

  // make 8 points to make a map bounding box, performing the tf on them
  // to get the range of values in the transformed map
  points.push_back(point3d(minX, minY, minZ));
  points.push_back(point3d(minX, minY, maxZ));
  points.push_back(point3d(minX, maxY, minZ));
  points.push_back(point3d(minX, maxY, maxZ));
  points.push_back(point3d(maxX, minY, minZ));
  points.push_back(point3d(maxX, minY, maxZ));
  points.push_back(point3d(maxX, maxY, minZ));
  points.push_back(point3d(maxX, maxY, maxZ));

  // transform points that define bounding box
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector4f point(points[i].x(), points[i].y(), points[i].z(), 1.);
    point = tf * point;
    points[i] = point3d(point(0), point(1), point(2));
  }
  // find min and max for each axis
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector3f pt(points[i].x(), points[i].y(), points[i].z());
    if (pt(0) < minX)
      minX = pt(0);
    if (pt(1) < minY)
      minY = pt(1);
    if (pt(2) < minZ)
      minZ < pt(2);
    if (pt(0) > maxX)
      maxX = pt(0);
    if (pt(1) > maxY)
      maxY = pt(1);
    if (pt(2) > maxZ)
      maxZ = pt(2);
  }
  
  // go through the possible destination voxels on a row by row basis
  // and calculate occupancy from source voxels with inverse tf
  // lock.lock();
  for (double z = minZ - res / 2; z < (maxZ + res / 2); z += res) {
    for (double y = minY - res / 2; y < (maxY + res / 2); y += res) {
      for (double x = minX - res / 2; x < (maxX + res / 2); x += res) {
        OcTreeKey destVoxel = transformed->coordToKey(point3d(x,y,z));

        Eigen::Vector4f point(x,y,z,1);
        point = invTransform * point;
        point3d sourcePoint = point3d(point(0), point(1), point(2));
        OcTreeKey sourceVoxel = tree->coordToKey(sourcePoint);
        point3d nn = tree->keyToCoord(sourceVoxel);

        // use nearest neighbour to set new occupancy in the transformed map
        OcTreeNode *oldNode = tree->search(sourceVoxel);

        // Occupancies to interpolate between
        double c000, c001, c010, c011, c100, c101, c110,
               c111, c00, c01, c10, c11, c0, c1;
        double xd,yd,zd;

        // differences in each direction between next closest voxel
        xd = (sourcePoint.x() - nn.x()) / res;
        yd = (sourcePoint.y() - nn.y()) / res;
        zd = (sourcePoint.z() - nn.z()) / res;

        if (oldNode != NULL) {
          c000 = oldNode->getOccupancy();
          OcTreeNode *node;

          // c001
          if ((node = tree->search(point3d(nn.x(), nn.y(), nn.z() +
                      getSign(zd) * res))) != NULL) {
            c001 = node->getOccupancy();
          } else
            c001 = 0;

          // c010
          if ((node = tree->search(point3d(nn.x(), nn.y() + 
                      getSign(yd) * res, nn.z()))) != NULL) {
            c010 =node->getOccupancy();
          } else
            c010 = 0;

          // c011
          if ((node = tree->search(point3d(nn.x(),nn.y() + 
                      getSign(yd) * res, nn.z() + getSign(zd) * res))) != NULL) {
            c011 = node->getOccupancy();
          } else
            c011 = 0;

          // c100
          if ((node = tree->search(point3d(nn.x() + getSign(xd) * res, nn.y(),
                      nn.z()))) != NULL) {
            c100 = node->getOccupancy();
          } else
            c100 = 0;

          // c101
          if ((node = tree->search(point3d(nn.x() + getSign(xd) * res, nn.y(),
                      nn.z() + getSign(zd) * res))) != NULL) {
            c101 = node->getOccupancy();
          } else
            c101 = 0;

          // c110
          if ((node = tree->search(point3d(nn.x() + getSign(xd) * res, nn.y() + 
                      getSign(yd) * res, nn.z()))) != NULL) {
            c110 = node->getOccupancy();
          } else
            c110 = 0;

          // c111
          if ((node = tree->search(point3d(nn.x() + getSign(xd) * res, nn.y() + 
                      getSign(yd) * res, nn.z() + getSign(zd) * res))) != NULL) {
            c111 = node->getOccupancy();
          } else
            c111 = 0;

          // interpolate in x
          c00 = (1-fabs(xd)) * c000 + fabs(xd) * c100;
          c10 = (1-fabs(xd)) * c010 + fabs(xd) * c110;
          c01 = (1-fabs(xd)) * c001 + fabs(xd) * c101;
          c11 = (1-fabs(xd)) * c011 + fabs(xd) * c111;

          // interpolate in y
          c0 = (1-fabs(yd)) * c00 + fabs(yd) * c10;
          c1 = (1-fabs(yd)) * c01 + fabs(yd) * c11;

          // assign the new node value
          OcTreeNode *newNode = transformed->updateNode(destVoxel, true);
          newNode->setLogOdds(octomap::logodds((1 - fabs(zd)) * c0 + fabs(zd) * c1));
        }
      }
    }
  }
  tree->swapContent(*transformed);
  // lock.unlock();
  delete transformed;
}

void MarbleMapping::publishOptionalMaps(const ros::TimerEvent& event) {
  // Publish optional maps such as marker arrays.  Performed in separate thread.
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time rostime = ros::Time::now();
  if (m_merged_tree->size() <= 1) {
    return;
  }

  // Check if we have any new subscribers, so we force an update without waiting for a new diff
  bool newMarkerSub = false;
  if (m_input != 1) {
    int curMarkerSub = m_markerPub.getNumSubscribers();
    newMarkerSub = curMarkerSub > m_lastMarkerSub;
    m_lastMarkerSub = curMarkerSub;
  }

  if (!m_mtree_markers_updated && !newMarkerSub) {
    return;
  }
  m_mtree_markers_updated = false;

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = m_publishMarkerArray && (m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = m_publishPointCloud && (m_pointCloudPub.getNumSubscribers() > 0);
  bool publishTravMarkerArray = m_publishTravMarkerArray && (m_travMarkerPub.getNumSubscribers() > 0);
  bool publishStairMarkerArray = m_publishStairMarkerArray && (m_stairMarkerPub.getNumSubscribers() > 0);

  if (!publishMarkerArray && !publishFreeMarkerArray && !publishPointCloud) {
    return;
  }

  // init markers:
  visualization_msgs::MarkerArray freeNodesVis, occupiedNodesVis, travMarkers, stairMarkers;

  // init pointcloud:
  pcl::PointCloud<PCLPointRGB> pclCloud;

  boost::mutex::scoped_lock lock(m_mtx);
  // Only traverse the tree if we're publishing a marker array or point cloud
  if (publishMarkerArray || publishFreeMarkerArray || publishPointCloud || publishTravMarkerArray || publishStairMarkerArray) {
    // each array stores all cubes of a different size, one for each depth level:
    freeNodesVis.markers.resize(m_treeDepth+1);
    occupiedNodesVis.markers.resize(m_treeDepth+1);

    double minX, minY, minZ, maxX, maxY, maxZ;
    m_merged_tree->getMetricMin(minX, minY, minZ);
    m_merged_tree->getMetricMax(maxX, maxY, maxZ);
    // Make sure the bottom is a little lower to drastic color changes at the beginning
    minZ = std::min(-1.0, minZ);

    // now, traverse all leafs in the tree:
    for (RoughOcTreeT::iterator it = m_merged_tree->begin(m_maxTreeDepth),
        end = m_merged_tree->end(); it != end; ++it) {
      if (m_merged_tree->isNodeOccupied(*it)) {
        // Skip this node if remove ceiling is enabled
        if (m_removeCeiling && isCeiling(it.getKey())) {
          continue;
        }

        double z = it.getZ();
        double half_size = it.getSize() / 2.0;
        if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
          double size = it.getSize();
          double x = it.getX();
          double y = it.getY();

          // Ignore speckles in the map:
          if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())) {
            ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
            continue;
          } // else: current octree node is no speckle, send it out

          // Color used for both markers and point cloud
          RGBColor _color;
          if (m_displayColor == 2 && m_merged_tree->getRoughEnabled()) {
            _color = it->getRoughColor();
          } else {
            _color = it->getAgentColor(z, minZ, maxZ, m_adjustAgent);
          }

          //create marker:
          if (publishMarkerArray) {
            unsigned idx = it.getDepth();
            assert(idx < occupiedNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            if ((m_displayColor == 1) || (m_displayColor == 2)) {
              std_msgs::ColorRGBA _color_msg; _color_msg.r = _color.r; _color_msg.g = _color.g; _color_msg.b = _color.b; _color_msg.a = 1.0f;
              occupiedNodesVis.markers[idx].colors.push_back(_color_msg);
            } else {
              double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
              occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
            }
          }

          // insert into pointcloud:
          if (publishPointCloud) {
            PCLPointRGB _point = PCLPointRGB();
            _point.x = x; _point.y = y; _point.z = z;
            _point.r = _color.r * 255;
            _point.g = _color.g * 255;
            _point.b = _color.b * 255;
            pclCloud.push_back(_point);
          }

          if (publishTravMarkerArray &&
              ((int)round(x / size) % m_travMarkerDensity == 0 &&
               (int)round(y / size) % m_travMarkerDensity == 0)) {
            char text_buf[50];
            sprintf(text_buf, "%0.2f", it->getRough());
            std::string text_str = text_buf;
            visualization_msgs::Marker marker;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z + size;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.text = text_buf;
            travMarkers.markers.push_back(marker);
          }

          if (publishStairMarkerArray &&
              ((int)round(x / size) % m_stairMarkerDensity == 0 &&
               (int)round(y / size) % m_stairMarkerDensity == 0)) {
            char text_buf[50];
            sprintf(text_buf, "%0.2f", it->getStairProbability());
            std::string text_str = text_buf;
            visualization_msgs::Marker marker;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z+size;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.text = text_buf;
            stairMarkers.markers.push_back(marker);
          }
        }
      } else { // node not occupied => mark as free in 2D map if unknown so far
        if (publishFreeMarkerArray) {
          double z = it.getZ();
          double half_size = it.getSize() / 2.0;
          if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
            double x = it.getX();
            double y = it.getY();

            //create marker for free space:
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }
      }
    }
  }

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_merged_tree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);
  }

  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_merged_tree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_fmarkerPub.publish(freeNodesVis);
  }

  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  if (publishTravMarkerArray){
    for (unsigned i= 0; i < travMarkers.markers.size(); ++i){
      double size = m_res;

      travMarkers.markers[i].header.frame_id = m_worldFrameId;
      travMarkers.markers[i].header.stamp = rostime;
      travMarkers.markers[i].ns = "map";
      travMarkers.markers[i].id = i;
      travMarkers.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      travMarkers.markers[i].scale.z = 0.1;
      travMarkers.markers[i].color.r = 1.0;
      travMarkers.markers[i].color.g = 1.0;
      travMarkers.markers[i].color.b = 1.0;
      travMarkers.markers[i].color.a = 1.0;

      if (travMarkers.markers[i].text != "")
        travMarkers.markers[i].action = visualization_msgs::Marker::ADD;
      else
        travMarkers.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_travMarkerPub.publish(travMarkers);
  }

  if (publishStairMarkerArray){
    for (unsigned i= 0; i < stairMarkers.markers.size(); ++i){
      double size = m_res;

      stairMarkers.markers[i].header.frame_id = m_worldFrameId;
      stairMarkers.markers[i].header.stamp = rostime;
      stairMarkers.markers[i].ns = "map";
      stairMarkers.markers[i].id = i;
      stairMarkers.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      stairMarkers.markers[i].scale.z = 0.1;
      stairMarkers.markers[i].color.r = 1.0;
      stairMarkers.markers[i].color.g = 1.0;
      stairMarkers.markers[i].color.b = 1.0;
      stairMarkers.markers[i].color.a = 1.0;

      if (stairMarkers.markers[i].text != "")
        stairMarkers.markers[i].action = visualization_msgs::Marker::ADD;
      else
        stairMarkers.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_stairMarkerPub.publish(stairMarkers);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Optional map publishing in MarbleMapping took %f sec", total_elapsed);
}

void MarbleMapping::publishOctoMaps(const ros::TimerEvent& event) {
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time rostime = ros::Time::now();
  if ((m_octree->size() <= 1) && (m_merged_tree->size() <= 1)) {
    return;
  }

  // Check if we have any new subscribers, so we force an update without waiting for a new diff
  bool newSub = false;
  if (m_input != 1) {
    int curSub = m_mergedBinaryMapPub.getNumSubscribers();
    newSub = curSub > m_lastSub;
    m_lastSub = curSub;
  }

  bool publishMergedBinaryMap = m_publishMergedBinaryMap && (m_mtree_updated || newSub) && (m_mergedBinaryMapPub.getNumSubscribers() > 0);
  bool publishMergedFullMap = m_publishMergedFullMap && m_mtree_updated && (m_mergedFullMapPub.getNumSubscribers() > 0);
  bool publishCameraMap = m_publishCameraMap && (m_cameraMapPub.getNumSubscribers() > 0);
  bool publishCameraView = m_publishCameraView && (m_cameraViewPub.getNumSubscribers() > 0);
  bool publishBinaryMap = m_octree_updated && (m_binaryMapPub.getNumSubscribers() > 0);
  bool publishFullMap = m_octree_updated && (m_fullMapPub.getNumSubscribers() > 0);

  // Prevent publishing repeat data
  m_octree_updated = false;
  m_mtree_updated = false;

  if (publishBinaryMap)
    publishBinaryOctoMap(rostime);

  if (publishFullMap)
    publishFullOctoMap(rostime);

  if (publishMergedBinaryMap)
    publishMergedBinaryOctoMap(rostime);

  if (publishMergedFullMap)
    publishMergedFullOctoMap(rostime);

  if (publishCameraMap)
    publishCameraOctoMap(rostime);

  if (publishCameraView)
    m_cameraViewPub.publish(m_cameraView);

  if (m_publishNeighborMaps)
    publishNeighborMaps(rostime);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in MarbleMapping took %f sec", total_elapsed);
}

bool MarbleMapping::octomapBinarySrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(*m_merged_tree, res.map))
    return false;

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool MarbleMapping::octomapFullSrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();


  if (!octomap_msgs::fullMapToMsg(*m_merged_tree, res.map))
    return false;

  return true;
}

bool MarbleMapping::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth +1);
  ros::Time rostime = ros::Time::now();
  m_octree->clear();
  m_merged_tree->clear();
  ROS_INFO("Cleared maps");

  publishBinaryOctoMap(rostime);
  publishMergedBinaryOctoMap(rostime);
  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  m_markerPub.publish(occupiedNodesVis);

  visualization_msgs::MarkerArray freeNodesVis;
  freeNodesVis.markers.resize(m_treeDepth +1);

  for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){

    freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
    freeNodesVis.markers[i].header.stamp = rostime;
    freeNodesVis.markers[i].ns = "map";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_fmarkerPub.publish(freeNodesVis);

  return true;
}

void MarbleMapping::publishBinaryOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::binaryMapToMsg(*m_octree, map))
    m_binaryMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishFullOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    m_fullMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishMergedBinaryOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::binaryMapToMsg(*m_merged_tree, map)) {
    m_mergedBinaryMapPub.publish(map);
  } else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishMergedFullOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::fullMapToMsg(*m_merged_tree, map)) {
    m_mergedFullMapPub.publish(map);
  } else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishCameraOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::binaryMapToMsg(*m_camera_tree, map)) {
    map.id = "OcTree";
    m_cameraMapPub.publish(map);
  } else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishNeighborMaps(const ros::Time& rostime) {
  boost::mutex::scoped_lock lock(m_mtx);
  for (auto& n : neighbor_maps) {
    // Find any maps that have been updated, and have subscribers
    if (n.second && neighbor_updated.at(n.first) && (neighbor_pubs.at(n.first).getNumSubscribers() > 0)) {
      neighbor_updated.at(n.first) = false;
      Octomap map;
      map.header.frame_id = m_worldFrameId;
      map.header.stamp = rostime;

      if (octomap_msgs::binaryMapToMsg(*n.second, map))
        neighbor_pubs.at(n.first).publish(map);
      else
        ROS_ERROR("Error serializing OctoMap");
    }
  }
}

void MarbleMapping::filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const{
  ground.header = pc.header;
  nonground.header = pc.header;

  if (pc.size() < 50){
    ROS_WARN("Pointcloud in MarbleMapping too small, skipping ground plane extraction");
    nonground = pc;
  } else {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PCLPoint> seg;
    seg.setOptimizeCoefficients (true);
    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold (m_groundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(m_groundFilterAngle);


    PCLPointCloud cloud_filtered(pc);
    // Create the filtering object
    pcl::ExtractIndices<PCLPoint> extract;
    bool groundPlaneFound = false;

    while(cloud_filtered.size() > 10 && !groundPlaneFound){
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_INFO("PCL segmentation did not find any plane.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative (false);
        extract.filter (ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        groundPlaneFound = true;
      } else{
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        pcl::PointCloud<PCLPoint> cloud_out;
        extract.setNegative (false);
        extract.filter(cloud_out);
        nonground +=cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        } else{
          cloud_filtered.points.clear();
        }
      }

    }
    // TODO: also do this if overall starting pointcloud too small?
    if (!groundPlaneFound){ // no plane found or remaining points too small
      ROS_WARN("No ground plane found in scan");

      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<PCLPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);

      second_pass.setFilterLimitsNegative (true);
      second_pass.filter(nonground);
    }

    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<PCLPoint>("nonground.pcd",pc_nonground, false);

  }


}

bool MarbleMapping::isSpeckleNode(const OcTreeKey&nKey) const {
  OcTreeKey key;
  bool neighborFound = false;
  boost::mutex::scoped_lock lock(m_mtx);
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          OcTreeNode* node = m_merged_tree->search(key);
          if (node && m_merged_tree->isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}

void MarbleMapping::reconfigureCallback(marble_mapping::MarbleMappingConfig& config, uint32_t level){
  if (m_maxTreeDepth != unsigned(config.max_depth))
    m_maxTreeDepth = unsigned(config.max_depth);
  else{
    m_pointcloudMinZ            = config.pointcloud_min_z;
    m_pointcloudMaxZ            = config.pointcloud_max_z;
    m_occupancyMinZ             = config.occupancy_min_z;
    m_occupancyMaxZ             = config.occupancy_max_z;
    m_filterSpeckles            = config.filter_speckles;
    m_filterGroundPlane         = config.filter_ground;
    m_compressMaps              = config.compress_maps;

    // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
    // will overwrite them because the server is not able to match parameters' names.
    if (m_initConfig){
      // If parameters do not have the default value, dynamic reconfigure server should be updated.
      if(!is_equal(m_groundFilterDistance, 0.04))
        config.ground_filter_distance = m_groundFilterDistance;
      if(!is_equal(m_groundFilterAngle, 0.15))
        config.ground_filter_angle = m_groundFilterAngle;
      if(!is_equal(m_groundFilterPlaneDistance, 0.07))
        config.ground_filter_plane_distance = m_groundFilterPlaneDistance;
      if(!is_equal(m_maxRange, -1.0))
        config.sensor_model_max_range = m_maxRange;
      if(!is_equal(m_octree->getProbHit(), 0.7))
        config.sensor_model_hit = m_octree->getProbHit();
      if(!is_equal(m_octree->getProbMiss(), 0.4))
        config.sensor_model_miss = m_octree->getProbMiss();
      if(!is_equal(m_octree->getClampingThresMin(), 0.12))
        config.sensor_model_min = m_octree->getClampingThresMin();
      if(!is_equal(m_octree->getClampingThresMax(), 0.97))
        config.sensor_model_max = m_octree->getClampingThresMax();
      m_initConfig = false;

      boost::recursive_mutex::scoped_lock reconf_lock(m_config_mutex);
      m_reconfigureServer.updateConfig(config);
    } else {
      m_groundFilterDistance      = config.ground_filter_distance;
      m_groundFilterAngle         = config.ground_filter_angle;
      m_groundFilterPlaneDistance = config.ground_filter_plane_distance;
      m_maxRange                  = config.sensor_model_max_range;
      m_octree->setClampingThresMin(config.sensor_model_min);
      m_octree->setClampingThresMax(config.sensor_model_max);
      m_merged_tree->setClampingThresMin(config.sensor_model_min);
      m_merged_tree->setClampingThresMax(config.sensor_model_max);
      m_displayColor = config.display_color;
      m_compressMaps = config.compress_maps;

      // Checking values that might create unexpected behaviors.
      if (is_equal(config.sensor_model_hit, 1.0))
        config.sensor_model_hit -= 1.0e-6;
      m_octree->setProbHit(config.sensor_model_hit);
      m_merged_tree->setProbHit(config.sensor_model_hit);
      if (is_equal(config.sensor_model_miss, 0.0))
        config.sensor_model_miss += 1.0e-6;
      m_octree->setProbMiss(config.sensor_model_miss);
      m_merged_tree->setProbMiss(config.sensor_model_miss);
    }
  }
  m_octree_updated = true;
  m_mtree_updated = true;
  m_mtree_markers_updated = true;
}

std_msgs::ColorRGBA MarbleMapping::heightMapColor(double h) {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}

void MarbleMapping::buildView(const tf::Matrix3x3& rotation, const octomap::point3d& position) {
  // Build the planes that make up the camera field of view

  // Vectors to define where the camera is looking
  Eigen::Vector3f view(rotation[0][0], rotation[1][0], rotation[2][0]);
  Eigen::Vector3f right(rotation[0][1], rotation[1][1], rotation[2][1]);
  Eigen::Vector3f up(rotation[0][2], rotation[1][2], rotation[2][2]);
  Eigen::Vector3f pos(position.x(), position.y(), position.z());

  // Points defining corners of the view
  Eigen::Vector3f fp_c(pos + view * camera_range);
  Eigen::Vector3f fp_tl(fp_c + (up * camera_h / 2) + (right * camera_w / 2));
  Eigen::Vector3f fp_tr(fp_c + (up * camera_h / 2) - (right * camera_w / 2));
  Eigen::Vector3f fp_bl(fp_c - (up * camera_h / 2) + (right * camera_w / 2));
  Eigen::Vector3f fp_br(fp_c - (up * camera_h / 2) - (right * camera_w / 2));

  // Build the planes
  pl_f.head<3>() = (fp_bl - fp_br).cross(fp_tr - fp_br);
  pl_f(3) = -fp_c.dot(pl_f.head<3>());

  Eigen::Vector3f a(fp_bl - pos);
  Eigen::Vector3f b(fp_br - pos);
  Eigen::Vector3f c(fp_tr - pos);
  Eigen::Vector3f d(fp_tl - pos);

  pl_r.head<3>() = b.cross(c);
  pl_l.head<3>() = d.cross(a);
  pl_t.head<3>() = c.cross(d);
  pl_b.head<3>() = a.cross(b);

  pl_r(3) = -pos.dot(pl_r.head<3>());
  pl_l(3) = -pos.dot(pl_l.head<3>());
  pl_t(3) = -pos.dot(pl_t.head<3>());
  pl_b(3) = -pos.dot(pl_b.head<3>());

  // Build the camera view polygon for visualization, if enabled
  if (m_publishCameraView) {
    geometry_msgs::Point camera;
    camera.x = pos(0);
    camera.y = pos(1);
    camera.z = pos(2);
    geometry_msgs::Point TL;
    TL.x = fp_tl(0);
    TL.y = fp_tl(1);
    TL.z = fp_tl(2);
    geometry_msgs::Point TR;
    TR.x = fp_tr(0);
    TR.y = fp_tr(1);
    TR.z = fp_tr(2);
    geometry_msgs::Point BL;
    BL.x = fp_bl(0);
    BL.y = fp_bl(1);
    BL.z = fp_bl(2);
    geometry_msgs::Point BR;
    BR.x = fp_br(0);
    BR.y = fp_br(1);
    BR.z = fp_br(2);

    m_cameraView.points.clear();
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(TL);
    m_cameraView.points.push_back(BL);
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(TL);
    m_cameraView.points.push_back(TR);
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(TR);
    m_cameraView.points.push_back(BR);
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(BR);
    m_cameraView.points.push_back(BL);
  }
}

bool MarbleMapping::pointInsideView(octomap::point3d& point) {
  // Find whether a given point lies inside the field of view of the camera
  Eigen::Vector4f pt(point.x(), point.y(), point.z(), 1.0f);
  return (pt.dot(pl_l) <= 0) &&
         (pt.dot(pl_r) <= 0) &&
         (pt.dot(pl_t) <= 0) &&
         (pt.dot(pl_b) <= 0) &&
         (pt.dot(pl_f) <= 0);
}

bool MarbleMapping::isCeiling(const octomap::OcTreeKey& curKey) {
  // Determine whether a node is part of the ceiling and should be removed
  bool skipNode = false;
  for (int i = 1; i <= m_removeCeilingDepth; i++) {
    // Look at i nodes above the current node to see if does NOT exist
    OcTreeKey checkKey = curKey;
    checkKey.k[2] = curKey.k[2] + i;
    RoughOcTreeNode* aboveNode = m_merged_tree->search(checkKey);
    if (!aboveNode) {
      for (int j = 1; j <= m_removeCeilingDepth; j++) {
        // If not, look at j nodes below the current node to see if it's free
        checkKey.k[2] = curKey.k[2] - j;
        RoughOcTreeNode* belowNode = m_merged_tree->search(checkKey);
        if (belowNode && belowNode->getLogOdds() < 0) {
          // Found a free node below, now check if it's surrounded by free nodes as well
          checkKey.k[0] = curKey.k[0] + 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          checkKey.k[0] = curKey.k[0] - 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          checkKey.k[1] = curKey.k[1] + 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          checkKey.k[1] = curKey.k[1] - 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          // Made it this far, then all the nodes below are free, so mark and break the loop
          skipNode = true;
          break;
        }
      }

      // If we found a node below surrounded by free nodes, we can stop searching
      if (skipNode)
        break;
    }
  }

  return skipNode;
}
}