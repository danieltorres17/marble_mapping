/**
* compare_octomaps: A tool to compare similar octomaps for differences 
* @author D. Riley, Copyright (C) 2020.
* License: BSD
*/

/*
 * Copyright 2020 Dan Riley
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


#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <locale>

#define USAGE "\nUSAGE: compare_octomaps mapTopic1 mapTopic2\n" \
              "OR:    compare_octomaps -f <map1.[bt|ot]> <map2.[bt|ot]>\n" \
              "Compares two OctoMaps to show size difference.\n" \
              "Visualize using 'cmaps_missing', 'cmaps_not_in_1' and 'cmaps_not_in_2' topics.\n"

using namespace octomap;
int main(int argc, char** argv){
  ros::init(argc, argv, "compare_octomaps");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  OcTree* tree1;
  OcTree* tree2;
  OcTree* ctree;
  OcTree* ctree1;
  OcTree* ctree2;

  std::string argmap1(""), argmap2("");

  std::string args = std::string(argv[1]);
  if (argc > 4 || (argc == 2 && args == "-h")) {
    std::cout << USAGE << std::endl;
    exit(-1);
  }

  bool useTopics = true;
  if (argc == 4) {
    if (args == "-f") {
      useTopics = false;
      argmap1 = std::string(argv[2]);
      argmap2 = std::string(argv[3]);
    }
  } else {
    argmap1 = std::string(argv[1]);
    argmap2 = std::string(argv[2]);
  }

  // Load the trees
  if (useTopics) {
    // Reading from topics
    octomap_msgs::OctomapConstPtr mapShared1 = ros::topic::waitForMessage<octomap_msgs::Octomap>(argmap1);
    octomap_msgs::Octomap map1 = *mapShared1;
    tree1 = (OcTree*)octomap_msgs::binaryMsgToMap(map1);

    octomap_msgs::OctomapConstPtr mapShared2 = ros::topic::waitForMessage<octomap_msgs::Octomap>(argmap2);
    octomap_msgs::Octomap map2 = *mapShared2;
    tree2 = (OcTree*)octomap_msgs::binaryMsgToMap(map2);
  } else {
    // Reading from files
    if (argmap1.substr(argmap1.length() - 2) == "ot")
      tree1 = dynamic_cast<OcTree*>(OcTree::read(argmap1));
    else
      tree1 = new OcTree(argmap1);

    if (argmap2.substr(argmap2.length() - 2) == "ot")
      tree2 = dynamic_cast<OcTree*>(OcTree::read(argmap2));
    else
      tree2 = new OcTree(argmap2);
  }

  if (!tree1 || !tree2) return 0;

  // Create a differences tree to visualize
  ros::Publisher cPub = nh.advertise<octomap_msgs::Octomap>("cmaps_missing", 1, true);
  ros::Publisher cPub1 = nh.advertise<octomap_msgs::Octomap>("cmaps_not_in_2", 1, true);
  ros::Publisher cPub2 = nh.advertise<octomap_msgs::Octomap>("cmaps_not_in_1", 1, true);
  // ctree = map showing all nodes that are different in both maps
  // ctree1 = map showing what's in tree1 that's not in tree2
  // ctree2 = map showing what's in tree2 that's not in tree1
  ctree = new OcTree(tree1->getResolution());
  ctree1 = new OcTree(tree1->getResolution());
  ctree2 = new OcTree(tree1->getResolution());
  tree1->expand();
  tree2->expand();

  int in1_occ = 0;
  int in1_free = 0;
  int in2_occ = 0;
  int in2_free = 0;
  int in_both_occ = 0;
  int in_both_free = 0;
  int not_in1_occ = 0;
  int not_in2_occ = 0;
  int not_in1_free = 0;
  int not_in2_free = 0;

  // Check the first tree and see which nodes are in both.  The total is typically what matters,
  // as that represents what has been seen by either sensor.
  for (OcTree::leaf_iterator it = tree1->begin_leafs(), end = tree1->end_leafs(); it != end; ++it) {
    bool occupied;
    if (it->getOccupancy() > 0.5) {
      in1_occ++;
      occupied = true;
    } else {
      in1_free++;
      occupied = false;
    }

    // Check the other tree to see if it's in both trees
    OcTreeNode* node2 = tree2->search(it.getCoordinate());
    if (node2) {
      if (node2->getOccupancy() > 0.5) in_both_occ++;
      else in_both_free++;
    } else {
      // Count how many nodes are missing from tree2
      if (occupied) not_in2_occ++;
      else not_in2_free++;
      ctree->updateNode(it.getCoordinate(), occupied);
      ctree1->updateNode(it.getCoordinate(), occupied);
    }
  }

  for (OcTree::leaf_iterator it = tree2->begin_leafs(), end = tree2->end_leafs(); it != end; ++it) {
    bool occupied;
    if (it->getOccupancy() > 0.5) {
      in2_occ++;
      occupied = true;
    } else {
      in2_free++;
      occupied = false;
    }

    // We already counted this node if it's in the other tree
    // If everything the sensor that created this map is enclosed by the other map,
    // this won't add anything new
    OcTreeNode* node2 = tree1->search(it.getCoordinate());
    if (!node2) {
      // Count how many nodes are missing from tree1
      if (occupied) not_in1_occ++;
      else not_in1_free++;
      ctree->updateNode(it.getCoordinate(), occupied);
      ctree2->updateNode(it.getCoordinate(), occupied);
    }
  }

  // Publish our differences map
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(*ctree, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  cPub.publish(msg);

  octomap_msgs::binaryMapToMsg(*ctree1, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  cPub1.publish(msg);

  octomap_msgs::binaryMapToMsg(*ctree2, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  cPub2.publish(msg);

  // Output statistics
  int in1 = in1_occ + in1_free;
  int in2 = in2_occ + in2_free;
  int in_both = in_both_occ + in_both_free;
  int not_in1 = not_in1_occ + not_in1_free;
  int not_in2 = not_in2_occ + not_in2_free;
  float occ_p, free_p, total_p;
  int smaller_tree;
  std::cout.imbue(std::locale(""));
  std::cout << "Tree 1: " << argmap1 << std::endl;
  std::cout << "Tree 2: " << argmap2 << std::endl << std::endl;

  std::cout << "Node counts: (occupied / free / total)" << std::endl;
  std::cout << "Nodes in tree 1: " << in1_occ << " / " << in1_free << " / " << in1 << std::endl;
  std::cout << "Nodes in tree 2: " << in2_occ << " / " << in2_free << " / " << in2 << std::endl;
  std::cout << "Nodes in both trees: " << in_both_occ << " / " << in_both_free << " / " << in_both << std::endl;

  occ_p = (float)not_in1_occ / (float)in2_occ * 100.0;
  free_p = (float)not_in1_free / (float)in2_free * 100.0;
  total_p = (float)not_in1 / (float)in2 * 100.0;

  std::cout << std::endl;
  std::cout << "Nodes missing from tree 1: " << not_in1_occ << " / " << not_in1_free << " / " << not_in1 << std::endl;
  std::cout << "Percentage missing: " << occ_p << "% / " << free_p << "% / " << total_p << "%" << std::endl;

  occ_p = (float)not_in2_occ / (float)in1_occ * 100.0;
  free_p = (float)not_in2_free / (float)in1_free * 100.0;
  total_p = (float)not_in2 / (float)in1 * 100.0;

  std::cout << std::endl;
  std::cout << "Nodes missing from tree 2: " << not_in2_occ << " / " << not_in2_free << " / " << not_in2 << std::endl;
  std::cout << "Percentage missing: " << occ_p << "% / " << free_p << "% / " << total_p << "%" << std::endl;

  std::cout << std::endl;
  std::cout << "Published the following Octomaps for visualization:" << std::endl;
  std::cout << "/cmaps_missing : All nodes that are not in both maps" << std::endl;
  std::cout << "/cmaps_not_in_1 : Nodes in " << argmap2 << " but not in " << argmap1 << std::endl;
  std::cout << "/cmaps_not_in_2 : Nodes in " << argmap1 << " but not in " << argmap2 << std::endl;
  std::cout << std::endl << "Press ctrl-c to stop publishing." << std::endl;

  try {
    ros::spin();
  } catch(std::runtime_error& e) {
    ROS_ERROR("compare_octomaps exception: %s", e.what());
    return -1;
  }

  return 0;
}
