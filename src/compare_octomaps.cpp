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
              "Visualize using 'compare_octomaps' topic.\n"

using namespace octomap;
int main(int argc, char** argv){
  ros::init(argc, argv, "compare_octomaps");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  OcTree* tree1;
  OcTree* tree2;
  OcTree* ctree;

  std::string mapTopic1(""), mapTopic2("");
  std::string filename1(""), filename2("");

  if (argc > 4 || (argc == 2 && std::string(argv[1]) == "-h")) {
    std::cout << USAGE << std::endl;
    exit(-1);
  }

  // Load the trees
  if (argc == 3) {
    // Reading from topics
    mapTopic1 = std::string(argv[1]);
    octomap_msgs::OctomapConstPtr mapShared1 = ros::topic::waitForMessage<octomap_msgs::Octomap>(mapTopic1);
    octomap_msgs::Octomap map1 = *mapShared1;
    tree1 = (OcTree*)octomap_msgs::binaryMsgToMap(map1);

    mapTopic2 = std::string(argv[2]);
    octomap_msgs::OctomapConstPtr mapShared2 = ros::topic::waitForMessage<octomap_msgs::Octomap>(mapTopic2);
    octomap_msgs::Octomap map2 = *mapShared2;
    tree2 = (OcTree*)octomap_msgs::binaryMsgToMap(map2);
  } else if (argc == 4 && std::string(argv[1]) == "-f") {
    // Reading from files
    filename1 = std::string(argv[2]);
    if (filename1.substr(filename1.length() - 2) == "ot")
      tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
    else
      tree1 = new OcTree(filename1);

    filename2 = std::string(argv[3]);
    if (filename2.substr(filename2.length() - 2) == "ot")
      tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));
    else
      tree2 = new OcTree(filename2);
  }

  if (!tree1 || !tree2) return 0;

  // Create a differences tree to visualize
  ros::Publisher cPub = nh.advertise<octomap_msgs::Octomap>("compare_octomaps", 1, true);
  ctree = new OcTree(tree1->getResolution());
  tree1->expand();
  tree2->expand();

  int in1_occ = 0;
  int in1_free = 0;
  int in2_occ = 0;
  int in2_free = 0;
  int in_both_occ = 0;
  int in_both_free = 0;

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
      ctree->updateNode(it.getCoordinate(), occupied);
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
      ctree->updateNode(it.getCoordinate(), occupied);
    }
  }

  // Publish our differences map
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(*ctree, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  cPub.publish(msg);

  // Output statistics
  int in1 = in1_occ + in1_free;
  int in2 = in2_occ + in2_free;
  int in_both = in_both_occ + in_both_free;
  float occ_p, free_p, total_p;
  int smaller_tree;
  std::cout.imbue(std::locale(""));
  std::cout << "Node counts: (occupied / free / total)" << std::endl;
  std::cout << "Nodes in tree 1: " << in1_occ << " / " << in1_free << " / " << in1 << std::endl;
  std::cout << "Nodes in tree 2: " << in2_occ << " / " << in2_free << " / " << in2 << std::endl;
  std::cout << "Nodes in both trees: " << in_both_occ << " / " << in_both_free << " / " << in_both << std::endl;
  if (in1 > in2) {
    smaller_tree = 2;
    occ_p = (float)(in1_occ - in2_occ) / in1_occ * 100.0;
    free_p = (float)(in1_free - in2_free) / in1_free * 100.0;
    total_p = (float)(in1 - in2) / in1 * 100.0;
  } else {
    smaller_tree = 1;
    occ_p = (float)(in2_occ - in1_occ) / in2_occ * 100.0;
    free_p = (float)(in2_free - in1_free) / in2_free * 100.0;
    total_p = (float)(in2 - in1) / in2 * 100.0;
  }
  std::cout << "Nodes missing from tree " << smaller_tree << ": " << abs(in2_occ - in1_occ) << " / " << abs(in2_free - in1_free) << " / " << abs(in2 - in1) << std::endl;
  std::cout << "Percentage missing: " << occ_p << "% / " << free_p << "% / " << total_p << "%" << std::endl;

  return 0;
}
