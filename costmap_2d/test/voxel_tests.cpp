/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Mayfield Robotics
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Mayfield Robotics nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Kaijen Hsiao, Hai Nguyen, Sarah Osentoski
*********************************************************************/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <costmap_2d/voxel_layer.h>
#include <costmap_2d/testing_helper.h>
#include <tf/transform_listener.h>

using namespace costmap_2d;

/*
 * For reference, the static map looks like this:
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0 254 254 254   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   upper left is 0,0, lower right is 9,9
 */

/**
 * Test for ray tracing free space
 */
TEST(costmap, testRaytracing){
  tf::TransformListener tf;

  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayer(layers, tf);  // This adds the static map
  VoxelLayer* olayer = addVoxelLayer(layers, tf);
  
  // Add a point at 0, 0, 0
  addObservation(olayer, 0.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);

  // This actually puts the LETHAL (254) point in the costmap at (0,0)
  layers.updateMap(0,0,0);  // 0, 0, 0 is robot pose
  //printMap(*(layers.getCostmap()));
  
  int lethal_count = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // We expect just one obstacle to be added (20 in static map)
  ASSERT_EQ(lethal_count, 21);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxel_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
