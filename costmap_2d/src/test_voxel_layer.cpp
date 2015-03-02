#include <ros/ros.h>
#include <costmap_2d/voxel_layer.h>
#include <costmap_2d/obstacle_layer.h>
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

//Testing clearNonLethal

void testClearNonLethal() {
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  
  //Add the static map
  addStaticLayer(layers, tf);
  ObstacleLayer *olayer = addObstacleLayer(layers, tf);
  
  //Add a point at 0, 0, 0
  addObservation(olayer, 0.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);
  layers.updateMap(0,0,0);
  
  printMap(*layers.getCostmap());

  int lethal_count = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  //We expect just one obstacle to be added (20 already in static map)
  if(lethal_count != 21){
    ROS_ERROR("testClearNonLethal failed!");
  }
  else{
    ROS_INFO("testClearNonLethal passed!");
  }
}

int testResetOldCosts(){
  tf::TransformListener tf;

  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayer(layers, tf);  // This adds the static map

  // Add a voxel layer
  VoxelLayer* vlayer = new VoxelLayer();
  vlayer->initialize(&layers, "voxels", &tf);
  boost::shared_ptr<costmap_2d::Layer> vptr(vlayer);
  layers.addPlugin(vptr);
  /*
  // Add a point at 0, 0, 0
  addObservation(vlayer, 0.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);

  // This actually puts the LETHAL (254) point in the costmap at (0,0)
  layers.updateMap(0,0,0);  // 0, 0, 0 is robot pose
  printMap(*(layers.getCostmap()));
  
  int lethal_count = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // We expect just one obstacle to be added (20 in static map)
  if(lethal_count != 21){
    ROS_ERROR("testResetOldCosts failed!");
    printf("testResetOldCosts failed!\n");
  }
  else{
    ROS_INFO("testResetOldCosts passed!");
    printf("testResetOldCosts passed!\n");
  }
  */
  printf("end of testResetOldCosts!\n");
  ROS_INFO("end of testResetOldCosts!");

  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxel_tests");
  testClearNonLethal();  
  int result = testResetOldCosts();
  printf("result was %d", result);
  ROS_INFO("result was %d", result);
  return 0;
}
