/*
 * PclToGridMapNode.cpp
 *
 *  Created on: July 23rd, 2019
 *      Author: Martin Freeman
 *  Jet Propulsion Laboratory
 */

#include <pcl_to_grid_map/PclToGridMapNode.hpp>

#include <pcl_to_grid_map/PclToGridMap.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

PclToGridMapNode::PclToGridMapNode(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(grid_map::GridMap({"elevation"}))
{
  //readParameters();
  map_.setBasicLayers({"elevation"});
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  pclSubscriber_ = nodeHandle_.subscribe("/voxblox_node/surface_pointcloud", 1, &PclToGridMapNode::surface_pointcloud_callback, this);

}

PclToGridMapNode::~PclToGridMapNode()
{
}

// bool PclToGridMapNode::readParameters()
// {
//   nodeHandle_.param("min_x", minX_, -1.0);
//   nodeHandle_.param("max_x", maxX_, 1.0);
//   nodeHandle_.param("min_y", minY_, -1.0);
//   nodeHandle_.param("max_y", maxY_, 1.0);
//   nodeHandle_.param("min_z", minZ_, -1.0);
//   nodeHandle_.param("max_z", maxZ_, 1.0);
//   return true;
// }




void PclToGridMapNode::surface_pointcloud_callback(
     const sensor_msgs::PointCloud2::Ptr pc_ptr) {
  std::lock_guard<std::mutex> lock(_map_update_mutex);
  ROS_INFO("Receieved pointcloud");
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pc_ptr, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *surface_cloud);
  _surface_pointcloud_ptr = surface_cloud;

    ROS_INFO("Starting Conversion");

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;
    
    float minX_ = 0;
    float maxX_ = 0;
    float minY_ = 0;
    float maxY_ = 0;
    float minZ_ = 0;
    float maxZ_ = 0;
    
    min_bound(0) = minX_;
    max_bound(0) = maxX_;
    min_bound(1) = minY_;
    max_bound(1) = maxY_;
    min_bound(2) = minZ_;
    max_bound(2) = maxZ_;

  bool res = PclToGridMapNode::toGridMap(*_surface_pointcloud_ptr, "elevation", map_, &min_bound, &max_bound);
  if (!res) {
    ROS_ERROR("Failed to call convert PCL.");
    return;
  }

  grid_map_msgs::GridMap gridMapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
  gridMapPublisher_.publish(gridMapMessage);
    return;
}


bool PclToGridMapNode::toGridMap(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                                          const std::string& layer,
                                          grid_map::GridMap& gridMap,
                                          const grid_map::Position3* minPoint,
                                          const grid_map::Position3* maxPoint)
{
  ROS_INFO("Made it to the conversion call");
  double pcl_size = pointcloud.size();
  double pcl_width = pointcloud.width;
  double pcl_height = pointcloud.height;

  ROS_INFO("PCL SIZE %f", pcl_size);
  ROS_INFO("PCL WIDTH %f", pcl_width);
  ROS_INFO("PCL HEIGHT %f", pcl_height);

  double resolution = 0.025;
  grid_map::Length length(10,10);
  grid_map::Position position;

  gridMap.setGeometry(length, resolution, position);
  gridMap.add(layer);
  gridMap.setBasicLayers({layer});


  grid_map::Matrix& gridMapData = gridMap[layer];

  for (int i = 0; i < pcl_size; i++)
  {
  	grid_map::Position position(pointcloud.points[i].x, pointcloud.points[i].y);
  	grid_map::Index index;
  	gridMap.getIndex(position, index);

  	if (!gridMap.isValid(index))
  	{
  		gridMapData(index(0),index(1)) = pointcloud.points[i].z;
  	}
  	else
  	{
  		if (gridMapData(index(0),index(1)) < pointcloud.points[i].z)
  		{
  			gridMapData(index(0),index(1)) = pointcloud.points[i].z;
  		}
  	}
  }

  return true;
}

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "pcl_to_gridmap_node");
  ros::NodeHandle nh("~");
  PclToGridMapNode PclToGridMapNode(nh);
  ros::Duration(2.0).sleep();
  ros::Rate r(0.1); // 1 hz
  r.sleep();
  ros::spin();
  return 0;
}
