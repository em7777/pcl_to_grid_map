/*
 * OctomapToGridmapDemo.hpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#pragma once

// ROS
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <string>
#include  <mutex>
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>

/*!
 * Receives a volumetric OctoMap and converts it to a grid map with an elevation layer.
 * The grid map is published and can be viewed in Rviz.
 */
class PclToGridMapNode
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PclToGridMapNode(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~PclToGridMapNode();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();
  void surface_pointcloud_callback(const sensor_msgs::PointCloud2::Ptr pc_ptr);

  static bool toGridMap(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                          const std::string& layer,
                          grid_map::GridMap& gridMap,
                          const grid_map::Position3* minPoint = nullptr,
                          const grid_map::Position3* maxPoint = nullptr);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Octomap publisher.
  ros::Subscriber pclSubscriber_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Name of the grid map topic.

  //! Octomap service client

  //! Bounding box of octomap to convert.
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;

  std::mutex _map_update_mutex;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _surface_pointcloud_ptr;
};
