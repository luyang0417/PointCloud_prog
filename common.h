#pragma once
#ifndef COMMON_H
#define COMMON_H

#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointN> PointCloudN;
using namespace std;

int load_pcd_cloud(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void pointcloud_Move(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double dx, double dy, double dz);

void pointcloud_Rotate(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double angle_x, double angle_y, double angle_z);

Eigen::Matrix4f pointcloud_Rotate(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double angle_x, double angle_y, double angle_z, bool ret_matrix);

void voxel_downsizing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<float> leaf_size);

void voxel_downsizing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<float> leaf_size);

void voxel_downsizing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

void remove_zero(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void single_cloud_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void single_cloud_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float coordinate_size);

double compute_point_cloud_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

double compute_point_cloud_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float k_nn);

#endif // !COMMON_H