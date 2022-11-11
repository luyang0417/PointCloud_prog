#pragma once
#include "common.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

void extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

void extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, Eigen::Matrix4f& transformation);;

void normal_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr viewport);

void normal_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr viewport, int level);

void ransac_segment_cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, double distance_threshold, double radius_limit_low, double radius_limit_high, pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers);

void extract_inliers_by_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers_cloud, pcl::PointIndices::Ptr& inliers, bool setNegative);

void extract_normals_by_indices(pcl::PointCloud<pcl::Normal>::Ptr& normals, pcl::PointCloud<pcl::Normal>::Ptr& inliers_cloud, pcl::PointIndices::Ptr& inliers, bool setNegative);

inline double compute_sqrt(double x1, double y1, double z1, double x2, double y2, double z2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

double compute_edge_intensity(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int index, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree, double radius);

pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree, double radius, double threshold);

pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double radius, double ratio);

void viz_cylinder_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float x, float y, float z, float rx, float ry, float rz);

void viz_cylinder_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients::Ptr& coefficients);

pcl::PointCloud<pcl::Normal>::Ptr compute_normal_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree, int k_search);

Eigen::Matrix4f point_cloud_adjust(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float lower_radius_b, float upper_radius_b, float lower_radius_m, float upper_radius_m, float thickness_threshold, float& radius_b, float& radius_m);

