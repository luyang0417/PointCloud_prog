#pragma once
#include "common.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/recognition/trimmed_icp.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/icp.h>

// structs 
struct callback_args 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr orig_points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
	pcl::PointCloud<pcl::PointXYZ>::Ptr chosen_area_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
	std::string filepath;
};

struct point_params
{
	int index;
	float distance;
};

// prototypes
void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
/*
void remove_zero(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void single_cloud_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void voxel_downsizing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<float> leaf_size);
*/
void voxel_downsizing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<float> leaf_size);

void voxel_downsizing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

void ap_callback(const pcl::visualization::AreaPickingEvent& event, void* args);

void roi_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, std::string filename);

Eigen::Matrix4f coarse_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target);

inline bool greater_sort(point_params a, point_params b) { return (a.distance > b.distance); }

inline bool less_sort(point_params a, point_params b) { return (a.distance < b.distance); }

double registration_mark(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target, float overlap_ratio);

Eigen::Matrix4f icp_trans_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target);

Eigen::Matrix4d tricp_trans_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float overlap_ratio);