#include "common.h"
#include "registration.h"
#include "extration.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

void zoom_out_point_cloud(PointCloudT::Ptr& cloud, float zoom_scale) {
	PointCloudT::Ptr cloud_small(new PointCloudT);
	for (PointT point : *cloud) {
		cloud_small->push_back(*new PointT(point.x /= zoom_scale, point.y /= zoom_scale, point.z /= zoom_scale));
	}
	cloud = cloud_small;
}

int load_ply_cloud(string filename, PointCloudT::Ptr cloud) {
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
	while (-1 == pcl::io::loadPLYFile(filename, *cloud2)) {
		pcl::console::print_highlight("Error input.\n");
		return 0;
	}
	pcl::fromPCLPointCloud2(*cloud2, *cloud);
	pcl::console::print_highlight("point cloud load successfully\n");
	return 1;
}

/*
 *  \param[in] cloud the input cloud
 *			   K the mean k
 */
PointCloudT::Ptr statistic_outliers_removal(PointCloudT::Ptr& cloud, int K) {
	pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	pcl::Indices indices;
	sor->setMeanK(K);
	sor->setInputCloud(cloud);
	sor->filter(*cloud_out);
	return cloud_out;
}

PointCloudT::Ptr radius_outliers_removal(PointCloudT::Ptr& cloud, double search_radius) {
	pcl::RadiusOutlierRemoval<PointT>::Ptr ror(new pcl::RadiusOutlierRemoval<PointT>);
	ror->setInputCloud(cloud);
	ror->setRadiusSearch(search_radius);
	ror->setMinNeighborsInRadius(100);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	ror->filter(*cloud_out);
	return cloud_out;
}

int main(int argc, char** argv) {
	string filepath("D:/PointCloud/fused_partial_cloud.pcd");
	PointCloudT::Ptr cloud(new PointCloudT);
	load_pcd_cloud(filepath, cloud);
	// load_ply_cloud(filepath, cloud);
	pointcloud_Move(cloud, 100, 100, 100);
	pointcloud_Rotate(cloud, 30, 30, 30);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	PointCloudT::Ptr cloud_vox(new PointCloudT);
	voxel_downsizing(cloud, cloud_vox, { 3,3,3 });
	single_cloud_visualization(cloud_vox, 100);

	PointCloudT::Ptr cloud_seam(new PointCloudT);
	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
	extraction(cloud, cloud_seam, transform_matrix);
	pcl::console::print_highlight("extracted seam cloud has ");
	cout << cloud_seam->size() << " points." << endl;
	system("pause");
	// zoom_out_point_cloud(cloud, 100000);
	single_cloud_visualization(cloud_seam, 100);

	PointCloudT::Ptr cloud_sor = statistic_outliers_removal(cloud_seam, 20);
	pcl::console::print_highlight("statistic outliers removal...\n");
	single_cloud_visualization(cloud_sor, 100);
	PointCloudT::Ptr cloud_ror = radius_outliers_removal(cloud_seam, 10);
	pcl::console::print_highlight("radius outliers removal...\n");
	single_cloud_visualization(cloud_ror, 100);
	/*
	PointCloudT::Ptr cloud_roi(new PointCloudT);
	roi_extraction(cloud_ror, cloud_roi, filepath.substr(0, filepath.length() - 4) + "_roi.pcd");
	single_cloud_visualization(cloud_roi, 100);
	*/
	pcl::console::print_highlight("start extracting edges...\n");
	single_cloud_visualization(cloud_seam, 100);
	PointCloudT::Ptr cloud_edge(edge_points_detection(cloud_seam, 10, 0.75));
	single_cloud_visualization(cloud_edge, 100);
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	PointCloudT::Ptr cloud_transform(new PointCloudT);
	pcl::transformPointCloud(*cloud, *cloud_transform, transform_matrix);
	viewer->addPointCloud(cloud_transform, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_seam, 255, 0, 0);
	viewer->addPointCloud(cloud_seam, red, "cloud_seam");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud_edge, 0, 255, 0);
	viewer->addPointCloud(cloud_edge, green, "cloud_edge");
	while (!viewer->wasStopped()) {
		viewer->spinOnce(1000);
	}
	viewer->close();

}