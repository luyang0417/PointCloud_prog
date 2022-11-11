#include "common.h"

// 加载点云文件
int load_pcd_cloud(string filename, PointCloudT::Ptr cloud)
{
	if (-1 == pcl::io::loadPCDFile(filename, *cloud)) {
		return 0;
	}
	pcl::console::print_highlight("point cloud loaded successfully\n");
	return 1;
}
// 点云平移
void pointcloud_Move(PointCloudT::Ptr& cloud, double dx, double dy, double dz)
{
	PointCloudT::Ptr cloud_mv(new PointCloudT);
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(0, 3) = dx;
	transformation_matrix(1, 3) = dy;
	transformation_matrix(2, 3) = dz;
	pcl::transformPointCloud(*cloud, *cloud_mv, transformation_matrix);
	cloud = cloud_mv;
}
// 点云旋转
// 逆轴向看，点云顺时针转动
void pointcloud_Rotate(PointCloudT::Ptr& cloud, double angle_x, double angle_y, double angle_z)
{
	PointCloudT::Ptr cloud_rotate_x(new PointCloudT);
	PointCloudT::Ptr cloud_rotate_y(new PointCloudT);
	PointCloudT::Ptr cloud_rotate_z(new PointCloudT);
	Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();
	angle_x *= M_PI / 180;
	angle_y *= M_PI / 180;
	angle_z *= M_PI / 180;
	rotation_x(1, 1) = cos(angle_x);
	rotation_x(1, 2) = sin(angle_x);
	rotation_x(2, 1) = -sin(angle_x);
	rotation_x(2, 2) = cos(angle_x);
	pcl::transformPointCloud(*cloud, *cloud_rotate_x, rotation_x);

	rotation_y(0, 0) = cos(angle_y);
	rotation_y(0, 2) = -sin(angle_y);
	rotation_y(2, 0) = sin(angle_y);
	rotation_y(2, 2) = cos(angle_y);
	pcl::transformPointCloud(*cloud_rotate_x, *cloud_rotate_y, rotation_y);

	rotation_z(0, 0) = cos(angle_z);
	rotation_z(0, 1) = sin(angle_z);
	rotation_z(1, 0) = -sin(angle_z);
	rotation_z(1, 1) = cos(angle_z);
	pcl::transformPointCloud(*cloud_rotate_y, *cloud_rotate_z, rotation_z);

	cloud = cloud_rotate_z;
}

Eigen::Matrix4f pointcloud_Rotate(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double angle_x, double angle_y, double angle_z, bool ret_matrix)
{
	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
	shared_ptr<Eigen::Matrix4f> res = make_shared<Eigen::Matrix4f>(transform_matrix);
	if (ret_matrix) {
		PointCloudT::Ptr cloud_rotate_x(new PointCloudT);
		PointCloudT::Ptr cloud_rotate_y(new PointCloudT);
		PointCloudT::Ptr cloud_rotate_z(new PointCloudT);
		Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();
		angle_x *= M_PI / 180;
		angle_y *= M_PI / 180;
		angle_z *= M_PI / 180;
		rotation_x(1, 1) = cos(angle_x);
		rotation_x(1, 2) = sin(angle_x);
		rotation_x(2, 1) = -sin(angle_x);
		rotation_x(2, 2) = cos(angle_x);
		pcl::transformPointCloud(*cloud, *cloud_rotate_x, rotation_x);

		rotation_y(0, 0) = cos(angle_y);
		rotation_y(0, 2) = -sin(angle_y);
		rotation_y(2, 0) = sin(angle_y);
		rotation_y(2, 2) = cos(angle_y);
		pcl::transformPointCloud(*cloud_rotate_x, *cloud_rotate_y, rotation_y);

		rotation_z(0, 0) = cos(angle_z);
		rotation_z(0, 1) = sin(angle_z);
		rotation_z(1, 0) = -sin(angle_z);
		rotation_z(1, 1) = cos(angle_z);
		pcl::transformPointCloud(*cloud_rotate_y, *cloud_rotate_z, rotation_z);

		cloud = cloud_rotate_z;
		return rotation_z * rotation_y * rotation_x;

	}
	else {
		pointcloud_Rotate(cloud, angle_x, angle_y, angle_z);
	}
	return transform_matrix;
}

void voxel_downsizing(PointCloudT::Ptr cloud, vector<float> leaf_size)
{
	pcl::console::print_highlight("start downsizing...\n");
	clock_t start = clock();
	PointCloudT::Ptr cloud_orig(cloud);
	pcl::VoxelGrid<PointT>::Ptr grid(new pcl::VoxelGrid<PointT>);
	if (leaf_size.size() != 3) {
		pcl::console::print_highlight("wrong params! 3 leaf size params are needed. \n");
		return;
	}
	grid->setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
	grid->setInputCloud(cloud_orig);
	grid->filter(*cloud);
	pcl::console::print_highlight("point cloud size after downsizing: ");
	cout << cloud->size() << endl;
	clock_t end = clock();
	double time = (double)(end - start) / (double)CLOCKS_PER_SEC;
	pcl::console::print_highlight("time used for voxel grid filtering: ");
	cout << time << "s. " << endl;
}


void voxel_downsizing(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out, vector<float> leaf_size) {
	pcl::console::print_highlight("start downsizing...\n");
	clock_t start = clock();
	pcl::VoxelGrid<PointT>::Ptr grid(new pcl::VoxelGrid<PointT>);
	if (leaf_size.size() != 3) {
		pcl::console::print_highlight("wrong params! 3 leaf size params are needed. \n");
		return;
	}
	grid->setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
	grid->setInputCloud(cloud_in);
	grid->filter(*cloud_out);
	pcl::console::print_highlight("point cloud size after downsizing: ");
	cout << cloud_out->size() << endl;
	clock_t end = clock();
	double time = (double)(end - start) / (double)CLOCKS_PER_SEC;
	pcl::console::print_highlight("time used for voxel grid filtering: ");
	cout << time << "s. " << endl;
}

void voxel_downsizing(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out) {
	pcl::console::print_highlight("start downsizing...\n");
	vector<float> leaf_size;
	pcl::console::print_highlight("enter voxel size: x = ");
	float temp(0);
	cin >> temp;
	leaf_size.push_back(temp);
	pcl::console::print_highlight(" ,y = ");
	cin >> temp;
	leaf_size.push_back(temp);
	pcl::console::print_highlight(" ,z = ");
	cin >> temp;
	leaf_size.push_back(temp);
	clock_t start = clock();
	pcl::VoxelGrid<PointT>::Ptr grid(new pcl::VoxelGrid<PointT>);
	if (leaf_size.size() != 3) {
		pcl::console::print_highlight("wrong params! 3 leaf size params are needed. \n");
		return;
	}
	grid->setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
	grid->setInputCloud(cloud_in);
	grid->filter(*cloud_out);
	pcl::console::print_highlight("point cloud size after downsizing: ");
	cout << cloud_out->size() << endl;
	clock_t end = clock();
	double time = (double)(end - start) / (double)CLOCKS_PER_SEC;
	pcl::console::print_highlight("time used for voxel grid filtering: ");
	cout << time << "s. " << endl;
}

void remove_zero(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_remove(new PointCloudT);
	for (auto iter = cloud->begin(); iter != cloud->end(); iter++) {
		if (iter->x == 0.0 && iter->y == 0.0 && iter->z == 0.0) {

		}
		else {
			PointT point(iter->x, iter->y, iter->z);
			cloud_remove->push_back(point);
		}
	}
	cloud = cloud_remove;
	cloud_remove.~shared_ptr();
}

void single_cloud_visualization(PointCloudT::Ptr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
	viewer->addPointCloud(cloud, "cloud_" + clock() / CLOCKS_PER_SEC);
	while (!viewer->wasStopped()) {
		viewer->spinOnce(1000);
	}
	viewer->close();
}

void single_cloud_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float coordinate_size)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer_wc"));
	viewer->addPointCloud(cloud, "cloud_" + clock() / CLOCKS_PER_SEC);
	viewer->addCoordinateSystem(coordinate_size);
	while (!viewer->wasStopped()) {
		viewer->spinOnce(1000);
	}
	viewer->close();
}

double compute_point_cloud_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	return compute_point_cloud_resolution(cloud, 20.0f);
}

double compute_point_cloud_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float k_nn)
{
	clock_t start = clock();

	pcl::KdTreeFLANN<PointT>::Ptr kd_tree(new pcl::KdTreeFLANN<PointT>);
	kd_tree->setInputCloud(cloud);
	vector<float> k_distances(2);
	vector<int> k_indexs(2);
	float res = 0.0f;
	for (auto point : *cloud) {
		kd_tree->nearestKSearch(point, k_nn, k_indexs, k_distances);
		float sum = 0.0f;
		for (float k_distance : k_distances) {
			sum = sum + k_distance;
		}
		res += sum / k_distances.size();
	}
	res /= cloud->size();
	clock_t end = clock();
	pcl::console::print_highlight("time used for calculating cloud resolution: ");
	cout << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	return res;
}
