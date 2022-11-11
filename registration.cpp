#include "common.h"
#include "registration.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointN> PointCloudN;
using namespace std;
boost::mutex cloud_mutex;

void registration(PointCloudT::Ptr cloud_source_in, PointCloudT::Ptr cloud_target_in, PointCloudT::Ptr &cloud_out)
{
	PointCloudT::Ptr cloud_source = cloud_source_in;
	PointCloudT::Ptr cloud_target = cloud_target_in;
	remove_zero(cloud_source);
	pcl::console::print_highlight("source point cloud after removing zero points\n");
	single_cloud_visualization(cloud_source);
	remove_zero(cloud_target);
	pcl::console::print_highlight("target point cloud after removing zero points\n");
	single_cloud_visualization(cloud_target);

	// 选roi，降低点云规模，提高计算效率
	
	// 降采样，降低点云密度，提高计算效率
	vector<float> leaf_size{ 0.005, 0.005, 0.005 };
	PointCloudT::Ptr cloud_voxel_src(new PointCloudT);
	// voxel_downsizing(cloud_source, cloud_voxel_src, leaf_size);
	voxel_downsizing(cloud_source, cloud_voxel_src);
	PointCloudT::Ptr cloud_voxel_tgt(new PointCloudT);
	// voxel_downsizing(cloud_target, cloud_voxel_tgt, leaf_size);
	voxel_downsizing(cloud_target, cloud_voxel_tgt);

	// 先进行粗配准，使两片点云靠近
	Eigen::Matrix4f coarse_regis_trans = Eigen::Matrix4f::Identity();
	cout << "please select which coarse registration method to use? 1: By Robot Pos; 2: By coarse registration algorithm.\n";
	int flag_cregis(2);
	cin >> flag_cregis;
	if (flag_cregis == 1) {
		// 可以直接读取机器人关节角度，通过矩阵变换进行粗配准
		// 机器人接口，用来获取数据
		Eigen::Matrix4f camera_to_tool_coordinate = Eigen::Matrix4f::Identity(); // 相机坐标系到工具坐标系的坐标变换矩阵
		Eigen::Matrix4f tool_to_base_coordinate_source = Eigen::Matrix4f::Identity(); // 源点云的工具坐标系到基坐标系的坐标变换矩阵
		Eigen::Matrix4f tool_to_base_coordinate_target = Eigen::Matrix4f::Identity(); // 目标点云的工具坐标系到基坐标系的坐标变换矩阵
		// 计算粗配准矩阵
		coarse_regis_trans *= camera_to_tool_coordinate.inverse();
		coarse_regis_trans *= tool_to_base_coordinate_source.inverse();
		coarse_regis_trans *= tool_to_base_coordinate_target;
		coarse_regis_trans *= camera_to_tool_coordinate;
	}
	else {
		// 或者采用粗配准算法
		coarse_regis_trans = coarse_registration(cloud_source, cloud_target);
	}

	PointCloudT::Ptr sac_result(new PointCloudT);
	pcl::transformPointCloud(*cloud_source, *sac_result, coarse_regis_trans);
	pcl::visualization::PCLVisualizer::Ptr viewer_coarse(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_sac(cloud_target, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_sac(sac_result, 0, 0, 255);
	viewer_coarse->addPointCloud(cloud_target, red_sac, "11");
	viewer_coarse->addPointCloud(sac_result, blue_sac, "22");
	while (!viewer_coarse->wasStopped()) {
		viewer_coarse->spinOnce(1000);
	}
	viewer_coarse->close();

	PointCloudT::Ptr sac_voxel(new PointCloudT);
	// voxel_downsizing(sac_result, sac_voxel, vector<float>{ 0.005, 0.005, 0.005 });
	voxel_downsizing(sac_result, sac_voxel);

	pcl::console::print_highlight("Please enter approximate overlapping ration: ");
	float overlap_ratio(1.0f);
	cin >> overlap_ratio;
	if (overlap_ratio < 0 || overlap_ratio > 1) {
		pcl::console::print_error("invalid overlap ratio!");
		return;
	}

	// 用icp算法进行精配准
	// icp
	/*
	clock_t start_icp = clock();
	PointCloudT::Ptr cloud_icp(new PointCloudT);
	pcl::IterativeClosestPoint<PointT, PointT, float> icp;
	icp.setInputSource(sac_voxel);
	icp.setInputTarget(cloud_target);
	icp.setMaxCorrespondenceDistance(1);
	icp.setMaximumIterations(100);
	icp.align(*cloud_icp);
	clock_t end_icp = clock();
	Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
	pcl::transformPointCloud(*sac_result, *cloud_icp, icp_trans);
	pcl::console::print_highlight("time used for icp: ");
	cout << (double)(end_icp - start_icp) / (double)CLOCKS_PER_SEC << " s." << endl;
	*/
	PointCloudT::Ptr cloud_icp(new PointCloudT);
	Eigen::Matrix4f icp_trans = icp_trans_matrix(sac_voxel, cloud_target);
	pcl::transformPointCloud(*sac_result, *cloud_icp, icp_trans);
	pcl::visualization::PCLVisualizer::Ptr viewer_icp(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_icp(cloud_source, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green_icp(cloud_target, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_icp(cloud_icp, 0, 0, 255);
	viewer_icp->addPointCloud(cloud_source, red_icp, "icp_s");
	viewer_icp->addPointCloud(cloud_target, green_icp, "icp_t");
	viewer_icp->addPointCloud(cloud_icp, blue_icp, "icp");
	while (!viewer_icp->wasStopped()) {
		viewer_icp->spinOnce(1000);
	}
	viewer_icp->close();

	// tricp
	/*
	clock_t start_tricp = clock();
	PointCloudT::Ptr cloud_tricp(new PointCloudT);
	pcl::recognition::TrimmedICP<PointT, double> tricp;
	Eigen::Matrix4d tricp_trans = Eigen::Matrix4d::Identity();
	tricp.init(cloud_target);
	// tricp.align(*cloud_voxel_src, (int)cloud_voxel_src->size() * overlap_ratio, tricp_trans);
	tricp.align(*sac_result, (int)sac_result->size() * overlap_ratio, tricp_trans);

	pcl::transformPointCloud(*sac_result, *cloud_tricp, tricp_trans);
	clock_t end_tricp = clock();
	pcl::console::print_highlight("time used for tricp: ");
	cout << (double)(end_tricp - start_tricp) / (double)CLOCKS_PER_SEC << " s." << endl;
	*/
	PointCloudT::Ptr cloud_tricp(new PointCloudT);
	Eigen::Matrix4d tricp_trans = tricp_trans_matrix(sac_result, cloud_target, overlap_ratio);
	pcl::transformPointCloud(*sac_result, *cloud_tricp, tricp_trans);
	pcl::visualization::PCLVisualizer::Ptr viewer_tricp(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_tricp(cloud_source, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green_tricp(cloud_target, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_tricp(cloud_tricp, 0, 0, 255);
	viewer_tricp->addPointCloud(cloud_source, red_tricp, "cloud_s");
	viewer_tricp->addPointCloud(cloud_target, green_tricp, "cloud_t");
	viewer_tricp->addPointCloud(cloud_tricp, blue_tricp, "cloud_regis");
	while (!viewer_tricp->wasStopped()) {
		viewer_tricp->spinOnce(1000);
	} 
	viewer_tricp->close();
	system("pause");

	for (PointT point : *cloud_target) {
		cloud_out->push_back(point);
	}
	pcl::console::print_highlight("use: 1. icp result; 2. tricp result\n");
	int choice;
	cin >> choice;
	double mark;
	switch (choice) {
	case 1:
		mark = registration_mark(cloud_icp, cloud_target, 1.0);
		cout << "Registration mark is: " << mark << endl;
		for (PointT point : *cloud_icp) {
			cloud_out->push_back(point);
		}
		break;
	case 2:
		mark = registration_mark(cloud_tricp, cloud_target, overlap_ratio);
		cout << "Registration mark is: " << mark << endl;
		for (PointT point : *cloud_tricp) {
			cloud_out->push_back(point);
		}
		break;
	default:
		PCL_ERROR("wrong input");
	}
}
// 从同一片点云中选取两个部分出来配准，通常是用来做测试的
void registration(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out)
{
	string flag;
	PointCloudT::Ptr cloud_source(new PointCloudT);
	PointCloudT::Ptr cloud_target(new PointCloudT);
	pcl::console::print_highlight("need separating input cloud and do registration? y/n ");
	cin >> flag;
	if (flag == "y") {
		// area picking event
		roi_extraction(cloud_in, cloud_source, "cloud_roi_1");
		roi_extraction(cloud_in, cloud_target, "cloud_roi_2");
		// 不要移动太远
		pointcloud_Move(cloud_target, 5, 5, 5);
		pointcloud_Rotate(cloud_target, 10, 10, 10);

		pcl::visualization::PCLVisualizer::Ptr viewer_dual(new pcl::visualization::PCLVisualizer);
		viewer_dual->addPointCloud(cloud_source, "source_0");
		viewer_dual->addPointCloud(cloud_target, "target_0");
		viewer_dual->addCoordinateSystem(10);
		while (!viewer_dual->wasStopped()) {
			viewer_dual->spinOnce(1000);
		} 
		viewer_dual->close();
	}
	else {
		for (PointT point : *cloud_in) {
			cloud_out->push_back(point);
		}
		return;
	}
	// do registration
	registration(cloud_source, cloud_target, cloud_out);
}

void ap_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	callback_args* data = (callback_args*)args;
	vector<int> indices;
	if (!event.getPointsIndices(indices)) {
		return;
	} 
	for (int i = 0;i < indices.size();i++) {
		data->chosen_area_points_3d->push_back(data->orig_points->points[indices[i]]);
	}
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->chosen_area_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("chosen_points");
	data->viewerPtr->addPointCloud(data->chosen_area_points_3d, red, "chosen_points");
	// data->viewerPtr->updatePointCloud(data->chosen_area_points_3d, red, "chosen_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "chosen_points");
	pcl::console::print_highlight("need saving roi? y/n ");
	string flag;
	cin >> flag;
	if (flag == "y") {
		pcl::io::savePCDFile(data->filepath + ".pcd", *data->chosen_area_points_3d);
	}
	pcl::console::print_highlight("selected ");
	cout << indices.size() << " points, now sum is " << data->chosen_area_points_3d->size() << endl;
}

void roi_extraction(PointCloudT::Ptr cloud_input, PointCloudT::Ptr cloud_output, string filename)
{
	cloud_mutex.lock();
	pcl::visualization::PCLVisualizer::Ptr viewer_roi(new pcl::visualization::PCLVisualizer("viewer_roi"));
	viewer_roi->addPointCloud(cloud_input, filename);
	callback_args cb_args
	{
		cloud_input,
		NULL,
		cloud_output,
		viewer_roi,
		filename
	};
	viewer_roi->registerAreaPickingCallback(ap_callback, (void*)&cb_args);
	cloud_output = cb_args.chosen_area_points_3d;
	pcl::console::print_highlight("press x to enter select model, then draw an area with left click\n press 'Q' to exit viewer\n");
	while (!viewer_roi->wasStopped()) {
		viewer_roi->spinOnce();
	}
	viewer_roi->close();
	cloud_mutex.unlock();
	system("pause");
}

Eigen::Matrix4f coarse_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	vector<float> leaf_size(0);
	cout << "voxel leaf size: x = ";
	float size;
	cin >> size;
	leaf_size.push_back(size);
	cout << ", y = ";
	cin >> size;
	leaf_size.push_back(size);
	cout << ", z = ";
	cin >> size;
	leaf_size.push_back(size);
	PointCloudT::Ptr cloud_voxel_src(new PointCloudT);
	voxel_downsizing(cloud_source, cloud_voxel_src, leaf_size);
	PointCloudT::Ptr cloud_voxel_tgt(new PointCloudT);
	voxel_downsizing(cloud_target, cloud_voxel_tgt, leaf_size);
	clock_t start_fpfhs = clock();
	PointCloudN::Ptr normals_src(new PointCloudN);
	pcl::NormalEstimationOMP<PointT, PointN> est_normal;
	est_normal.setInputCloud(cloud_voxel_src);
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	est_normal.setSearchMethod(kdtree);
	est_normal.setKSearch(20);
	est_normal.compute(*normals_src);
	pcl::FPFHEstimationOMP<PointT, PointN, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloud_voxel_src);
	fpfh.setInputNormals(normals_src);
	fpfh.setSearchMethod(kdtree);
	fpfh.setKSearch(20);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_feature_src(new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfh.compute(*fpfh_feature_src);

	PointCloudN::Ptr normals_tgt(new PointCloudN);
	est_normal.setInputCloud(cloud_voxel_tgt);
	est_normal.setSearchMethod(kdtree);
	est_normal.setKSearch(20);
	est_normal.compute(*normals_tgt);
	fpfh.setInputCloud(cloud_voxel_tgt);
	fpfh.setInputNormals(normals_tgt);
	fpfh.setSearchMethod(kdtree);
	fpfh.setKSearch(20);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_feature_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfh.compute(*fpfh_feature_tgt);

	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_voxel_src);
	scia.setInputTarget(cloud_voxel_tgt);
	scia.setSourceFeatures(fpfh_feature_src);
	scia.setTargetFeatures(fpfh_feature_tgt);
	PointCloudT::Ptr sac_result(new PointCloudT);
	Eigen::Matrix4f sac_trans = Eigen::Matrix4f::Identity();
	scia.align(*sac_result);
	sac_trans = scia.getFinalTransformation();
	pcl::transformPointCloud(*cloud_source, *sac_result, sac_trans);
	clock_t end_fpfhs = clock();
	cout << "time used for ia-ransac registration: " << (double)(end_fpfhs - start_fpfhs) / (double)CLOCKS_PER_SEC << " s." << endl;
	
	return sac_trans;
}

double registration_mark(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target, float overlap_ratio)
{
	pcl::KdTreeFLANN<PointT> kd_tree;
	kd_tree.setInputCloud(cloud_target);
	vector<point_params> match_points;
	double sum = 0.0;
	for (auto i = 0; i < cloud_source->size(); ++i) {
		vector<float> k_distances(2);
		vector<int> k_indexs(2);
		kd_tree.nearestKSearch(cloud_source->at(i), 1, k_indexs, k_distances);
		point_params match_point{
			k_indexs[0],
			k_distances[0]
		};
		match_points.emplace_back(match_point);
	}
	sort(match_points.begin(), match_points.end(), less_sort);
	int point_count_take_into_account = (int)match_points.size() * overlap_ratio;
	for (auto i = 0; i < point_count_take_into_account; ++i) {
		sum += match_points[i].distance;
	}
	return sum / point_count_take_into_account;
}

Eigen::Matrix4f icp_trans_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	clock_t start_icp = clock();
	PointCloudT::Ptr cloud_icp(new PointCloudT);
	pcl::IterativeClosestPoint<PointT, PointT, float> icp;
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.setMaxCorrespondenceDistance(3);
	icp.setMaximumIterations(100);
	icp.align(*cloud_icp);
	clock_t end_icp = clock();
	Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
	pcl::console::print_highlight("time used for icp: ");
	cout << (double)(end_icp - start_icp) / (double)CLOCKS_PER_SEC << " s." << endl;
	return icp_trans;
}

Eigen::Matrix4d tricp_trans_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float overlap_ratio)
{
	clock_t start_tricp = clock();
	PointCloudT::Ptr cloud_tricp(new PointCloudT);
	pcl::recognition::TrimmedICP<PointT, double> tricp;
	Eigen::Matrix4d tricp_trans = Eigen::Matrix4d::Identity();
	tricp.init(cloud_target);
	// tricp.align(*cloud_voxel_src, (int)cloud_voxel_src->size() * overlap_ratio, tricp_trans);
	tricp.align(*cloud_source, (int)cloud_source->size() * overlap_ratio, tricp_trans);
	clock_t end_tricp = clock();
	pcl::console::print_highlight("time used for tricp: ");
	cout << (double)(end_tricp - start_tricp) / (double)CLOCKS_PER_SEC << " s." << endl;
	return tricp_trans;
}
