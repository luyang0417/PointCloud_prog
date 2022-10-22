#include "common.h"
#include "extration.h"

void extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	PointCloudN::Ptr normals = compute_normal_cloud(cloud, kdtree, 20);

	PointCloudT::Ptr seam(new PointCloudT);
	pcl::ModelCoefficients::Ptr coeffients_init_b(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_init_b(new pcl::PointIndices);
	// 外部输入支管圆柱半径及厚度
	cout << "enter the lower limit of branch pipe radius: ";
	float lower_radius_b;
	cin >> lower_radius_b;
	cout << "enter the upper limit of branch pipe radius: ";
	float upper_radius_b;
	cin >> upper_radius_b;
	// 外部输入主管圆柱半径及厚度
	cout << "enter the lower limit of main pipe radius: ";
	float lower_radius_m;
	cin >> lower_radius_m;
	cout << "enter the upper limit of main pipe radius: ";
	float upper_radius_m;
	cin >> upper_radius_m;
	cout << "enter tolerance threshold of fitted model: ";
	float thickness_threshold;
	cin >> thickness_threshold;
	ransac_segment_cylinder(cloud, normals, thickness_threshold, lower_radius_b, upper_radius_b, coeffients_init_b, inliers_init_b);
	pcl::console::print_highlight("branch pipe cylinder parameters: [");
	for (int i = 0; i < coeffients_init_b->values.size(); ++i) {
		if (i != coeffients_init_b->values.size() - 1) {
			cout << coeffients_init_b->values.at(i) << " ";
		}
		else {
			cout << coeffients_init_b->values.at(i);
		}
	}
	cout << "]" << endl;

	// 摆正
	// 使点云坐标系与支管坐标系重合
	float x = coeffients_init_b->values[0];
	float y = coeffients_init_b->values[1];
	float z = coeffients_init_b->values[2];
	float rx = coeffients_init_b->values[3];
	float ry = coeffients_init_b->values[4];
	float rz = coeffients_init_b->values[5];
	float radius = coeffients_init_b->values[6];
	float angle_x, angle_y;

	// 圆柱轴线可视化
	viz_cylinder_axis(cloud, x, y, z, rx, ry, rz);
	// 移到中心
	pointcloud_Move(cloud, -x, -y, -z);
	single_cloud_visualization(cloud, 100);

	if (abs(rx - 1) < 1e-2 || abs(rx + 1) < 1e-2 || abs(ry) < 1e-2) {
		angle_x = 0;
	}
	else {
		angle_x = asin(abs(ry) / sqrt(1 - pow(rx, 2))) * 180 / M_PI;
		
	}
	if (ry * rz < 0) {
		angle_x *= -1;
	}
	angle_y = 90 - acos(rx) * 180 / M_PI;
	if (rz < 0) {
		angle_y *= -1;
	}

	pcl::console::print_highlight("rotating to z axis direction...\n");
	pointcloud_Rotate(cloud, -angle_x, 0, 0);
	pcl::console::print_highlight("after x rotation...\n");
	single_cloud_visualization(cloud, 100);
	pointcloud_Rotate(cloud, 0, angle_y, 0);
	pcl::console::print_highlight("after y rotation...\n");
	single_cloud_visualization(cloud, 100);
	if (rz < 0) {
		pointcloud_Rotate(cloud, 180, 0, 0);
		pcl::console::print_highlight("after z reverse...\n");
		single_cloud_visualization(cloud, 100);
	}
	pcl::console::print_highlight("need reverse z? y/n ");
	string flag;
	cin >> flag;
	if (flag == "y") {
		pointcloud_Rotate(cloud, 180, 0, 0);
		pcl::console::print_highlight("after z reverse...\n");
		single_cloud_visualization(cloud, 100);
	}	
	PointCloudN::Ptr normals_m = compute_normal_cloud(cloud, kdtree, 20);
	pcl::ModelCoefficients::Ptr coeffients_init_m(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_init_m(new pcl::PointIndices);
	ransac_segment_cylinder(cloud, normals_m, thickness_threshold, lower_radius_m, upper_radius_m, coeffients_init_m, inliers_init_m);
	pcl::console::print_highlight("main pipe cylinder parameters: [");
	for (int i = 0; i < coeffients_init_m->values.size(); ++i) {
		if (i != coeffients_init_m->values.size() - 1) {
			cout << coeffients_init_m->values.at(i) << " ";
		}
		else {
			cout << coeffients_init_m->values.at(i);
		}
	}
	cout << "]" << endl;
	float x_m = coeffients_init_m->values[0];
	float y_m = coeffients_init_m->values[1];
	float z_m = coeffients_init_m->values[2];
	float rx_m = coeffients_init_m->values[3];
	float ry_m = coeffients_init_m->values[4];
	float rz_m = coeffients_init_m->values[5];
	float radius_m = coeffients_init_m->values[6];
	viz_cylinder_axis(cloud, x_m, y_m, z_m, rx_m, ry_m, rz_m);

	float angle_m_z = acos(ry_m) * 180 / M_PI;
	if (rx_m > 0) {
		angle_m_z *= -1;
	}
	pointcloud_Move(cloud, 0, 0, -1 * coeffients_init_m->values[2]);
	pointcloud_Rotate(cloud, 0, 0, angle_m_z);
	// 摆正完成
	pcl::console::print_highlight("rotation finished...\n");
	single_cloud_visualization(cloud, 100);
	
	// 计算出的焊脚点
	PointCloudT::Ptr cloud_cal_1(new PointCloudT);
	PointCloudT::Ptr cloud_cal_2(new PointCloudT);
	cout << "Enter layer thickness" << endl;
	float layer_thickness;
	cin >> layer_thickness;
	float radius_inner = radius - 21;
	for (float angle = 0; angle < 360; angle += 0.1) {
		float x1, x2, y1, y2, z1, z2;
		// 不能用radius，应该用内圈
		
		x1 = x2 = (radius_inner + layer_thickness) * cos(angle * M_PI / 180);
		y1 = y2 = (radius_inner + layer_thickness) * sin(angle * M_PI / 180);
		z1 = sqrt(pow(coeffients_init_m->values[6], 2) - pow(radius_inner * cos(angle * M_PI / 180), 2)) +
			 layer_thickness * tan(40 * M_PI / 180);
		z2 = sqrt(pow(coeffients_init_m->values[6], 2) - pow(x1, 2));

		cloud_cal_1->push_back(*new PointT(x1, y1, z1));
		cloud_cal_2->push_back(*new PointT(x2, y2, z2));
	}
	pcl::visualization::PCLVisualizer::Ptr viewer_cal(new pcl::visualization::PCLVisualizer("cal"));
	viewer_cal->addPointCloud(cloud);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_cal_1, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud_cal_2, 0, 255, 0);
	viewer_cal->addPointCloud(cloud_cal_1, red, "cal_1");
	viewer_cal->addPointCloud(cloud_cal_2, green, "cal_2");
	while (!viewer_cal->wasStopped()) {
		viewer_cal->spinOnce(1000);
	}
	viewer_cal->close();


	// 提取支管圆柱
	pcl::ModelCoefficients::Ptr coeffients_b(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_b(new pcl::PointIndices);

	PointCloudN::Ptr normals_2 = compute_normal_cloud(cloud, kdtree, 20);
	ransac_segment_cylinder(cloud, normals_2, thickness_threshold, lower_radius_b, upper_radius_b, coeffients_b, inliers_b);
	// 去除支管圆柱
	PointCloudT::Ptr inliers_cloud(new PointCloudT);
	PointCloudT::Ptr outliers_cloud(new PointCloudT);
	PointCloudT::Ptr outliers_cloud_b(new PointCloudT);
	extract_inliers_by_indices(cloud, inliers_cloud, inliers_b, true);
	extract_inliers_by_indices(cloud, outliers_cloud_b, inliers_b, false);
	for (PointT point : *outliers_cloud_b) {
		// 对于 ransac 误提取的支管点云，重新划分回到inliers_cloud中
		if (point.z < radius_m + 1) {
			inliers_cloud->emplace_back(point);
		}
		else {
			outliers_cloud->emplace_back(point);
		}
	}
	pcl::console::print_highlight("inliers...\n");
	single_cloud_visualization(inliers_cloud);
	pcl::console::print_highlight("outliers...\n");
	single_cloud_visualization(outliers_cloud);

	// 提取主管圆柱
	pcl::ModelCoefficients::Ptr coeffients_m(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_m(new pcl::PointIndices);

	PointCloudN::Ptr normals_3 = compute_normal_cloud(inliers_cloud, kdtree, 20);
	ransac_segment_cylinder(inliers_cloud, normals_3, thickness_threshold, lower_radius_m, upper_radius_m, coeffients_m, inliers_m);
	// 去除主管圆柱
	PointCloudT::Ptr outliers_cloud_m(new PointCloudT);
	extract_inliers_by_indices(inliers_cloud, cloud_out, inliers_m, true);
	extract_inliers_by_indices(inliers_cloud, outliers_cloud_m, inliers_m, false);
	for (PointT point : *outliers_cloud_m) {
		// 对于 ransac 误提取的主管点云，重新划分回到seam中
		if (pow(point.x, 2) + pow(point.y, 2) <= pow(upper_radius_b, 2)) {
			cloud_out->emplace_back(point);
		}
		else {
			outliers_cloud->emplace_back(point);
		}
	}
	pcl::console::print_highlight("seam point cloud...\n");
	single_cloud_visualization(cloud_out);
	pcl::console::print_highlight("outliers...\n");
	single_cloud_visualization(outliers_cloud);
}

void normal_visualization(PointCloudT::Ptr cloud, PointCloudN::Ptr normals, PointCloudT::Ptr viewport)
{
	normal_visualization(cloud, normals, viewport, 20);
}

void normal_visualization(PointCloudT::Ptr cloud, PointCloudN::Ptr normals, PointCloudT::Ptr viewport, int level)
{
	
	pcl::visualization::PCLVisualizer::Ptr viewer_normal(new pcl::visualization::PCLVisualizer("viewer_normal"));
	viewer_normal->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(cloud, 255, 0, 0);
	viewer_normal->addPointCloud(cloud, cloud_color, "viewer_pcd");
	viewer_normal->addPointCloudNormals<PointT, PointN>(cloud, normals, level, 5, "normal");
	viewer_normal->addCoordinateSystem(10);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_view(viewport, 0, 255, 0);
	viewer_normal->addPointCloud(viewport, cloud_view, "viewport");
	viewer_normal->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "viewport");
	while (!viewer_normal->wasStopped()) {
		viewer_normal->spinOnce(1000);
	}
	viewer_normal->close();
}

void ransac_segment_cylinder(PointCloudT::Ptr& cloud, PointCloudN::Ptr& normals, double distance_threshold, double radius_limit_low, double radius_limit_high, pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers)
{
	pcl::SACSegmentationFromNormals<PointT, PointN> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setNormalDistanceWeight(0.1);
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.setRadiusLimits(radius_limit_low, radius_limit_high);
	seg.setNumberOfThreads(10);
	seg.segment(*inliers, *coefficients);
}

void extract_inliers_by_indices(PointCloudT::Ptr& cloud, PointCloudT::Ptr& inliers_cloud, pcl::PointIndices::Ptr& inliers, bool setNegative)
{
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(setNegative);
	extract.filter(*inliers_cloud);
}

void extract_normals_by_indices(PointCloudN::Ptr& normals, PointCloudN::Ptr& inliers_cloud, pcl::PointIndices::Ptr& inliers, bool setNegative)
{
	pcl::ExtractIndices<PointN> extract_normal;
	extract_normal.setInputCloud(normals);
	extract_normal.setIndices(inliers);
	extract_normal.setNegative(setNegative);
	extract_normal.filter(*inliers_cloud);
}

double compute_edge_intensity(PointCloudT::Ptr& cloud, int index, pcl::KdTreeFLANN<PointT>::Ptr kd_tree, double radius)
{
	PointT point = cloud->at(index);
	PointCloudT::Ptr cloud_neighbour(new PointCloudT);
	cloud_neighbour->push_back(point);
	vector<float> k_distance(2);
	vector<int> k_indexs(2);
	kd_tree->radiusSearch(point, radius, k_indexs, k_distance);
	double max_dist(0);
	for (auto k_d : k_distance) {
		max_dist = max_dist > k_d ? max_dist : k_d;
	}
	radius = radius < max_dist ? radius : max_dist;
	for (auto k_index : k_indexs) {
		cloud_neighbour->push_back(cloud->at(k_index));
	}
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud_neighbour, centroid);
	double edge_intensity = compute_sqrt(centroid[0], centroid[1], centroid[2], point.x, point.y, point.z) / radius;
	return edge_intensity;
}

PointCloudT::Ptr edge_points_detection(PointCloudT::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree, double radius, double threshold)
{
	clock_t start = clock();
	PointCloudT::Ptr edge_points(new PointCloudT);
	for (auto i = 0; i < cloud->size();++i) {
		double edge_intensity = compute_edge_intensity(cloud, i, kd_tree, radius);
		if (i % 1000 == 0) {
			pcl::console::print_highlight("edge intensity of point ");
			cout << i << " is: " << edge_intensity << endl;
		}
		if (edge_intensity >= threshold) {
			edge_points->push_back(cloud->at(i));
		}
	}
	clock_t end = clock();
	pcl::console::print_highlight("time used for edge detection: ");
	cout << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	return edge_points;
}

PointCloudT::Ptr edge_points_detection(PointCloudT::Ptr & cloud, double radius, double ratio)
{
	pcl::KdTreeFLANN<PointT>::Ptr kd_tree(new pcl::KdTreeFLANN<PointT>);
	kd_tree->setInputCloud(cloud);
	vector<float> intensities;
	clock_t start = clock();
	PointCloudT::Ptr edge_points(new PointCloudT);
	for (auto i = 0; i < cloud->size();++i) {
		double edge_intensity = compute_edge_intensity(cloud, i, kd_tree, radius);
		intensities.push_back(edge_intensity);
	}
	int size = intensities.size();
	sort(intensities.begin(), intensities.end());
	double threshold = intensities.at(size * ratio);

	for (auto i = 0; i < cloud->size(); ++i) {
		double edge_intensity = compute_edge_intensity(cloud, i, kd_tree, radius);
		if (i % 1000 == 0) {
			pcl::console::print_highlight("edge intensity of point ");
			cout << i << " is: " << edge_intensity << endl;
		}
		if (edge_intensity >= threshold) {
			if(pow(cloud->at(i).x, 2) + pow(cloud->at(i).y, 2) <= pow(55, 2))
				edge_points->push_back(cloud->at(i));
		}
	}
	clock_t end = clock();
	pcl::console::print_highlight("time used for edge detection: ");
	cout << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	return edge_points;
}

void viz_cylinder_axis(PointCloudT::Ptr& cloud, float x, float y, float z, float rx, float ry, float rz)
{
	cout << "visualize cylinder axis? y/n ";
	string viz_axis;
	cin >> viz_axis;
	if (viz_axis == "y") {
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer_cylinder_axis"));
		viewer->addPointCloud(cloud, "raw cloud");
		PointCloudT::Ptr cloud_axis(new PointCloudT);
		for (int i = -100; i < 101; i += 1) {
			cloud_axis->push_back(*(new PointT(x + i * rx, y + i * ry, z + i * rz)));
		}
		pcl::visualization::PointCloudColorHandlerCustom<PointT> red_axis(cloud_axis, 255, 0, 0);
		viewer->addPointCloud(cloud_axis, red_axis, "axis cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "axis cloud");
		while (!viewer->wasStopped()) {
			viewer->spinOnce(1000);
		}
		viewer->close();
		system("pause");
	}
}

void viz_cylinder_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients::Ptr& coefficients)
{
	cout << "visualize cylinder axis? y/n ";
	string viz_axis;
	cin >> viz_axis;
	if (viz_axis == "y") {
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer_axis"));
		pcl::ModelCoefficients::Ptr coefficients_axis(new pcl::ModelCoefficients);
		for (int i = 0; i < 6; i++) {
			coefficients_axis->values[i] = coefficients->values[i];
		}
		viewer->addLine(*coefficients_axis, "axis");
		while (!viewer->wasStopped()) {
			viewer->spinOnce(1000);
		}
		viewer->close();
		system("pause");
	}
}

PointCloudN::Ptr compute_normal_cloud(PointCloudT::Ptr& cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree, int k_search)
{
	pcl::console::print_highlight("Start computing normals...\n");
	clock_t start = clock();
	pcl::NormalEstimationOMP<PointT, PointN> ne;
	PointCloudT::Ptr viewport(new PointCloudT);
	// viewport->push_back(*new PointT(0, 0, 500));
	PointCloudN::Ptr normals(new PointCloudN);
	ne.setNumberOfThreads(10);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(kdtree);
	ne.setKSearch(k_search);
	ne.compute(*normals);
	clock_t end = clock();
	pcl::console::print_highlight("Normal estimation complete\n");
	pcl::console::print_highlight("time used for computing normals: ");
	cout << (double)(end - start) / (double)CLOCKS_PER_SEC << " s." << endl;
	pcl::console::print_highlight("Need visualizing normals? y/n ");
	string need_viz_normal;
	cin >> need_viz_normal;
	if (need_viz_normal == "y") {
		normal_visualization(cloud, normals, viewport, 20);
	}
	return normals;
}

