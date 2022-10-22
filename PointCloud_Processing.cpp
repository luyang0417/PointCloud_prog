#include "common.h"
#include "registration.h"
#include "extration.h"

int main(int argc, char** argv)
{
    PointCloudT::Ptr cloud_fused(new PointCloudT);
    PointCloudT::Ptr cloud_seam(new PointCloudT);
    while (true) {
        PointCloudT::Ptr cloud_source(new PointCloudT);
        PointCloudT::Ptr cloud_target(new PointCloudT);
        swap(cloud_target, cloud_fused);
        cloud_fused->clear();
        // 读入下一个配准的源点云到cloud_source
        cout << "enter file name, end with .pcd\n";
        string file;
        cin >> file;
        if (file.size() < 5 || file.substr(file.size() - 4, file.size()) != ".pcd") {
            cout << "Error file format!\n";
            continue;
        }
        if (0 == load_pcd_cloud(file, cloud_source)) {
            cout << "File not found!\n";
            continue;
        } 
        if (cloud_target->size() == 0) {
            registration(cloud_source, cloud_fused);
        }
        else {
            registration(cloud_source, cloud_target, cloud_fused);
        }
        single_cloud_visualization(cloud_fused);
        // 跳出条件
        cout << "need save fused point cloud? y/n ";
        string need_save_fused_cloud;
        cin >> need_save_fused_cloud;
        if (need_save_fused_cloud == "y") {
            cout << "enter save file name: ";
            string save_file_name;
            cin >> save_file_name;
            pcl::io::savePCDFile(save_file_name, *cloud_fused);
        }
        cout << "exit registration? y/n ";
        string exit_registration;
        cin >> exit_registration;
        if (exit_registration == "y") {
            pcl::console::print_highlight("registratoin complete\n");
            break;
        }  
    }
    // 在配准融合后的点云中提取特征
    extraction(cloud_fused, cloud_seam);
    PointCloudT::Ptr cloud_edge = edge_points_detection(cloud_seam, 10, 0.75);
    single_cloud_visualization(cloud_edge);
    system("pause");
}