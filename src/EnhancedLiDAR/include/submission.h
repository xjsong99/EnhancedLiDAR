#include <cstdio>
#include <algorithm>
#include "EnhancedLiDAR_core.h"
#include "ground_remove_RANSAC.h"

void submission(int start_index)
{
    OrganizeTool OrgTool;

    float *data = (float*)malloc(1000000*sizeof(float));
    FILE *stream;
    
    std::string cloud_file = "/media/song/程序磁盘/3D_KITTI/data_object_velodyne/training/velodyne/";
    std::string image_file = "/media/song/程序磁盘/3D_KITTI/image_2/training/image_2/";
    std::string depth_file = "/media/song/程序磁盘/3D_KITTI/predicted_depth/training/";
    std::string calib_file = "/media/song/程序磁盘/3D_KITTI/data_object_calib/training/calib/";
    std::string dense_object_file = "/media/song/程序磁盘/3D_KITTI/dense_object_velodyne/training/";
    std::string dense_full_file = "/media/song/程序磁盘/3D_KITTI/dense_full_velodyne/training/";

    struct dirent **namelist_cloud, **namelist_depth, **namelist_calib, **namelist_image; //文件名list

    int num_of_file = scandir(cloud_file.c_str(), &namelist_cloud, PclTestCore::fileNameFilter_bin, alphasort);
    scandir(depth_file.c_str(), &namelist_depth, PclTestCore::fileNameFilter_txt, alphasort);
    scandir(calib_file.c_str(), &namelist_calib, PclTestCore::fileNameFilter_txt, alphasort);
    scandir(image_file.c_str(), &namelist_image, PclTestCore::fileNameFilter_png, alphasort);

    printf("num_of_file=%d\n",num_of_file);

    for (int file_index = start_index; file_index < num_of_file; file_index++)
    {
        cv::Mat Im = cv::imread((image_file+std::string(namelist_image[file_index]->d_name)).c_str());
        OrgTool.row_of_depth_image = Im.rows;
        OrgTool.col_of_depth_image = Im.cols;

        printf("file_index=%d\n",file_index);

        stream = fopen ((calib_file+std::string(namelist_calib[file_index]->d_name)).c_str(),"r");
        OrgTool.read_calib_submission(stream);
        fclose(stream);

        pcl::PointCloud<pcl::PointXYZI>::Ptr original_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr object_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr dense_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        double depth_image[400][1300];

        // allocate 4 MB buffer (only ~130*4*4 KB are needed)
        int num = 1000000;

        stream = fopen ((cloud_file+std::string(namelist_cloud[file_index]->d_name)).c_str(),"rb");
        num = fread(data,sizeof(float),num,stream)/4;

        // pointers
        float *px = data+0;
        float *py = data+1;
        float *pz = data+2;
        float *pr = data+3;

        for (int i=0; i<num; i++) {
            pcl::PointXYZI tmp;
            tmp.x = *px; tmp.y = *py; tmp.z = *pz; tmp.intensity = *pr;
            original_pc_ptr->points.push_back(tmp);
            px+=4; py+=4; pz+=4; pr+=4;
        }
        fclose(stream);

        OrgTool.eraseNAN(original_pc_ptr);//剔除NAN点

        stream = fopen ((depth_file+std::string(namelist_depth[file_index]->d_name)).c_str(),"r");
        for (int i = 0; i < OrgTool.row_of_depth_image; i++)
            for (int j = 0; j < OrgTool.col_of_depth_image; j++)
                fscanf(stream,"%lf",&depth_image[i][j]);
        fclose(stream);

        ground_remove_RANSAC(original_pc_ptr, ground_pc_ptr, object_pc_ptr);

        OrgTool.denser_OpenMP(object_pc_ptr, dense_pc_ptr, &depth_image[0][0]);//用深度进行点云稠密化

        *object_pc_ptr = *object_pc_ptr + *dense_pc_ptr;
        stream = fopen ((dense_object_file+std::string(namelist_cloud[file_index]->d_name)).c_str(),"wb");
        int num_of_point = object_pc_ptr->size();
        for (int i = 0; i < num_of_point; i++)
        {
            fwrite(&(object_pc_ptr->points.at(i).x), sizeof(float), 1, stream);
            fwrite(&(object_pc_ptr->points.at(i).y), sizeof(float), 1, stream);
            fwrite(&(object_pc_ptr->points.at(i).z), sizeof(float), 1, stream);
            fwrite(&(object_pc_ptr->points.at(i).intensity), sizeof(float), 1, stream);
        }
        fclose(stream);

        *original_pc_ptr = *original_pc_ptr + *dense_pc_ptr;
        stream = fopen ((dense_full_file+std::string(namelist_cloud[file_index]->d_name)).c_str(),"wb");
        num_of_point = original_pc_ptr->size();
        for (int i = 0; i < num_of_point; i++)
        {
            fwrite(&(original_pc_ptr->points.at(i).x), sizeof(float), 1, stream);
            fwrite(&(original_pc_ptr->points.at(i).y), sizeof(float), 1, stream);
            fwrite(&(original_pc_ptr->points.at(i).z), sizeof(float), 1, stream);
            fwrite(&(original_pc_ptr->points.at(i).intensity), sizeof(float), 1, stream);
        }
        fclose(stream);
    }



    return;
}