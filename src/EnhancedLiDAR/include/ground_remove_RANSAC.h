#include <pcl/point_types.h>   //点类型头文件
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "geometry.h"

inline void ground_remove_RANSAC(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud_ptr)
{
    // 创建一个分割器
    pcl::SACSegmentation<pcl::PointXYZI> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory-设置目标几何形状
    seg.setModelType (pcl::SACMODEL_PLANE);

    //分割方法：随机采样法
    seg.setMethodType (pcl::SAC_RANSAC);

    //设置误差容忍范围(距离阈值) 单位m
    seg.setDistanceThreshold (0.08);

    //第一次找到的地面的法向量
    Point3 first_n_vector(0, 0, 1);

    //将原点云拷贝给object
    pcl::copyPointCloud(*input_cloud_ptr, *object_cloud_ptr);

    //存放一些点数很多的平面型的object
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_object_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    while(1)
    {
        //创建一个模型参数对象，用于记录结果(法向量)
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        //inliers表示误差能容忍的点 记录的是点云的序号
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        //输入点云
        seg.setInputCloud (object_cloud_ptr);

        //分割点云
        seg.segment (*inliers, *coefficients);

        if(inliers->indices.size () < 4000)
            break;

        Point3 n_vector(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

        //单位化法向量
        n_vector = n_vector / sqrt(n_vector * n_vector);

        if (fabs(fabs(n_vector * first_n_vector) - 1) > 0.01
            || (coefficients->values[2]>0?coefficients->values[3]:-coefficients->values[3])<1.55) //非地面的平面物体
        {
            //将检测到的平面提取到tmp_object_cloud_ptr
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_object_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(object_cloud_ptr);
            extract.setIndices (inliers);
            extract.filter (*tmp_object_cloud_ptr);
            *plane_object_cloud_ptr = *plane_object_cloud_ptr + *tmp_object_cloud_ptr;

            //剩余点提取到object
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_object_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            extract.setNegative (true);
            extract.filter (*new_object_cloud_ptr);
            pcl::copyPointCloud(*new_object_cloud_ptr, *object_cloud_ptr);

            //打印法向量
            //printf("Not ground: (%.1f,%.1f,%.1f),%lf\n", n_vector.x, n_vector.y, n_vector.z,coefficients->values[3]);
        }
        else//检测到的是地面
        {
            //提取ground
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(object_cloud_ptr);
            extract.setIndices (inliers);
            extract.filter (*new_ground_cloud_ptr);
            *ground_cloud_ptr = *ground_cloud_ptr + *new_ground_cloud_ptr;

            //提取object
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_object_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            extract.setNegative (true);
            extract.filter (*new_object_cloud_ptr);
            pcl::copyPointCloud(*new_object_cloud_ptr, *object_cloud_ptr);

            //打印法向量
            //printf("Ground: (%.1f,%.1f,%.1f),%lf\n", n_vector.x, n_vector.y, n_vector.z,coefficients->values[3]);
        }
    }

    *object_cloud_ptr = *object_cloud_ptr + *plane_object_cloud_ptr;

    printf("ground_cloud_ptr->size() = %d\n",(int)(ground_cloud_ptr->size() ));
    printf("object_cloud_ptr->size() = %d\n",(int)(object_cloud_ptr->size() ));

    return;
}