#pragma once
 
#include <ros/ros.h>      
//导入ROS系统包含核心公共头文件
 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>   //点类型头文件

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>  
 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
 
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <pcl/kdtree/kdtree_flann.h>//KD-Tree头文件

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>      //贪婪投影三角化算法

#include <pcl/visualization/pcl_visualizer.h>

#include "geometry.h"

class OrganizeTool
{
  private:
    int row_size , col_size;//行数列数
    float radius;//kdtree搜索半径
    unsigned int max_nn;//kdtree最多返回的点数
    float zero_alpha_threshold,alpha_threshold;//地面判断角度阈值

    int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1},
        dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    
    static float computeVerAngle(const pcl::PointXYZI &P);
    static float computeHorAngle(const pcl::PointXYZI &P);
    static bool cmp_order(const pcl::PointXYZI &lhs, const pcl::PointXYZI &rhs);
    inline void init(pcl::PointCloud<pcl::PointXYZI>::Ptr organised_cloud_ptr);//点云清空，初始化

    /*
    //2011_09_26
    double R00_inv[4][4] =  
    {{9.999238839427931e-01,	-9.869795589571715e-03,	7.402526949776609e-03,                         0},
     {9.837760022069099e-03,	9.999421785822904e-01,	4.351614783597308e-03,                         0},
     {-7.445048405358227e-03,	-4.278458830303748e-03,	9.999631646873483e-01,                         0},
     {                    0,                         0,                         0,     		           1}};
    double T_inv[4][4] = 
    { {7.533745e-3,	1.480249e-2,	9.998620e-1,	2.729034e-1},
      {-9.999715e-1,	7.280732e-4,	7.523790e-3,	-1.969266e-3},
      {-6.166020e-4,	-9.998901e-1,	1.480755e-2,	-7.228590e-2},
      {0,0,0,1}};
    */

    //2011_09_29
    double R00_inv[4][4] =  
    {{ 9.999477623386955e-01,    -9.806938410737948e-03,     2.873828123230253e-03,                         0},
     { 9.791706986274263e-03,     9.999381759546567e-01,     5.267133659010349e-03,                         0},
     {-2.925304766217524e-03,    -5.238719207215431e-03,     9.999819980894601e-01,                         0},
     {                    0,                         0,                         0,                         1}};
    double T_inv[4][4] = 
    {{ 7.755448452452784e-03,     2.294055666534548e-03,     9.999672892532474e-01,     2.672341673242784e-01},
     {-9.999694232695880e-01,     1.032122841965585e-03,     7.753097642421263e-03,    -5.139645452231175e-03},
     {-1.014302253796026e-03,    -9.999968720211324e-01,     2.301990468729539e-03,    -6.263302502186824e-02},
     {                     0,                         0,                         0,                         1}};

    /*
    //2011_10_03
    double R00_inv[4][4] =      
    {{  9.999453584199238e-1,  -7.292213176878839e-3,   7.487471393066381e-3,                         0},
     {  7.259128689002590e-3,   9.999638227681424e-1,   4.436324006246385e-3,                         0},
     { -7.519550710547824e-3,  -4.381729338932304e-3,   9.999621553701089e-1,                         0},
     {                     0,                      0,                      0,                         1}};
    double T_inv[4][4] = 
    {{  7.967513518807133e-3,  -2.771052653220483e-3,  9.999644387247446e-1,   2.918047200764812e-1},
     { -9.999679015949287e-1,   8.241709364607878e-4,  7.969825802652784e-3,  -1.140550664859292e-2},
     { -8.462265018155325e-4,  -9.999958419909836e-1, -2.764397568989711e-3,  -5.623941263833525e-2},
     {                   0  ,                      0,                     0,                      1}};
    */
    double fu, fv, cu, cv, bx, by;

  public:
    int row_of_depth_image, col_of_depth_image;//深度图行列数

    pcl::PointCloud<pcl::PointXYZ>::Ptr triangles_cloud_ptr;
    visualization_msgs::Marker pub_surface,pub_line;

    OrganizeTool();
    ~OrganizeTool();

    inline bool available_point(pcl::PointXYZ p);
    inline bool available_point(pcl::PointXYZI p);
    inline bool available_point(Point3 p);

    void read_calib(std::string calib_file);
    void order(pcl::PointCloud<pcl::PointXYZI>::Ptr unorganised_cloud_ptr,
               pcl::PointCloud<pcl::PointXYZI>::Ptr organised_cloud_ptr);
    void showOneRay(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr);
    float compute_alpha(pcl::PointXYZI a, pcl::PointXYZI b);
    void ground_remove_bfs(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud_ptr);
    void denser(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr,
                           double *depth_image);
    void get_pseudo(double *depth_image,pcl::PointCloud<pcl::PointXYZI>::Ptr pseudo_cloud_ptr);
    void surface_build(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, pcl::PolygonMesh::Ptr triangles_ptr);
    void eraseNAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
    void eraseNAN(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
    void eraseOutlier(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
    void matrix_dot(double *A, double *X);
    Point3 From2Dto3D(int v, int u, double d);
    pcl::PointXYZI find_exact_coord(pcl::PolygonMesh::Ptr triangles_ptr, int v, int u);
    double compute_distance(pcl::PointXYZI lhs, pcl::PointXYZI rhs);
    void test_view(pcl::PolygonMesh::Ptr triangles_ptr);
};

class PclTestCore
{
  private:
    ros::Subscriber sub_point_cloud_;      //为接收点云信息创建了一个订阅节点

    ros::Publisher pub_original_points_;  //创建了一个发布原点云的节点
    //ros::Publisher pub_organised_points_; //创建了一个发布有序化点云的节点
    ros::Publisher pub_ground_points_;    //创建了一个发布地面点云的节点
    ros::Publisher pub_object_points_;    //创建了一个发布非地面点云的节点
    ros::Publisher pub_pseudo_points_;    //创建了一个发布伪点云的节点
    ros::Publisher pub_dense_points_;     //创建了一个发布稠密点云的节点    
    //ros::Publisher pub_triangle_points_;  //创建了一个发布triangle点云的节点
    ros::Publisher pub_surface_polygon_;     //创建了一个发布三角形面的节点

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
//void point_cb是声明一个函数，这里面设置了一个数据类型为sensor_msgs::PointCloud2ConstPtr& in_cloud形参，const在这里修饰函数中的参数。将点云格式sensor_mgs/pointcloud2转换为pcl/pointcloud

    static int fileNameFilter_bin(const struct dirent *cur);
    static int fileNameFilter_txt(const struct dirent *cur);
    void workFromFile();
    void test_one_frame();

    OrganizeTool OrgTool;

  public:
    bool is_Read_From_File = true;
    bool is_test_one_frame = true;

    PclTestCore(ros::NodeHandle &nh);    
    ~PclTestCore();
    void Spin();
};