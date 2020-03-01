#include "EnhancedLiDAR_core.h"
 
int main(int argc, char **argv)   //main函数，节点入口
{
    ros::init(argc, argv, "pcl_test");   //初始化节点，第三个参数  node_name，节点参数

    ros::NodeHandle nh;    //nh每个节点都对应一个句柄，实例化节点？

    PclTestCore core(nh); //   没查到，反正与后续的节点启动有关吧

    return 0;
}