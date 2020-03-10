#include "submission.h"

#include "EnhancedLiDAR_core.h"

#include "geometry.h"

int main(int argc, char **argv)   //main函数，节点入口
{
    if(argv[1][0]=='r') //rviz 可视化
    {
        ros::init(argc, argv, "pcl_test"); //初始化节点，第三个参数  node_name，节点参数

        ros::NodeHandle nh;    //nh每个节点都对应一个句柄，实例化节点

        PclTestCore core(nh); //没查到，反正与后续的节点启动有关吧
    }
    else if(argv[1][0]=='f') //file 输出
    {
        submission();
    }
    else
        printf("[ Args Error ! ]\n");
    return 0;
}