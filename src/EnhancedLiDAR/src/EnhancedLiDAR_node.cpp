#include "submission.h"

#include "EnhancedLiDAR_core.h"

#include "geometry.h"

int main(int argc, char **argv)
{
    if(argv[1][0]=='r') //rviz 可视化
    {
        ros::init(argc, argv, "pcl_test"); //初始化节点

        ros::NodeHandle nh;    //实例化节点

        PclTestCore core(nh);
    }
    else if(argv[1][0]=='f') //file 输出
    {
        int start_index;
        sscanf(argv[2], "%d", &start_index);
        submission(start_index);
    }
    else
        printf("[ Args Error ! ]\n");
    return 0;
}