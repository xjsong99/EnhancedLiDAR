#define Pii std::pair<int,int>
#include <utility>
#include <algorithm>
#include <dirent.h>
#include <string>

#include "EnhancedLiDAR_core.h"
#include "ground_remove_RANSAC.h"


OrganizeTool::OrganizeTool()
{
    radius = 3;//KDTree搜索半径
    max_nn = 2000;//KDTree搜索点数
    zero_alpha_threshold = 60;//最下面一行的点 地面判断角度阈值
    alpha_threshold = 10;//除最下面一行外 地面判断角度阈值

    row_of_depth_image = 375;
    col_of_depth_image = 1242;

    /*
    //VLP-16
    row_size=16;
    col_size=3600;
    */

    /*
    //HDL-32E
    row_size=32;
    col_size=2250;
    */

    //HDL-64线
    row_size = 64;
    col_size = 4600;// 360/0.08=4500
}

OrganizeTool::~OrganizeTool(){}

void OrganizeTool::read_calib(std::string calib_file)
{
    FILE *cam_to_cam;
    cam_to_cam = fopen((calib_file + std::string("calib_cam_to_cam.txt")).c_str(), "r");

    char buf[1001];

    //位置指针移动到P_rect_02所在行
    for (int i = 1; i <= 25; i++)
        fgets(buf, 1000, cam_to_cam);

    //位置指针向后移动11个字符
    fseek(cam_to_cam, 11, SEEK_CUR);

    double a;
    int b;
    char flag;

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);
    fu = a * pow(10, flag == '-' ? -b : b);

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);
    cu = a * pow(10, flag == '-' ? -b : b);

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);
    bx = a * pow(10, flag == '-' ? -b : b) / (-fu);

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);
    fv = a * pow(10, flag == '-' ? -b : b);

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);
    cv = a * pow(10, flag == '-' ? -b : b);

    fscanf(cam_to_cam, "%[^e]", buf);
    fscanf(cam_to_cam, "e%c", &flag);
    fscanf(cam_to_cam, "%d",&b);
    sscanf(buf, "%lf", &a);
    by = a * pow(10, flag == '-' ? -b : b) / (-fv);

    fclose(cam_to_cam);

    return;
}

bool OrganizeTool::available_point(pcl::PointXYZ p)
{
    return !std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z);
}
bool OrganizeTool::available_point(pcl::PointXYZI p)
{
    return !std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z) && !std::isnan(p.intensity);
}
bool OrganizeTool::available_point(Point3 p)
{
    return !std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z);
}

void OrganizeTool::eraseNAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    for (pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud_ptr->points.begin(); iter != cloud_ptr->points.end();)
    {
        if(std::isnan(iter->x)||std::isnan(iter->y)||std::isnan(iter->z))
            iter = cloud_ptr->points.erase(iter);
        else
            iter++;
    }
    return;
}
void OrganizeTool::eraseNAN(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    for (pcl::PointCloud<pcl::PointXYZI>::iterator iter = cloud_ptr->points.begin(); iter != cloud_ptr->points.end();)
    {
        if(std::isnan(iter->x)||std::isnan(iter->y)||std::isnan(iter->z)||std::isnan(iter->intensity))
            iter = cloud_ptr->points.erase(iter);
        else
            iter++;
    }
    return;
}

void OrganizeTool::surface_build(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, pcl::PolygonMesh::Ptr triangles_ptr)
{
    //eraseNAN(input_cloud_ptr);//剔除NAN点

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;                        //法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//储存估计的法线
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//kdtree指针
    tree->setInputCloud(input_cloud_ptr);

    n.setInputCloud(input_cloud_ptr);
    n.setSearchMethod(tree);
    n.setKSearch(10);//计算点云法向量时，搜索的点的个数(在计算点的法线时,设置邻域内需要多少点来模拟平面计算法线)
    
    n.compute(*normals);//将估计法线储存到normals中

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*input_cloud_ptr,*normals,*cloud_with_normals);//*cloud_with_normals = *input_cloud_ptr + *normals

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
    
    // 设置各参数值
    gp3.setSearchRadius (2);                //设置连接点之间的最大距离，即三角形最大边长
    gp3.setMu (2.5);                          //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
    gp3.setMaximumNearestNeighbors (100);   //设置样本点可搜索的邻域个数
    gp3.setMaximumSurfaceAngle(M_PI/4);     //设置某点法线方向偏离样本点法线的最大角度45
    gp3.setMinimumAngle(M_PI/18);           //设置三角化后得到的三角形内角的最小的角度为10
    gp3.setMaximumAngle(2*M_PI/3);          //设置三角化后得到的三角形内角的最大角度为120
    gp3.setNormalConsistency(false);        //设置该参数保证法线朝向一致

    // Get result
    gp3.setInputCloud (cloud_with_normals); //设置输入点云为有向点云
    gp3.setSearchMethod (tree2);            //设置搜索方式
    gp3.reconstruct (*triangles_ptr);         //重建提取三角化

    /*
    // 附加顶点信息
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    */
    
    return;
}
void OrganizeTool::order(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr)
{
    output_cloud_ptr->resize(row_size * col_size);
    output_cloud_ptr->height = row_size;
    output_cloud_ptr->width = col_size;
    for (int i = 0; i < row_size; i++)
        for (int j = 0; j < col_size; j++)
            output_cloud_ptr->at(j, i).x = output_cloud_ptr->at(j, i).y = output_cloud_ptr->at(j, i).z = output_cloud_ptr->at(j, i).intensity = NAN;

    int output_size = 0;

    double pre_angle = 0;
    int rowIndex = 0;
    double horAnglePerPT = 360.0 / col_size;

    int num = input_cloud_ptr->size();
    for (int i = 0; i < num; i++)
    {
        double HorizonRad = atan2(input_cloud_ptr->points.at(i).y, input_cloud_ptr->points.at(i).x);
        double HorizonAngle = HorizonRad * 180 / M_PI;
        double Delta_angle = std::fabs((HorizonAngle < 0 ? HorizonAngle + 360 : HorizonAngle) - (pre_angle < 0 ? pre_angle + 360 : pre_angle));

        if((pre_angle <0 && HorizonAngle >0) || Delta_angle>100)
            rowIndex++;
        pre_angle = HorizonAngle;
        
        HorizonAngle += 180;
        int colIndex = int(HorizonAngle / horAnglePerPT);
        if(rowIndex >= row_size)
            printf("! Error: rowIndex > row_size\n");
        rowIndex = std::min(rowIndex, row_size - 1);
        if (!available_point(output_cloud_ptr->at(colIndex, rowIndex)))
        //if (std::isnan(output_cloud_ptr->at(colIndex, rowIndex).x) &&
        //    std::isnan(output_cloud_ptr->at(colIndex, rowIndex).y) &&
        //    std::isnan(output_cloud_ptr->at(colIndex, rowIndex).z) &&
        //    std::isnan(output_cloud_ptr->at(colIndex, rowIndex).intensity))
        {
            output_size++;
            output_cloud_ptr->at(colIndex, rowIndex) = input_cloud_ptr->points.at(i);
        }
        else if(output_cloud_ptr->at(colIndex, rowIndex).x*output_cloud_ptr->at(colIndex, rowIndex).x+output_cloud_ptr->at(colIndex, rowIndex).y*output_cloud_ptr->at(colIndex, rowIndex).y+output_cloud_ptr->at(colIndex, rowIndex).z*output_cloud_ptr->at(colIndex, rowIndex).z
              >input_cloud_ptr->points.at(i).x*input_cloud_ptr->points.at(i).x+input_cloud_ptr->points.at(i).y*input_cloud_ptr->points.at(i).y+input_cloud_ptr->points.at(i).z*input_cloud_ptr->points.at(i).z)
            output_cloud_ptr->at(colIndex, rowIndex) = input_cloud_ptr->points.at(i);
    }
    printf("rowIndex=%d\n",rowIndex);
    printf("input_size=%d, output_size=%d\n", int(input_cloud_ptr->size()), output_size);
    
    /*
    //绘制点云深度剖开图
	Mat imgshow(row_size, col_size, CV_8UC1, Scalar(0,0,0));
    pcl::PointXYZI p;
    for (size_t i = 0; i < imgshow.rows; i++)
        for (size_t j = 0; j < imgshow.cols; j++)
		{
            p = output_cloud_ptr->at(j, i);
            double d;
            if(p.x == NAN && p.y == NAN && p.z == NAN)
                d = 0;
            else
            {
                d = sqrt(p.x * p.x + p.y * p.y + p.z * p.z) * 2.5;
                d = d > 250 ? 250 : d; //因为点云距离一般在100米内，因此在这里做映射，使得图像看起来明显点
            }
            imgshow.data[i * imgshow.step + j] = d;
        }
    imshow("", imgshow);
    waitKey(0);
	*/
    return;
}
void OrganizeTool::showOneRay(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr)
{
    srand(time(0));
    int rowIndex = rand() % row_size;
    for (int i = 0; i < row_size; i++)
        if(i != rowIndex) for (int j = 0; j < col_size; j++)
                input_cloud_ptr->at(j, i).x = input_cloud_ptr->at(j, i).y = input_cloud_ptr->at(j, i).z = input_cloud_ptr->at(j, i).intensity = NAN;
    return;
}
float OrganizeTool::compute_alpha(pcl::PointXYZI a,pcl::PointXYZI b)
{
    float delta_z = b.z - a.z;
    float delta_x = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    return fabs(atan2(delta_z, delta_x)) * 180 / M_PI;
}
void OrganizeTool::ground_remove(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud_ptr)
{
    pcl::copyPointCloud(*input_cloud_ptr, *object_cloud_ptr);

    bool vis[row_size][col_size];
    memset(vis, 0, sizeof(vis));

    std::queue<Pii> Q;
    while(!Q.empty())
        Q.pop();

    int x, y;

    for (int j = 0; j < col_size; j++)
    {
        x = row_size - 1;

        if (available_point(input_cloud_ptr->at(j, x)) && available_point(input_cloud_ptr->at(j, x-1)) && compute_alpha(input_cloud_ptr->at(j, x), input_cloud_ptr->at(j, x-1)) < zero_alpha_threshold)
        {
            vis[x][j] = true;
            Q.push(Pii(x, j));
            ground_cloud_ptr->push_back(input_cloud_ptr->at(j,x));
            object_cloud_ptr->at(j, x).x = object_cloud_ptr->at(j, x).y = object_cloud_ptr->at(j, x).z = object_cloud_ptr->at(j, x).intensity = NAN;
        }
    }

    while(!Q.empty())
    {
        x = Q.front().first, y = Q.front().second;
        Q.pop();
        int tx, ty;
        for (int i = 0; i < 8; i++)
        {
            tx = x + dx[i];
            ty = y + dy[i];
            if(tx < 0 || tx >= row_size || ty < 0 || ty >= col_size || vis[tx][ty])
                continue;
            if (available_point(input_cloud_ptr->at(ty,tx)))
                printf("%.0f ",compute_alpha(input_cloud_ptr->at(y,x),input_cloud_ptr->at(ty,tx)));
            if (available_point(input_cloud_ptr->at(ty,tx)) && compute_alpha(input_cloud_ptr->at(y,x),input_cloud_ptr->at(ty,tx)) < alpha_threshold)
            {
                vis[tx][ty] = true;
                Q.push(Pii(tx, ty));
                ground_cloud_ptr->push_back(input_cloud_ptr->at(ty,tx));
                object_cloud_ptr->at(ty, tx).x = object_cloud_ptr->at(ty, tx).y = object_cloud_ptr->at(ty, tx).z = object_cloud_ptr->at(ty, tx).intensity = NAN;
            }
        }
    }

    return;
}
void OrganizeTool::denser(pcl::PointCloud<pcl::PointXYZI>::Ptr organised_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr)
{
    //调整并复制原有点云
    output_cloud_ptr->points.clear();
    output_cloud_ptr->height = 2 * organised_cloud_ptr->height;
    output_cloud_ptr->width = organised_cloud_ptr->width;
    output_cloud_ptr->resize( output_cloud_ptr->height * output_cloud_ptr->width );
    for (int i = 0; i < organised_cloud_ptr->height;i++)
        for (int j = 0; j < organised_cloud_ptr->width;j++)
            output_cloud_ptr->points.at(2 * i * col_size + j) = organised_cloud_ptr->points.at(i * col_size + j);
    
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    std::vector<int> indices;//记录搜索到的点的下标
    std::vector<float> sqr_distance; //记录搜索到的点的距离
    //pcl::PointCloud<pcl::PointXYZI>::Ptr valid_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::removeNaNFromPointCloud(*organised_cloud_ptr,*valid_cloud_ptr,indices);//剔除NAN点
    //kdtree.setInputCloud(valid_cloud_ptr); //设置kdtree对应的点云
    kdtree.setInputCloud(organised_cloud_ptr); //设置kdtree对应的点云
    int return_nn;//实际返回的搜索到的点数

    pcl::PointXYZI search_point;//搜索的中心点
    pcl::PointXYZI predicted_point;
    pcl::PointXYZ tmp_Point;
    for (int i = 0; i < organised_cloud_ptr->height; i+=1000)//由于计算速度的原因，隔1000行/1000列做一次最近点提取和曲面重建
        for (int j = 0; j < organised_cloud_ptr->width;j+=1000)
        {
            search_point = organised_cloud_ptr->points.at(i * col_size + j);
            if(!available_point(search_point)) continue;
            //printf("(%lf,%lf,%lf)\n",search_point.x,search_point.y,search_point.z);
            return_nn = kdtree.radiusSearch(search_point, radius, indices, sqr_distance, max_nn);
            if(return_nn)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//将搜索到的点复制到这个点云中
                predicted_point.x = 0;predicted_point.y = 0;predicted_point.z = 0;predicted_point.intensity = 0;
                for (int k = 0; k < return_nn; k++)
                {
                    tmp_Point.x=organised_cloud_ptr->points.at(indices[k]).x;
                    tmp_Point.y=organised_cloud_ptr->points.at(indices[k]).y;
                    tmp_Point.z=organised_cloud_ptr->points.at(indices[k]).z;
                    nearest_point_cloud_ptr->push_back(tmp_Point);
                    predicted_point.x += organised_cloud_ptr->points.at(indices[k]).x;
                    predicted_point.y += organised_cloud_ptr->points.at(indices[k]).y;
                    predicted_point.z += organised_cloud_ptr->points.at(indices[k]).z;
                    predicted_point.intensity += organised_cloud_ptr->points.at(indices[k]).intensity;
                }
                predicted_point.x /= return_nn;
                predicted_point.y /= return_nn;
                predicted_point.z /= return_nn;
                predicted_point.intensity /= return_nn;

                //surface_build(nearest_point_cloud_ptr);

                output_cloud_ptr->points.at((2 * i + 1) * col_size + j) = predicted_point;
            }
        }
    return;
}
double OrganizeTool::compute_distance(pcl::PointXYZI lhs,pcl::PointXYZI rhs)
{
    return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y) + (lhs.z - rhs.z) * (lhs.z - rhs.z));
}
Point3 OrganizeTool::From2Dto3D(int v, int u, double d)
{
    double axis[4][1];
    Point3 p;

    //project_image_to_rect
    axis[0][0] = (u - cu) * d / fu + bx;
    axis[1][0] = (v - cv) * d / fv + by;
    axis[2][0] = d;
    axis[3][0] = 1;

    //project_rect_to_ref
    matrix_dot(&R00_inv[0][0], &axis[0][0]);

    //project_ref_to_velo
    matrix_dot(&T_inv[0][0], &axis[0][0]);
    
    p.x = axis[0][0];
    p.y = axis[1][0];
    p.z = axis[2][0];

    return p;
}
pcl::PointXYZI OrganizeTool::find_exact_coord(pcl::PolygonMesh::Ptr triangles_ptr, int v, int u)
{
    Line3 ray(From2Dto3D(v, u, 10), From2Dto3D(v, u, 100)); //像素(v,u)对应的雷达坐标空间中的射线

    pcl::PointCloud<pcl::PointXYZ>::Ptr triangles_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    Point3 triangle_Point3[3], intersection, final_intersection;

    int num_of_triangles = triangles_ptr->polygons.size();//统计三角形个数
    int dis_to_origin_2 = INT32_MAX;//初始final_intersection没有记录点，离原点距离的平方设为int32_max

    //点云格式转换
    pcl::fromPCLPointCloud2(triangles_ptr->cloud, *triangles_cloud_ptr);

    for (int i = 0; i < num_of_triangles; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            triangle_Point3[j].x = triangles_cloud_ptr->points.at(triangles_ptr->polygons.at(i).vertices.at(j)).x;
            triangle_Point3[j].y = triangles_cloud_ptr->points.at(triangles_ptr->polygons.at(i).vertices.at(j)).y;
            triangle_Point3[j].z = triangles_cloud_ptr->points.at(triangles_ptr->polygons.at(i).vertices.at(j)).z;
        }
        Plane triangle_plane(triangle_Point3[0], triangle_Point3[1], triangle_Point3[2]);

        if(triangle_plane.crossline(ray, intersection) == 1
           && triangle_plane.PointInTriangle(intersection) 
           && intersection*intersection<dis_to_origin_2)
        //射线与三角平面有交点 且 交点在三角形内 且 距离比之前记录的点离原点近
        {
            final_intersection = intersection;
            dis_to_origin_2 = intersection * intersection;
        }
    }
    
    pcl::PointXYZI return_Point(0.f);//intensity=0.f
    if(dis_to_origin_2!=INT32_MAX)
    {
        return_Point.x = final_intersection.x;
        return_Point.y = final_intersection.y;
        return_Point.z = final_intersection.z;
        return return_Point;
    }
    else
    {
        return_Point.x = return_Point.y = return_Point.z = 0;
        return return_Point;
    }
}
void OrganizeTool::test_view(pcl::PolygonMesh::Ptr triangles_ptr)
{
    printf("size=%d\n",(int)(triangles_ptr->polygons.size()));

    //输出triangle点云
    ros::Publisher pub_triangle_points_;     //创建了一个发布triangle点云的节点
    sensor_msgs::PointCloud2 pub_triangle;   //声明输出的点云格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr triangles_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(triangles_ptr->cloud, *triangles_cloud_ptr);
    pcl::toROSMsg(*triangles_cloud_ptr, pub_triangle);
    pub_triangle.header.frame_id = "/velodyne";
    pub_triangle.header.stamp = ros::Time::now();
    pub_triangle_points_.publish(pub_triangle);

    sleep(10);

    /*
    //图形显示模块
    //创建显示对象指针
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  //设置窗口颜色
    viewer->addPolygonMesh(*triangles_ptr, "triangles");  //设置所要显示的网格对象
    viewer->addCoordinateSystem(1);  //设置坐标系,参数为坐标显示尺寸
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    */
    return;
}
void OrganizeTool::denser_with_depth(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr,
                                     double *depth_image)
{
    //pcl::copyPointCloud(*input_cloud_ptr, *output_cloud_ptr);//不需清空output，直接覆盖

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    std::vector<int> indices;//记录搜索到的点的下标
    std::vector<float> sqr_distance; //记录搜索到的点的距离
    kdtree.setInputCloud(input_cloud_ptr); //设置kdtree对应的点云
    
    int return_nn; //实际返回的搜索到的点数

    pcl::PointXYZI search_point;//搜索的中心点
    search_point.intensity = 0;

    pcl::PointXYZI last_search_point;//上一个搜索的中心点
    last_search_point.x = last_search_point.y = last_search_point.z = last_search_point.intensity = 0;

    pcl::PointXYZI predicted_point;
    pcl::PointXYZ tmp_Point;

    pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_point_cloud_ptr;//提取的临近点

    Point3 temp_Point;

    pcl::PolygonMesh::Ptr triangles_ptr(new pcl::PolygonMesh);//曲面重建完后的三角形
    triangles_ptr->polygons.clear();

    int test_num[5]={0,0,0,0,0};

    for (int v = 0; v < row_of_depth_image; v += 5)//间隔取点，原因：计算速度太慢&有些点距离过近
        for (int u = 0; u < col_of_depth_image;u += 2)
        {
            temp_Point = From2Dto3D(v, u, *(depth_image + v * col_of_depth_image + u));

            if(!available_point(temp_Point)) continue;

            search_point.x = temp_Point.x;
            search_point.y = temp_Point.y;
            search_point.z = temp_Point.z;

            //printf("(%lf,%lf,%lf)\n",search_point.x,search_point.y,search_point.z);


            /*//使用平均值的计算方法
            return_nn = kdtree.radiusSearch(search_point, radius, indices, sqr_distance, max_nn);
            //printf("(u,v)=(%d,%d) return_nn=%d\n",u,v,return_nn);
            
            if (return_nn)
            {
                //printf("%d ",return_nn);

                predicted_point.x = 0;
                predicted_point.y = 0;
                predicted_point.z = 0;
                predicted_point.intensity = 0;
                for (int k = 0; k < return_nn; k++)
                {
                    predicted_point.x += input_cloud_ptr->points.at(indices[k]).x;
                    predicted_point.y += input_cloud_ptr->points.at(indices[k]).y;
                    predicted_point.z += input_cloud_ptr->points.at(indices[k]).z;
                    //predicted_point.intensity += input_cloud_ptr->points.at(indices[k]).intensity;
                }
                predicted_point.x /= return_nn;
                predicted_point.y /= return_nn;
                predicted_point.z /= return_nn;
                //predicted_point.intensity /= return_nn;

                output_cloud_ptr->push_back(predicted_point);
            }
            */

            test_num[0]++;

            //使用曲面重建的计算方法
            if(compute_distance(search_point,last_search_point)>radius)//与上一次搜索中心点的距离超过kdtree搜索半径,重建曲面
            {
                test_num[1]++;

                last_search_point = search_point;

                nearest_point_cloud_ptr = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;//新建近邻点点云

                triangles_ptr = (pcl::PolygonMesh::Ptr) new pcl::PolygonMesh;//新建曲面三角形

                return_nn = kdtree.radiusSearch(search_point, radius, indices, sqr_distance, max_nn);
                for (int k = 0; k < return_nn; k++)
                {
                    tmp_Point.x=input_cloud_ptr->points.at(indices[k]).x;
                    tmp_Point.y=input_cloud_ptr->points.at(indices[k]).y;
                    tmp_Point.z=input_cloud_ptr->points.at(indices[k]).z;
                    nearest_point_cloud_ptr->push_back(tmp_Point);
                }

                triangles_ptr->polygons.clear();

                if(nearest_point_cloud_ptr->size() >= 3)//点云中的点数>=3,可以组成三角形
                    surface_build(nearest_point_cloud_ptr, triangles_ptr),test_num[2]++;
            }

            if(triangles_ptr->polygons.size() > 0)
            {
                test_num[3]++;

                if(triangles_ptr->polygons.size() > 300) test_view(triangles_ptr);

                predicted_point = find_exact_coord(triangles_ptr, v, u);
                if(predicted_point.x!=0 || predicted_point.y!=0 || predicted_point.z!=0)
                    output_cloud_ptr->push_back(predicted_point),test_num[4]++;
            }
        }

    for (int i = 0; i < 5;i++)
        printf("test_num[%d]=%d ",i,test_num[i]);
    printf("\n");
    return;
}
void OrganizeTool::matrix_dot(double *A,double *X)
{
    double Y[4][1];
    memset(Y,0,sizeof(Y));
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4;j++)
            Y[i][0] += (*(A+i*4+j)) * (*(X+j));
    memcpy(X,Y,sizeof(Y));
    return;
}
void OrganizeTool::get_pseudo(double *depth_image,pcl::PointCloud<pcl::PointXYZI>::Ptr pseudo_cloud_ptr)
{
    pcl::PointXYZI search_point;//搜索的中心点
    search_point.intensity = 0;
    pcl::PointXYZ tmp_Point;
    Point3 tmp_Point3;

    double axis[4][1], *z_Caxis = depth_image;
    for (int v = 0; v < row_of_depth_image; v++)
        for (int u = 0; u < col_of_depth_image;u++)
        {
            tmp_Point3 = From2Dto3D(v, u, *(z_Caxis++));

            search_point.x = tmp_Point3.x;
            search_point.y = tmp_Point3.y;
            search_point.z = tmp_Point3.z;

            pseudo_cloud_ptr->push_back(search_point);

        }

    return;
}

//构造函数
PclTestCore::PclTestCore(ros::NodeHandle &nh){

    //sub_point_cloud_ = nh.subscribe("/velodyne_points",10, &PclTestCore::point_cb, this);//VLP实时点云
    //sub_point_cloud_ = nh.subscribe("/kitti/velo/pointcloud",10, &PclTestCore::point_cb, this);//kitti数据集点云

    pub_original_points_= nh.advertise<sensor_msgs::PointCloud2>("/original_points", 10);
    pub_organised_points_= nh.advertise<sensor_msgs::PointCloud2>("/organised_points", 10);
    pub_pseudo_points_ = nh.advertise<sensor_msgs::PointCloud2>("/pseudo_points", 10);
    pub_ground_points_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10);
    pub_object_points_ = nh.advertise<sensor_msgs::PointCloud2>("/object_points", 10);
    pub_dense_points_ = nh.advertise<sensor_msgs::PointCloud2>("/dense_points", 10);

    if(is_test_one_frame == true)
        test_one_frame();
    else if (is_Read_From_File == true)
        workFromFile();

    ros::spin(); //回调函数
}
 
PclTestCore::~PclTestCore(){}
void PclTestCore::Spin(){}
 
void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){

    if(is_Read_From_File == true)
        return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr original_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr organized_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr dense_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *original_pc_ptr);

    //OrgTool.transform(original_pc_ptr,organized_pc_ptr);//点云有序化
    //OrgTool.order(original_pc_ptr,organized_pc_ptr);

    OrgTool.denser(organized_pc_ptr, dense_pc_ptr);//点云稠密化

    //输出有序化organised点云
    sensor_msgs::PointCloud2 pub_organised;   //声明输出的点云格式
    pcl::toROSMsg(*organized_pc_ptr, pub_organised);
    pub_organised.header = in_cloud_ptr->header;
    pub_organised_points_.publish(pub_organised);

    //输出稠密化denser点云
    sensor_msgs::PointCloud2 pub_dense;
    pcl::toROSMsg(*dense_pc_ptr, pub_dense);
    pub_dense.header = in_cloud_ptr->header;
    pub_dense_points_.publish(pub_dense);

    return;
}
int PclTestCore::fileNameFilter_bin(const struct dirent *cur)
{
    std::string str(cur->d_name);
    if(str.find(".bin")!=std::string::npos)
        return 1;
    return 0;
}
int PclTestCore::fileNameFilter_txt(const struct dirent *cur)
{
    std::string str(cur->d_name);
    if(str.find(".txt")!=std::string::npos)
        return 1;
    return 0;
}
void PclTestCore::workFromFile()
{
    float *data = (float*)malloc(1000000*sizeof(float));
    FILE *stream;

    std::string cloud_file = "/media/song/程序磁盘/mouse/PSMNet/2011_10_03/2011_10_03_drive_0047_sync/velodyne_points/data/";
    std::string depth_file = "/media/song/程序磁盘/mouse/PSMNet/2011_10_03/2011_10_03_drive_0047_sync/predicted_depth/";
    std::string calib_file = "/media/song/程序磁盘/mouse/PSMNet/2011_10_03/";

    OrgTool.read_calib(calib_file);

    struct dirent **namelist_bin,**namelist_txt;//文件名list

    int num_of_file = std::min(scandir(cloud_file.c_str(), &namelist_bin, fileNameFilter_bin, alphasort),
                               scandir(depth_file.c_str(), &namelist_txt, fileNameFilter_txt, alphasort));

    printf("num_of_file=%d\n",num_of_file);

    for (int file_index = 0; file_index < num_of_file; file_index++)
    {

        printf("file_index=%d\n",file_index);

        pcl::PointCloud<pcl::PointXYZI>::Ptr original_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        //pcl::PointCloud<pcl::PointXYZI>::Ptr organized_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr object_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr pseudo_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr dense_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        double depth_image[375][1242];

        // allocate 4 MB buffer (only ~130*4*4 KB are needed)
        int num = 1000000;

        stream = fopen ((cloud_file+std::string(namelist_bin[file_index]->d_name)).c_str(),"rb");
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


        stream = fopen ((depth_file+std::string(namelist_txt[file_index]->d_name)).c_str(),"r");
        for (int i = 0; i < 375;i++)
            for (int j = 0; j < 1242;j++)
                fscanf(stream,"%lf",&depth_image[i][j]);
        fclose(stream);

        //OrgTool.order(original_pc_ptr,organized_pc_ptr);

        //OrgTool.showOneRay(organized_pc_ptr); //随机标出一条激光扫描到的点，若使用此函数请注释掉下面ground_remove、get_pseudo、denser_with_depth三个函数的调用

        //OrgTool.ground_remove(organized_pc_ptr, ground_pc_ptr, object_pc_ptr);
        ground_remove_RANSAC(original_pc_ptr, ground_pc_ptr, object_pc_ptr);

        OrgTool.get_pseudo(&depth_image[0][0], pseudo_pc_ptr);

        OrgTool.denser_with_depth(original_pc_ptr, dense_pc_ptr, &depth_image[0][0]);//用深度进行点云稠密化

        //输出原点云
        sensor_msgs::PointCloud2 pub_original;   //声明输出的点云格式
        pcl::toROSMsg(*original_pc_ptr, pub_original);
        pub_original.header.frame_id = "/velodyne";
        pub_original.header.stamp = ros::Time::now();
        pub_original_points_.publish(pub_original);

        
        /*
        //输出有序点云
        sensor_msgs::PointCloud2 pub_organised;   //声明输出的点云格式
        pcl::toROSMsg(*organized_pc_ptr, pub_organised);
        pub_organised.header.frame_id = "/velodyne";
        pub_organised.header.stamp = ros::Time::now();
        pub_organised_points_.publish(pub_organised);
        */

        //输出地面点云
        sensor_msgs::PointCloud2 pub_ground;   //声明输出的点云格式
        pcl::toROSMsg(*ground_pc_ptr, pub_ground);
        pub_ground.header.frame_id = "/velodyne";
        pub_ground.header.stamp = ros::Time::now();
        pub_ground_points_.publish(pub_ground);

        //输出非地面点云
        sensor_msgs::PointCloud2 pub_object;   //声明输出的点云格式
        pcl::toROSMsg(*object_pc_ptr, pub_object);
        pub_object.header.frame_id = "/velodyne";
        pub_object.header.stamp = ros::Time::now();
        pub_object_points_.publish(pub_object);

        //输出伪点云
        sensor_msgs::PointCloud2 pub_pseudo;   //声明输出的点云格式
        pcl::toROSMsg(*pseudo_pc_ptr, pub_pseudo);
        pub_pseudo.header.frame_id = "/velodyne";
        pub_pseudo.header.stamp = ros::Time::now();
        pub_pseudo_points_.publish(pub_pseudo);

        //输出稠密化denser点云
        sensor_msgs::PointCloud2 pub_dense;   //声明输出的点云格式
        pcl::toROSMsg(*dense_pc_ptr, pub_dense);
        pub_dense.header.frame_id = "/velodyne";
        pub_dense.header.stamp = ros::Time::now();
        pub_dense_points_.publish(pub_dense);
    }

    return;
}
void PclTestCore::test_one_frame()
{
    float *data = (float*)malloc(1000000*sizeof(float));
    FILE *stream;

    
    std::string cloud_file = "/media/song/程序磁盘/mouse/PSMNet/2011_09_26/2011_09_26_drive_0101_sync/velodyne_points/data/";
    std::string depth_file = "/media/song/程序磁盘/mouse/PSMNet/2011_09_26/2011_09_26_drive_0101_sync/predicted_depth/";
    std::string calib_file = "/media/song/程序磁盘/mouse/PSMNet/2011_09_26/";
    
    /*
    std::string cloud_file = "/media/song/程序磁盘/mouse/PSMNet/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/";
    std::string depth_file = "/media/song/程序磁盘/mouse/PSMNet/2011_10_03/2011_10_03_drive_0042_sync/predicted_depth/";
    std::string calib_file = "/media/song/程序磁盘/mouse/PSMNet/2011_10_03/";
    */
    OrgTool.read_calib(calib_file);

    struct dirent **namelist_bin,**namelist_txt;//文件名list

    int num_of_file = std::min(scandir(cloud_file.c_str(), &namelist_bin, fileNameFilter_bin, alphasort),
                               scandir(depth_file.c_str(), &namelist_txt, fileNameFilter_txt, alphasort));

    printf("num_of_file=%d\n",num_of_file);

    int file_index = 914; //测试第几帧

    printf("file_index=%d\n",file_index);

    pcl::PointCloud<pcl::PointXYZI>::Ptr original_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr object_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pseudo_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr dense_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    double depth_image[375][1242];

    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int num = 1000000;

    stream = fopen ((cloud_file+std::string(namelist_bin[file_index]->d_name)).c_str(),"rb");
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

    stream = fopen ((depth_file+std::string(namelist_txt[file_index]->d_name)).c_str(),"r");
    for (int i = 0; i < 375;i++)
        for (int j = 0; j < 1242;j++)
            fscanf(stream,"%lf",&depth_image[i][j]);
    fclose(stream);

    ground_remove_RANSAC(original_pc_ptr, ground_pc_ptr, object_pc_ptr);

    OrgTool.get_pseudo(&depth_image[0][0], pseudo_pc_ptr);

    OrgTool.denser_with_depth(object_pc_ptr, dense_pc_ptr, &depth_image[0][0]);//用深度进行点云稠密化

    //输出原点云
    sensor_msgs::PointCloud2 pub_original;   //声明输出的点云格式
    pcl::toROSMsg(*original_pc_ptr, pub_original);
    pub_original.header.frame_id = "/velodyne";

    //输出地面点云
    sensor_msgs::PointCloud2 pub_ground;   //声明输出的点云格式
    pcl::toROSMsg(*ground_pc_ptr, pub_ground);
    pub_ground.header.frame_id = "/velodyne";

    //输出非地面点云
    sensor_msgs::PointCloud2 pub_object;   //声明输出的点云格式
    pcl::toROSMsg(*object_pc_ptr, pub_object);
    pub_object.header.frame_id = "/velodyne";
    
    //输出伪点云
    sensor_msgs::PointCloud2 pub_pseudo;   //声明输出的点云格式
    pcl::toROSMsg(*pseudo_pc_ptr, pub_pseudo);
    pub_pseudo.header.frame_id = "/velodyne";
    
    //输出稠密化denser点云
    sensor_msgs::PointCloud2 pub_dense;   //声明输出的点云格式
    pcl::toROSMsg(*dense_pc_ptr, pub_dense);
    pub_dense.header.frame_id = "/velodyne";

    time_t last_time=0;

    while(1)//保证屏幕连续显示此帧
    {
        if(time(NULL) - last_time > 1.5)
        {
            last_time = time(NULL);

            pub_original.header.stamp = ros::Time::now();
            pub_original_points_.publish(pub_original);

            pub_ground.header.stamp = ros::Time::now();
            pub_ground_points_.publish(pub_ground);

            pub_object.header.stamp = ros::Time::now();
            pub_object_points_.publish(pub_object);

            pub_pseudo.header.stamp = ros::Time::now();
            pub_pseudo_points_.publish(pub_pseudo);

            pub_dense.header.stamp = ros::Time::now();
            pub_dense_points_.publish(pub_dense);
        }
    }

    return;
}
