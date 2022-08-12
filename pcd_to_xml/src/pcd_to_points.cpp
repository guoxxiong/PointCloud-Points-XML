#include<cstdio>
#include<iostream>
#include<cstdlib>
#include<vector>
#include<ctime>
#include<fstream>

#include <ros/ros.h>
//#include "std_msgs/String.h"

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/PointIndices.h>
#include<pcl/segmentation/extract_clusters.h>
#include<pcl/segmentation/impl/extract_clusters.hpp>
//#include<pcl_conversions/pcl_conversions.h>
//#include<sensor_msgs/PointCloud2.h>
//#include<pcl/common/transforms.h>
#include<pcl/segmentation/extract_clusters.h>
#include<pcl/visualization/cloud_viewer.h>

#define PCD_DIR "../pcd/"

typedef pcl::PointXYZRGB PointT;

pcl::visualization::CloudViewer viewer1("cluster cloud viewer");
//pcl::visualization::CloudViewer viewer2("original cloud viewer");

/*BEGIN_FUNCTION_HDR
*******************************************************************************************
*Function Name:del_samePoints
*Description:delete some points needless
*
*
*Inputs:vector
*
*
*Outputs:vector
*
*
*Limitations:
*******************************************************************************************
END_FUNCTION_HDR*/
//剔除在一个区间内数量较多的点
std::vector<double> del_samePoints(std::vector<double> vd)
{
    double revdarr[vd.size()];
    std::vector<double> revd;
    revd.clear();
    for (int k=0;k<vd.size();k++)
    {
        revdarr[k] = vd[k];
    }
    revd.push_back(revdarr[0]);
    //printf("vd.size():%lu\n",vd.size());
    for (int i=1;i<vd.size()-1;)
    { 
        int n = 0;
        int m = 1;
        while ((revdarr[i]-revdarr[i-1]) < (revdarr[vd.size()-1]-revdarr[0])*0.002) //根据坐标范围变化设置剔除尺度
        {
            revdarr[i] = revdarr[i+m];
            if ( (i+m) == (vd.size()-1) )
            { break; }
            n++;
            m++; 
        }
        revd.push_back(revdarr[i]);
        i = i+n+1;
    }
    return revd;
}


//pcd存取格式需按照阿拉伯数字加.pcd的格式，例如"1.pcd"，每一帧有一个pcd文件
std::vector< std::vector<PointT> > points_cluster(std::string pcd_dir,int index_start,int index_end)
{
    
    std::string pcd_ext = ".pcd";
    std::vector< std::vector<PointT> > retn;

    for (int j=index_start;j<index_end+1;j++)
    { 
        std::stringstream ss;//将j转为string
        ss << j;
        std::string str = ss.str();
        std::string pcd_index = pcd_dir + str + pcd_ext;//目标文件路径
        pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr newcloud(new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile (pcd_index,*cloud1);
        printf("文件序号：%d\n",j);
        PointT p;

        //对从文件中读取到的点云进行坐标转换，由相机坐标系转换到笛卡尔
        for (int i=0;i<cloud1->points.size();i++)
        {
            p.x = cloud1->points[i].x/1000;//点云坐标单位换算，从原来的mm换算为m
            p.y = cloud1->points[i].z/1000;
            p.z = -(cloud1->points[i].y)/1000;
            p.r = cloud1->points[i].r;
            p.g = cloud1->points[i].g;
            p.b = cloud1->points[i].b;
            //std::cout<<p.x<<","<<p.y<<","<<p.z<<","<<(ushort)p.r<<","<<(ushort)p.g<<","<<(ushort)p.b<<std::endl;
            newcloud->points.push_back(p);
        }

        //设置一个Kdtree来按顺序存储点云，方便后续遍历查找
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree-> setInputCloud(newcloud); //设置要使用kdtree的点云

        std::vector<pcl::PointIndices> cluster_indices; //被分割出来的点云团，标号队列
        pcl::EuclideanClusterExtraction<PointT> ec;  //欧几里得聚类算法方法

        ec.setClusterTolerance (1);        //设置近邻搜索的半径
        ec.setMinClusterSize(20);               //每一类的最少点数（可以用来取出孤立点）
        ec.setMaxClusterSize(20000);             //每一类的最大点数
        ec.setSearchMethod(tree);                //设置查找的tree
        ec.setInputCloud(newcloud);
        ec.extract(cluster_indices);            //将聚好的类存在cluster_indices

        //新建向量，存储一帧中心点的三个坐标，方便求点云团的几何质心
        std::vector<PointT> centerPointVector;
        std::vector<double> vx;
        std::vector<double> vy;
        std::vector<double> vz;
        std::vector<double> vxd;
        std::vector<double> vyd;
        std::vector<double> vzd;

        //循环访问每一个聚类
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end(); it++)
        {
            //将每一个聚类的点存入到临时的点云里
            pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            //printf("here1...\n");
            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            {
                cloud_cluster->points.push_back(newcloud->points[*pit]);
                //printf("here2...\n");
            }
            cloud_cluster->height = 1;
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->is_dense = true;

            //取出点云团的x,y,z坐标并排序和剔除
            for (int j=0;j<cloud_cluster->points.size();j++)
            {
                vx.push_back(cloud_cluster->points[j].x);
                vy.push_back(cloud_cluster->points[j].y);
                vz.push_back(cloud_cluster->points[j].z);
            }
            sort(vx.begin(),vx.end());
            vxd = del_samePoints(vx);
            vx.clear();
        
            sort(vy.begin(),vy.end());
            vyd = del_samePoints(vy);
            vy.clear();
    
            sort(vz.begin(),vz.end());
            vzd = del_samePoints(vz);
            vz.clear();
            //对已经均匀化的点云求坐标的中值
            PointT p1;
            p1.x = vxd[vxd.size()/2];
            p1.y = vyd[vyd.size()/2];
            p1.z = vzd[vzd.size()/2];
            p1.r = cloud_cluster->points[0].r;
            p1.g = cloud_cluster->points[0].g;
            p1.b = cloud_cluster->points[0].b;

            //清空本次vector释放空间
            vxd.clear();
            vyd.clear();
            vzd.clear();
            
            //将上述向量压入总向量（vector容器）
            centerPointVector.push_back(p1);
        }
        retn.push_back(centerPointVector);
    }
    return retn;//返回聚类完成并取完中心点的坐标vector
}

//与上一帧点云最近的点融合入一个vector
// vector< vector <PointT> > mix_vector(vector< vector<PointT> > frames_vector)
// {
//     PointT xp;
//     int counter = 1;
//     vector<PointT> vector1;
//     vector< vector<PointT> > mixVector;
//     int sz = frames_vector[0].size();
//     for (int i = 0;i < sz;i++)
//     {
//         xp.x = frames_vector[0][i].x;
//         xp.y = frames_vector[0][i].y;
//         xp.z = frames_vector[0][i].z;
//         xp.r = frames_vector[0][i].r;
//         xp.g = frames_vector[0][i].g;
//         xp.b = frames_vector[0][i].b;
//         vector1.push_back(xp);
//         mixVector.push_back(vector1);
//         vector1.clear();    
//     }
//     for (int i = 1;i < frames_vector.size();i++)
//     {
//          for (int j = 0;j < frames_vector[i].size();j++)
//          {
//              for (int k = 0;k < mixVector.size();k++)
//              {
//                  if (pow((pow(frames_vector[i][j].x-frames_vector[i-1][k].x,2)+pow(frames_vector[i][j].y-frames_vector[i-1][k].y,2)+pow(frames_vector[i][j].z-frames_vector[i-1][k].z,2)),0.5)<0.5) 
//                  {
//                     xp.x = frames_vector[i][j].x;
//                     xp.y = frames_vector[i][j].y;
//                     xp.z = frames_vector[i][j].z;
//                     xp.r = frames_vector[i][j].r;
//                     xp.g = frames_vector[i][j].g;
//                     xp.b = frames_vector[i][j].b;
//                     mixVector[k].push_back(xp);
//                  }
//              }

//          }
//     }
// }

//给一个vector中的点云排序，按x坐标从小到大
bool comp(PointT a,PointT b)
{
    return a.x > b.x;
}

int main()
{
    int start = 0;  //设置索引起始文件与终止文件
    int end = 400;
    //pcl::PointCloud<PointT>::Ptr ori_cloud(new pcl::PointCloud<PointT>);
    //pcl::io::loadPCDFile ("../pcd2/444.pcd",*ori_cloud);
    std::vector< std::vector<PointT> > centerOfPoints; //存储经过聚类后取中心点的坐标点
    centerOfPoints = points_cluster(PCD_DIR,start,end);//对原始点云进行聚类和取中心点
    // std::vector< std::vector<PointT> > centerOfPoints_order;
    // centerOfPoints_order = centerOfPoints;

    pcl::PointXYZRGB pc;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pc(new pcl::PointCloud<pcl::PointXYZRGB>);//定义一个存储聚类后取完中心点的点云的点云指针
    //将聚类完成后的点云存入点云指针cloud_pc
    int points_num = 0;
    for (int i = 0;i < centerOfPoints.size();i++)
    {
        //printf("%lu\n",centerOfPoints[i].size());
        for (int j =0;j < centerOfPoints[i].size();j++)
        {
            points_num++;
            pc.x = centerOfPoints[i][j].x;
            pc.y = centerOfPoints[i][j].y;
            pc.z = centerOfPoints[i][j].z;
            pc.r = centerOfPoints[i][j].r;
            pc.g = centerOfPoints[i][j].g;
            pc.b = centerOfPoints[i][j].b;
            cloud_pc->points.push_back(pc);
            std::cout<<centerOfPoints[i][j].x<<","<<centerOfPoints[i][j].y<<","<<centerOfPoints[i][j].z<<","<<(ushort)centerOfPoints[i][j].r<<","<<(ushort)centerOfPoints[i][j].g<<","<<(ushort)centerOfPoints[i][j].b<<std::endl;
        }
    }
    printf("聚类后点云个数:%d\n",points_num);
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::io::loadPCDFile ("../pcd/108.pcd",*cloud3);

    pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>);
    tree2-> setInputCloud(cloud_pc); //设置要使用kdtree的点云
    
    //对已经融合在一起的点云再次聚类
    std::vector<pcl::PointIndices> cluster_indices2; //被分割出来的点云团，标号队列
    pcl::EuclideanClusterExtraction<PointT> ec2;

    ec2.setClusterTolerance (1.5);        //设置近邻聚类半径
    ec2.setMinClusterSize(50);               //每一类的最少点数（可以用来取出孤立点）
    ec2.setMaxClusterSize(1000);             //每一类的最大点数
    ec2.setSearchMethod(tree2);
    ec2.setInputCloud(cloud_pc);
    ec2.extract(cluster_indices2);            //将聚好的类存在cluster_indices2

    std::vector< std::vector<PointT> > last_vector2;

    for(std::vector<pcl::PointIndices>::const_iterator it2 = cluster_indices2.begin();it2 != cluster_indices2.end(); it2++)
    {
        std::vector<PointT> last_vector1;
        for(std::vector<int>::const_iterator pit2 = it2->indices.begin(); pit2 != it2->indices.end(); pit2++)
        {
            last_vector1.push_back(cloud_pc->points[*pit2]);
        }
        last_vector2.push_back(last_vector1);
    }

   
    std::vector<PointT> last_cluster;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0;i < last_vector2.size();i++)
    {
        double s_x = 0;
        double s_y = 0;
        double s_z = 0;
        PointT lp;
        //printf("size:%lu\n",last_vector2[i].size());
        for (int j = 0;j < last_vector2[i].size();j++)
        {
            //std::cout<<last_vector2[i][j].x<<" "<<last_vector2[i][j].y<<" "<<last_vector2[i][j].z<<std::endl;
            s_x =  s_x + last_vector2[i][j].x;
            s_y =  s_y + last_vector2[i][j].y;
            s_z =  s_z + last_vector2[i][j].z;
        }
        lp.x = s_x / last_vector2[i].size(); //对坐标求均值
        lp.y = s_y / last_vector2[i].size();
        lp.z = s_z / last_vector2[i].size();
        lp.r = 255;
        lp.g = 255;
        lp.b = 255;
        last_cluster.push_back(lp);
        clusted_cloud->points.push_back(lp);
        //printf("\n");
    }
    printf("SIZE:%lu\n",last_vector2.size());
    
    double lane_width;
    std::cout<<last_cluster[0].x<<" "<<last_cluster[0].y<<"       "<<last_cluster[1].x<<" "<<last_cluster[1].y<<std::endl;
    lane_width = pow((pow((last_cluster[0].x - last_cluster[1].x),2) + pow((last_cluster[0].y - last_cluster[1].y),2)),0.5)/2; //计算车道宽度
    printf("lane_width:%lf\n",lane_width);

    //排序
    // for (int i = 0;i < last_vector2.size();i++)
    // {
    //     sort(last_vector2[i].begin(),last_vector2[i].end(),comp);
    // }

    //剔除过密的点
    std::vector< std::vector<PointT> > sparse_vectors;   //稀疏点云vector
    std::vector<PointT> sparse_vector;
    pcl::PointCloud<PointT>::Ptr sparse_cloud(new pcl::PointCloud<PointT>);
    for(int i = 0;i < last_vector2.size();i++)
    {
        PointT p;
        for(int j = 0;j < last_vector2[i].size();j++)
        {
            if (j == 0)
            {
                p.x = last_vector2[i][j].x;
                p.y = last_vector2[i][j].y;
                p.z = last_vector2[i][j].z;
                p.r = last_vector2[i][j].r;
                p.g = last_vector2[i][j].g;
                p.b = last_vector2[i][j].b;
                sparse_vector.push_back(p);
                sparse_cloud->points.push_back(p);
            }
            //根据两点之间的距离公式判断要不要将该点加入到新的vector
            else if ( pow((pow((last_vector2[i][j].x-sparse_vector[sparse_vector.size()-1].x),2) + pow((last_vector2[i][j].y-sparse_vector[sparse_vector.size()-1].y),2)),0.5) > 0.5 ) //可在此设置要剔除的最小距离
            {
                p.x = last_vector2[i][j].x;
                p.y = last_vector2[i][j].y;
                p.z = last_vector2[i][j].z;
                p.r = last_vector2[i][j].r;
                p.g = last_vector2[i][j].g;
                p.b = last_vector2[i][j].b;
                sparse_vector.push_back(p);
                sparse_cloud->points.push_back(p);
            }
        }
        sparse_vectors.push_back(sparse_vector);
        sparse_vector.clear();//用完vector后清除现有的点，不影响下一次的使用
    }

    std::ofstream outfile;
    outfile.open("data.txt", ios::binary | ios::app | ios::in | ios::out);
    for (int i = 0;i < sparse_vectors.size();i++)
    {
        printf("一条车道点数：%lu\n",sparse_vectors[i].size());
        for (int j = 0;j < sparse_vectors[i].size();j++)
        {
            outfile<<sparse_vectors[i][j].x<<","<<sparse_vectors[i][j].y<<","<<sparse_vectors[i][j].z<<" ";
        }
        outfile<<std::endl;
    }


    //printf("cloud size:%lu\n",cloud_pc->points.size());
    //viewer1.showCloud(cloud_pc);
    viewer1.showCloud(sparse_cloud);
    //viewer1.showCloud(ori_cloud);
    pause();


    //整体聚类
    // std::string pcd_dir2 = "../pcd2/";
    // int index_start2 = 300;
    // int index_end2 = 441;
    // std::string pcd_ext2 = ".pcd";
    // std::vector< std::vector<PointT> > retn2;
    // pcl::PointCloud<PointT>::Ptr newcloud2(new pcl::PointCloud<PointT>);
    // for (int j = index_start2;j < index_end2;j++)
    // { 
    //     std::stringstream ss;//将j转为string
    //     ss << j;
    //     std::string str = ss.str();
    //     std::string pcd_index = pcd_dir2 + str + pcd_ext2;//目标文件路径
    //     pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    //     //pcl::PointCloud<PointT>::Ptr newcloud(new pcl::PointCloud<PointT>);
    //     pcl::io::loadPCDFile (pcd_index,*cloud1);
    //     PointT p;

    //     //对从文件中读取到的点云进行坐标转换，由相机坐标系转换到笛卡尔
    //     for (int i=0;i<cloud1->points.size();i++)
    //     {
    //         p.x = cloud1->points[i].x/100;
    //         p.y = cloud1->points[i].z/100;
    //         p.z = -(cloud1->points[i].y)/100;
    //         p.r = cloud1->points[i].r;
    //         p.g = cloud1->points[i].g;
    //         p.b = cloud1->points[i].b;
    //         std::cout<<p.x<<","<<p.y<<","<<p.z<<","<<(ushort)p.r<<","<<(ushort)p.g<<","<<(ushort)p.b<<std::endl;
    //         newcloud2->points.push_back(p);
    //     }
    // }
    // //viewer1.showCloud(newcloud2);
    // //pause();


    // pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>);
    // tree2-> setInputCloud(newcloud2); //设置要使用kdtree的点云

    // std::vector<pcl::PointIndices> cluster_indices2; //被分割出来的点云团，标号队列
    // pcl::EuclideanClusterExtraction<PointT> ec2;

    // ec2.setClusterTolerance (1);        //设置近邻搜索的半径
    // ec2.setMinClusterSize(1000);               //每一类的最少点数（可以用来取出孤立点）
    // ec2.setMaxClusterSize(1500000);             //每一类的最大点数
    // ec2.setSearchMethod(tree2);                //设置点云的搜索机制
    // ec2.setInputCloud(newcloud2);
    // ec2.extract(cluster_indices2);            //将聚好的类存在cluster_indices2

    // std::vector< std::vector<PointT> > all_vector2;
    // std::vector<PointT> all_vector1;
    // PointT lp;
    // pcl::PointCloud<PointT>::Ptr line_cloud(new pcl::PointCloud<PointT>);
    
    // for(std::vector<pcl::PointIndices>::const_iterator it2 = cluster_indices2.begin();it2 != cluster_indices2.end(); it2++)
    // {
    //     for(std::vector<int>::const_iterator pit2 = it2->indices.begin(); pit2 != it2->indices.end(); pit2++)
    //     {
    //         all_vector1.push_back(newcloud2->points[*pit2]);
    //     }
    //     if (all_vector1.size() > 2000)
    //     {
    //         for (int i = 0;i < all_vector1.size();i++)
    //         {
    //              lp.x = all_vector1[i].x;
    //              lp.y = all_vector1[i].y;
    //              lp.z = all_vector1[i].z;
    //              lp.r = all_vector1[i].r;
    //              lp.g = all_vector1[i].g;
    //              lp.b = all_vector1[i].b;
    //              line_cloud->points.push_back(lp);
    //         }
    //     }
    //     all_vector2.push_back(all_vector1);
    // }
  
    // viewer1.showCloud(line_cloud);
    // //viewer1.showCloud(newcloud2);
    // pause();

    // sensor_msgs::PointCloud2 output;
    // ros::init(argc, argv, "talker");
    // ros::NodeHandle nh;
    // ros::Publisher my_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
    // output.header.frame_id = "odom";
    // ros::Rate loop_rate(1);
    // pcl::toROSMsg(*cloud_pc,output);
    // while (ros::ok())
    // {
    //     my_pub.publish(output);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}