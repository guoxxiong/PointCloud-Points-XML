 /*BEGIN_FILE_HDR
 ************************************************************************************
 *NOTICE

 ************************************************************************************
 *File Name:create_xml
 ************************************************************************************
 *Project/Product:ROS VSLAM
 *Title:
 *Author:xiong.guo
 ************************************************************************************
 *Description:
 *
 *(Requirements,pseudo code and etc.)
 ************************************************************************************
 *Limitations:
 *
 *(limitations)
 ************************************************************************************

 ************************************************************************************
 *Revision History:
 *
 *Version          Date         Initials        CR#          Descriptions
 *--------     -----------     -----------   ---------     ----------------------
 *1.0           05/07/18                       N/A            Original
 *
 ************************************************************************************
 *END_FILE_HDR*/


 /*Includes*/
 /*****************************************************************************************/
 #include"../include/pcd_to_xml/edgconfig.h"
 #include<pcl_conversions/pcl_conversions.h>
 #include<sensor_msgs/PointCloud2.h>
 #include<pcl/common/transforms.h>


 using namespace std;

 /*Static variables*/
 /*****************************************************************************************/
typedef pcl::PointXYZRGB PointT;
 typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;

 PointT p;
 pcl::visualization::CloudViewer viewer("cloud viewer");
 ColorCloud::Ptr cloud(new ColorCloud);

 vector<PointT> PointVector;
 vector< vector<PointT> > PointsVector;

 unsigned char strid[32];
 unsigned char strid2[32];
 unsigned char strx[64];
 unsigned char stry[64];
 unsigned char strz[64];
 unsigned char strfrom[64];
 unsigned char strto[64];
 unsigned char strshape[5000]; 
 unsigned char strnumlanes[5]="3";  //Define quantity of lanes
 unsigned char strtypes[20]="priority"; //Define element "type" for an edge

 int nodeid=1;
 int egdefrom[100];
 int edgeto[100];
 char shapes[]="\0";
 char ashape[]="\0";

 //point结构体,double coordinates
 typedef struct point_t{
     int id1;
     double x1;
     double y1;
     double z1;
 }nodepoint;
 nodepoint node_point;


 /*BEGIN_FUNCTION_HDR
 *******************************************************************************************
 *Function Name:node_point_init
 *Description:
 *
 *
 *Inputs:PointT point1
 *
 *
 *Outputs:void
 *
 *
 *Limitations:
 *******************************************************************************************
 END_FUNCTION_HDR*/
 void node_point_init(PointT point1)
 {
     node_point.id1=nodeid;
     node_point.x1=point1.x;
     node_point.y1=point1.y;
     node_point.z1=point1.z;
     nodeid++;
 }


 /*BEGIN_FUNCTION_HDR
 *******************************************************************************************
 *Function Name:node_point_init
 *Description:将字符串char str[]转换为三维坐标存储到PointVector
 *
 *
 *Inputs:str_to_point()
 *
 *
 *Outputs:void
 *
 *
 *Limitations:
 *******************************************************************************************
 END_FUNCTION_HDR*/
 void str_to_point(char str[])
 {
     int i=0;
     int j=0;
     while(str[i]!='\0')
     {
         while((str[i]!=' ')&&str[i]!='\0')
         {
             for(int l=0;l<3;l++)
             {
                 char str1[20]="\0";
                 int leng=0;
                 float x1,y1,z1;
                 while((str[i]!=',')&&(str[i]!=' ')&&(str[i]!='\0'))
                 {
                     str1[leng]=str[i];
                     i++;
                     leng++;
                 }
                 if(l==0)
                 { 
                     sscanf(str1,"%f",&x1);
                     p.x=x1;
                    
                 }
                 if(l==1)
                 { 
                     sscanf(str1,"%f",&y1);
                     p.y=y1;
                 }
                 if(l==2)
                 { 
                     sscanf(str1,"%f",&z1);
                     p.z=z1;
                 } 
                 i++;  
            }
            PointVector.push_back(p);
            if((str[i-1]!='\0')&&(str[i-1]!=' '))
            { i++; }
            if(str[i-1]=='\0')
            { i--; }
               
         }
     }
 }


 /*BEGIN_FUNCTION_HDR
 *******************************************************************************************
 *Function Name:node_point_init
 *Description:edge define(dynamic properties),struct
 *
 *
 *Inputs:
 *
 *
 *Outputs:
 *
 *
 *Limitations:
 *******************************************************************************************
 END_FUNCTION_HDR*/
 typedef struct{
     int id;
     int from;
     int to;
     char shape[5000];
 }edgest;


 /*BEGIN_FUNCTION_HDR
 *******************************************************************************************
 *Function Name:edge_init
 *Description:initing properties of every edge
 *
 *
 *Inputs:vector< vector<PointT> >
 *
 *
 *Outputs:
 *
 *
 *Limitations:
 *******************************************************************************************
 END_FUNCTION_HDR*/
 void edge_init(vector< vector<PointT> > edgePoints,edgest edges[],int edgefrom[],int edgeto[])
 {
     int s,l;
     char shapes[5000];
     char ashape[50];
     s=edgePoints.size(); //判断一共有几条edge
     for(int i=0;i<s;i++)
     {
         edges[i].id=10000+i;
         l=edgePoints[i].size();
         edges[i].from=edgefrom[i];
         edges[i].to=edgeto[i];
         //"shape"属性初始化
         for(int j=0;j<edgePoints[i].size();j++)
         {
             char comma[]=",";
             char space[]=" ";
             sprintf((char*)ashape,"%lf",edgePoints[i][j].x);
             strcat(shapes,ashape);
             memset(ashape,'\0',sizeof(ashape));
             strcat(shapes,comma);
             sprintf((char*)ashape,"%lf",edgePoints[i][j].y);
             strcat(shapes,ashape);
             memset(ashape,'\0',sizeof(ashape));
             //strcat(shapes,comma);
             //sprintf((char*)ashape,"%lf",edgePoints[i][j].z);
             //strcat(shapes,ashape);
             //memset(ashape,'\0',sizeof(ashape));
             if(j!=edgePoints[i].size()-1)
             {
                 strcat(shapes,space);
             } 
         }
         //printf("%s\n",shapes);
         for(int m=0;m<strlen(shapes);m++)
         {
             edges[i].shape[m]=shapes[m]; 
             printf("%c",edges[i].shape[m]);
         }
         printf("\n");
         memset(shapes,'\0',sizeof(shapes));
     }
 }


 /*BEGIN_FUNCTION_HDR
 *******************************************************************************************
 *Function Name:PointVector_offset()
 *Description:doing some random offsets to points in a vector
 *
 *
 *Inputs:vector<PointT>
 *
 *
 *Outputs:vector<PointT>
 *
 *
 *Limitations:
 *******************************************************************************************
 END_FUNCTION_HDR*/
 /*vector<PointT> PointVector_offset(vector<PointT> vp)
 {
     vector<PointT> vr;
     PointT pt;
     srand((unsigned)time(NULL));//init random seeds
     for (int i=0;i<vp.size();i++)
     {
         if(i%2==0)
         {
             pt.x=vp[i].x+0.05*(rand()/double(RAND_MAX));
             pt.y=vp[i].y+0.05*(rand()/double(RAND_MAX));
             pt.z=vp[i].z+0.05*(rand()/double(RAND_MAX));
         }
         else
         {
             pt.x=vp[i].x-0.05*(rand()/double(RAND_MAX));
             pt.y=vp[i].y-0.05*(rand()/double(RAND_MAX));
             pt.z=vp[i].z-0.05*(rand()/double(RAND_MAX));
         }
         vr.push_back(pt);
     }
     return vr;
 }*/


 /*BEGIN_FUNCTION_HDR
 *******************************************************************************************
 *Function Name:main
 *Description:main function was achieved by using many functions
 *
 *
 *Inputs:
 *
 *
 *Outputs:
 *
 *
 *Limitations:
 *******************************************************************************************
 END_FUNCTION_HDR*/
  int main()
 {
     char strs1[]="-261.19,65.95,0 -221.69,60.71,0 -212.09,59.38,0 -202.18,58.74,0 -174.12,60.72,0 -154.27,63.46,0 -133.18,65.72,0 -115.17,67.11,0 -107.94,66.89,0 -97.93,65.79,0 -80.26,63.56,0 -60.70,61.62,0 -39.03,60.45,0 111.04,59.10,0";
     char strs2[]="186.23,60.92,0 200.79,24.57,0 202.78,21.06,0 205.00,17.87,0 207.26,15.30,0 210.08,12.88,0 213.94,10.08,0 217.14,7.90,0 221.51,5.31,0 225.65,2.94,0 229.43,0.95,0 259.61,-14.92,0";
     char strs3[]="332.34,58.32,0 335.38,-32.60,0";
     char strs4[]="335.38,-32.60,0 332.34,58.32,0";
     char strs5[]="111.04,59.10,0 -27.52,60.49,0 -41.42,60.55,0 -62.50,61.82,0 -80.04,63.60,0 -95.40,65.42,0 -104.39,66.58,0 -112.93,67.11,0 -123.10,66.46,0 -133.64,65.61,0 -154.98,63.47,0 -174.41,60.70,0 -202.12,58.74,0 -212.21,59.43,0 -222.36,60.78,0 -261.19,65.95,0";
     char strs6[]="328.12,163.03,0 332.34,58.32,0";
     char strs7[]="332.34,58.32,0 328.12,163.03,0";
     char strs8[]="111.04,59.10,0 186.23,60.92,0";
     char strs9[]="186.23,60.92,0 111.04,59.10,0";
     char strs10[]="186.23,60.92,0 332.34,58.32,0";
     char strs11[]="332.34,58.32,0 186.23,60.92,0";
     char strs12[]="469.51,51.94,0 332.34,58.32,0";
     char strs13[]="332.34,58.32,0 469.51,51.94,0";
     char strs14[]="186.23,60.92,0 99.77,110.89,0";
     int nodeid=1;
     int edgefrom1[50];
     int edgeto1[50];
     //vector<PointT> offset_PointVector;

     //可以在这里设置与点云数据的接口，PointsVector=vector< vector<pcl::pointXYZRGB> >
     str_to_point(strs1);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs2);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs3);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs4);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs5);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs6);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs7);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs8);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs9);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs10);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs11);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs12);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs13);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     str_to_point(strs14);
     PointsVector.push_back(PointVector);
     for(int i=0;i<PointVector.size();i++)
     {
         PointVector[i].r=255;
         PointVector[i].g=255;
         PointVector[i].b=255;
         cloud->points.push_back(PointVector[i]);
     }
     PointVector.clear();

     //offset_PointVector=PointVector_offset(PointVector);

     //PointsVector.push_back(offset_PointVector);

     /*for(int i=0;i<offset_PointVector.size();i++)
     {
         offset_PointVector[i].r=255;
         offset_PointVector[i].g=255;
         offset_PointVector[i].b=255;
         cloud->points.push_back(offset_PointVector[i]);
     }*/
    
     //offset_PointVector.clear();
     //str_to_point(strs2);
     //PointsVector.push_back(PointVector);
     printf("coordinates:\n");
     for(int i=0;i<PointsVector.size();i++)
     {
          for(int j=0;j<PointsVector[i].size();j++)
          {
             cout<<"("<<PointsVector[i][j].x<<","<<PointsVector[i][j].y<<","<<PointsVector[i][j].z<<")"<<endl;
          }
          printf("\n");
     }

     //generated nod.xml
     xmlDocPtr nodedoc = xmlNewDoc(BAD_CAST"1.0");//文档指针
     xmlNodePtr root_node = xmlNewNode(NULL,BAD_CAST"nodes");
     xmlDocSetRootElement(nodedoc,root_node);//设置根节点


     for(int i=0;i<PointsVector.size();i++)
     {
         for(int j=0;j<PointsVector[i].size();j++)
         {
             node_point_init(PointsVector[i][j]);
             //分别取出edge的起点与终点的id
             if(j==0)
             {
                 edgefrom1[i]=node_point.id1;
                 //double convert to unsigned char
                 sprintf((char*)strid, "%d", node_point.id1);
                 sprintf((char*)strx, "%lf", node_point.x1);
                 sprintf((char*)stry, "%lf", node_point.y1);
                 //sprintf((char*)strz, "%lf", node_point.z1);

                 //add a newnode
                 xmlNodePtr new_node = xmlNewNode(NULL,BAD_CAST "node");
                 xmlAddChild(root_node,new_node);
                 xmlNewProp(new_node,BAD_CAST "id",strid);
                 xmlNewProp(new_node,BAD_CAST "x",strx);
                 xmlNewProp(new_node,BAD_CAST "y",stry);
                 //xmlNewProp(new_node,BAD_CAST "z",strz);
                 xmlNewProp(new_node,BAD_CAST "type",strtypes);
             }
             if(j==PointsVector[i].size()-1)
             {
                 edgeto1[i]=node_point.id1;
                 //double convert to unsigned char
                 sprintf((char*)strid, "%d", node_point.id1);
                 sprintf((char*)strx, "%lf", node_point.x1);
                 sprintf((char*)stry, "%lf", node_point.y1);
                 //sprintf((char*)strz, "%lf", node_point.z1);

                 //add a newnode
                 xmlNodePtr new_node = xmlNewNode(NULL,BAD_CAST "node");
                 xmlAddChild(root_node,new_node);
                 xmlNewProp(new_node,BAD_CAST "id",strid);
                 xmlNewProp(new_node,BAD_CAST "x",strx);
                 xmlNewProp(new_node,BAD_CAST "y",stry);
                 //xmlNewProp(new_node,BAD_CAST "z",strz);
                 xmlNewProp(new_node,BAD_CAST "type",strtypes);
             }      
         }
     }
     //存储文档
     int nRel = xmlSaveFile(NODE_FILE,nodedoc);
 	if (nRel != -1)
 	{
 		printf("一个nod.xml文档被创建，写入%d个字节\n", nRel);
 	}

     xmlFreeDoc(nodedoc);

     edgest edges1[50];
     //generated edg.xml
     xmlDocPtr edgedoc = xmlNewDoc(BAD_CAST"1.0");//文档指针
     xmlNodePtr edge_root_node = xmlNewNode(NULL,BAD_CAST"edges");
     xmlDocSetRootElement(edgedoc,edge_root_node);//设置根节点
    
     edge_init(PointsVector,edges1,edgefrom1,edgeto1);

     for(int i=0;i<PointsVector.size();i++)
     {
         sprintf((char*)strid2,"%d",edges1[i].id);
         sprintf((char*)strfrom,"%d",edges1[i].from);
         sprintf((char*)strto,"%d",edges1[i].to);
         sprintf((char*)strshape,"%s",edges1[i].shape);

         xmlNodePtr new_node = xmlNewNode(NULL,BAD_CAST "edge");
 	    xmlAddChild(edge_root_node,new_node);
 	    xmlNewProp(new_node,BAD_CAST "id",strid2);
         xmlNewProp(new_node,BAD_CAST "from",strfrom);
         xmlNewProp(new_node,BAD_CAST "to",strto);
         xmlNewProp(new_node,BAD_CAST "numLanes",strnumlanes);
         xmlNewProp(new_node,BAD_CAST "shape",strshape);
     }
     int nRel1 = xmlSaveFile(EDGE_FILE,edgedoc);
 	if (nRel1 != -1)
 	{
 		printf("一个edg.xml文档被创建，写入%d个字节\n", nRel1);
 	}
     xmlFreeDoc(edgedoc);
     viewer.showCloud(cloud);
     pause();
     return 0;
 }

 void MyTopicCallback(std::vector< std::vector<PointT> > pubpoints)
 {
     std::cout<<pubpoints[0][0].x<<std::endl;
 }

 int main(int argc,char **argv)
 {
     ros::init(argc,argv,"listener");
     ros::NodeHandle nh;
     ros::Subscriber my_sub = nh.subscribe("PointsCloud", 100, MyTopicCallback);
     ros::spin();
 }