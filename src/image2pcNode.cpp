// Author of MMS_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
//cv_bridge
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//opencv lib
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

std::mutex mutex_lock;
std::queue<sensor_msgs::ImageConstPtr> colorImageBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
double map_resolution = 0.4;
int color_width,color_height;

ros::Publisher pubStaticPointCloud;
ros::Publisher pubDynamicPointCloud;
ros::Publisher pubimage;
void ColorImageHandler(const sensor_msgs::ImageConstPtr &colorImageMsg)
{
    mutex_lock.lock();
    colorImageBuf.push(colorImageMsg);
    mutex_lock.unlock();
     //ROS_INFO("im in");
}
void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
    //ROS_INFO("pc in");
}

void draBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, int r, int g, int b){
    for(double i=min_x;i<max_x;i=i+0.02){
        pcl::PointXYZRGB point_temp1;
        point_temp1.x = i;
        point_temp1.y = min_y;
        point_temp1.z = min_z;
        point_temp1.r = r;
        point_temp1.g = g;
        point_temp1.b = b;
        pc_in->push_back(point_temp1);

        pcl::PointXYZRGB point_temp2;
        point_temp2.x = i;
        point_temp2.y = min_y;
        point_temp2.z = max_z;
        point_temp2.r = r;
        point_temp2.g = g;
        point_temp2.b = b;
        pc_in->push_back(point_temp2);

        pcl::PointXYZRGB point_temp3;
        point_temp3.x = i;
        point_temp3.y = max_y;
        point_temp3.z = min_z;
        point_temp3.r = r;
        point_temp3.g = g;
        point_temp3.b = b;
        pc_in->push_back(point_temp3);

        pcl::PointXYZRGB point_temp4;
        point_temp4.x = i;
        point_temp4.y = max_y;
        point_temp4.z = max_z;
        point_temp4.r = r;
        point_temp4.g = g;
        point_temp4.b = b;
        pc_in->push_back(point_temp4);
    }

    for(double i=min_y;i<max_y;i=i+0.02){
        pcl::PointXYZRGB point_temp1;
        point_temp1.x = min_x;
        point_temp1.y = i;
        point_temp1.z = min_z;
        point_temp1.r = r;
        point_temp1.g = g;
        point_temp1.b = b;
        pc_in->push_back(point_temp1);

        pcl::PointXYZRGB point_temp2;
        point_temp2.x = min_x;
        point_temp2.y = i;
        point_temp2.z = max_z;
        point_temp2.r = r;
        point_temp2.g = g;
        point_temp2.b = b;
        pc_in->push_back(point_temp2);

        pcl::PointXYZRGB point_temp3;
        point_temp3.x = max_x;
        point_temp3.y = i;
        point_temp3.z = min_z;
        point_temp3.r = r;
        point_temp3.g = g;
        point_temp3.b = b;
        pc_in->push_back(point_temp3);

        pcl::PointXYZRGB point_temp4;
        point_temp4.x = max_x;
        point_temp4.y = i;
        point_temp4.z = max_z;
        point_temp4.r = r;
        point_temp4.g = g;
        point_temp4.b = b;
        pc_in->push_back(point_temp4);
    }

    for(double i=min_z;i<max_z;i=i+0.02){
        pcl::PointXYZRGB point_temp1;
        point_temp1.x = min_x;
        point_temp1.y = min_y;
        point_temp1.z = i;
        point_temp1.r = r;
        point_temp1.g = g;
        point_temp1.b = b;
        pc_in->push_back(point_temp1);

        pcl::PointXYZRGB point_temp2;
        point_temp2.x = min_x;
        point_temp2.y = max_y;
        point_temp2.z = i;
        point_temp2.r = r;
        point_temp2.g = g;
        point_temp2.b = b;
        pc_in->push_back(point_temp2);

        pcl::PointXYZRGB point_temp3;
        point_temp3.x = max_x;
        point_temp3.y = min_y;
        point_temp3.z = i;
        point_temp3.r = r;
        point_temp3.g = g;
        point_temp3.b = b;
        pc_in->push_back(point_temp3);

        pcl::PointXYZRGB point_temp4;
        point_temp4.x = max_x;
        point_temp4.y = max_y;
        point_temp4.z = i;
        point_temp4.r = r;
        point_temp4.g = g;
        point_temp4.b = b;
        pc_in->push_back(point_temp4);
    }



}

double total_time =0;
int total_frame=0;
void image2pc(){
        // std::chrono::milliseconds dura(3000);
        // std::this_thread::sleep_for(dura);
    while(1){
        if(!pointCloudBuf.empty() && !colorImageBuf.empty()){

            //read data
            mutex_lock.lock();
            //if(total_frame<100){


            if(pointCloudBuf.front()->header.stamp.toSec() - 0.01 > colorImageBuf.front()->header.stamp.toSec()){
                // ROS_INFO("time stamp unaligned error and colorimage discarded, pc:%f, image:%f--> image2pc_process",pointCloudBuf.front()->header.stamp.toSec(), colorImageBuf.front()->header.stamp.toSec()); 
                colorImageBuf.pop();
                mutex_lock.unlock();

                continue;              
            }
            if(colorImageBuf.front()->header.stamp.toSec() - 0.01 > pointCloudBuf.front()->header.stamp.toSec()){
                // ROS_INFO("time stamp unaligned error and depthimage discarded, pc:%f, image:%f--> image2pc_process",pointCloudBuf.front()->header.stamp.toSec(), colorImageBuf.front()->header.stamp.toSec()); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }
            

            //ROS_INFO("time stamp unaligned error and colorimage discarded, pc:%f, image:%f--> image2pc_process",pointCloudBuf.front()->header.stamp.toSec(), colorImageBuf.front()->header.stamp.toSec()); 
            //if time aligned 
            cv_bridge::CvImagePtr color_image_ptr = cv_bridge::toCvCopy(*colorImageBuf.front(), sensor_msgs::image_encodings::MONO8);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (colorImageBuf.front())->header.stamp;
            colorImageBuf.pop();
            pointCloudBuf.pop();
            mutex_lock.unlock();

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            total_frame++;

            //ROS_INFO("total frame %d",total_frame);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_obj_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
            //fuse color image and depth image

  
            Eigen::Isometry3d pose_trans = Eigen::Isometry3d::Identity();
            pose_trans.translation() = Eigen::Vector3d(0.001, 0.014, -0.007);
            Eigen::Quaterniond quaternion_temp(1.00,-0.012, -0.001, -0.003); 
            pose_trans.linear() = quaternion_temp.toRotationMatrix();;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*pointcloud_in, *transformed_pc, pose_trans.cast<float>());
            // std::cout<<pointcloud_in->size()<<std::endl;
            if (pointcloud_in->size()<10000)
            {
                ROS_WARN("too few points: %d",pointcloud_in->size());
                continue;
            }
            //image dilation
            
            
            //0: Rect - 1: Cross - 2: Ellipse
            int morph_elem = 1;
            //max 21
            int morph_size = 3;
            cv::Mat element = cv::getStructuringElement(morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
            cv::Mat dilated_image;

            // ROS_INFO("dimension %d * %d",color_image_ptr->image.cols,color_image_ptr->image.rows);
            if(color_image_ptr->image.cols!=color_width || color_image_ptr->image.rows!=color_height){
                ROS_WARN("input mask dimension error,%d,%d",color_image_ptr->image.rows,color_image_ptr->image.cols);
                // ROS_WARN("width and cols,%d,%d",color_width,color_image_ptr->image.cols);
                // ROS_WARN("height and rows,%d,%d",color_image_ptr->image.rows,color_height);
                continue;
            }
            cv::dilate(color_image_ptr->image, dilated_image, element,cv::Point(-1,-1), 2, cv::BORDER_REPLICATE);
            // count_temp=0;
            for(int i=0;i<color_width;i++){
                for(int j=0;j<color_height;j++){
                    if (dilated_image.at<unsigned char>(j,i)!=0)
                        dilated_image.at<unsigned char>(j,i) *= 100;
                }
            }
            //ROS_INFO("total points dilated %d * %d",dilated_image.rows,dilated_image.cols);
                  

            std::vector<int> static_pc_id;
            std::vector<int> dynamic_pc_id;
            bool is_dynamic_final[pointcloud_in->points.size()];
            for(int i=0;i<pointcloud_in->points.size();i++){
                is_dynamic_final[i]=false;
            	pcl::PointXYZRGB point_temp;
            	point_temp.x = pointcloud_in->points[i].x;
            	point_temp.y = pointcloud_in->points[i].y;
            	point_temp.z = pointcloud_in->points[i].z;
            	point_temp.r = pointcloud_in->points[i].r;
            	point_temp.g = pointcloud_in->points[i].g;
            	point_temp.b = pointcloud_in->points[i].b;
            	float fx=912.513,fy=913.079,cx=960,cy=540;
                int pixel_x = fx * transformed_pc->points[i].x/transformed_pc->points[i].z + cx;
                int pixel_y = fy * transformed_pc->points[i].y/transformed_pc->points[i].z + cy;
                //ROS_INFO("width = %d, height=%d",color_image_ptr->image.cols,color_image_ptr->image.rows);
                // width = cols=1280, height=720
                if(pixel_x< 0 || pixel_x>=color_image_ptr->image.cols){
                    ROS_WARN("unaligned points, pixel_x = %d, width=%d",pixel_x,color_image_ptr->image.cols);
                    continue;
                }
                if(pixel_y< 0 || pixel_y>=color_image_ptr->image.rows){
                    ROS_WARN("unaligned points, pixel_y = %d, height=%d",pixel_y,color_image_ptr->image.rows);
                    continue;
                }
                
                    // static_pc->push_back(point_temp);
                    // static_pc_id.push_back(i);
                    
                if (dilated_image.at<unsigned char>(pixel_y,pixel_x)==100)
            	{
                    //is_dynamic_final[i]=true;
                    dynamic_pc->push_back(point_temp); 
                    dynamic_pc_id.push_back(i);
                }else if (dilated_image.at<unsigned char>(pixel_y,pixel_x)!=0){
                    static_obj_pc->push_back(point_temp); 
                    static_pc->push_back(point_temp);
                    static_pc_id.push_back(i);
                }else{
                    static_pc->push_back(point_temp);
                    static_pc_id.push_back(i);
                }

            }
            // ROS_INFO("all = %d, static=%d, dynamic = %d",pointcloud_in->points.size(),static_pc->points.size(),dynamic_pc->points.size());
            if(pointcloud_in->points.size() - dynamic_pc->points.size()<100){
                ROS_WARN("not enough static points%d/%d", dynamic_pc->points.size(),pointcloud_in->points.size());
                continue;
            }   

            //draw object shape
            if(static_obj_pc->points.size()>0){
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_obj_downsized(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
                downSizeFilter.setInputCloud (static_obj_pc);
                downSizeFilter.setLeafSize (map_resolution, map_resolution, map_resolution);
                downSizeFilter.filter (*static_obj_downsized);
                
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtreeDynamicMap (new pcl::search::KdTree<pcl::PointXYZRGB>);
                //pcl::KdTree<pcl::PointXYZRGB>::Ptr kdtreeDynamicMap(new pcl::KdTree<pcl::PointXYZRGB>);
                if(static_obj_downsized->points.size()==0)
                {
                    ROS_WARN("ZERO 343");
                }
                // ROS_INFO("ZERO 343 %d",static_obj_downsized->points.size()); 
                kdtreeDynamicMap->setInputCloud (static_obj_downsized);//创建点云索引向量，用于存储实际的点云信息
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
                ec.setClusterTolerance (0.1); //set searching radius to 4dm
                ec.setMinClusterSize(20);//set least number of clustering 
                ec.setMaxClusterSize(100000); //set maximum clustering point num
                ec.setSearchMethod(kdtreeDynamicMap);
                ec.setInputCloud(static_obj_downsized);
                ec.extract(cluster_indices);
                for (int cluster_index=0;cluster_index<cluster_indices.size();cluster_index++)
                {
                    int segment_size = cluster_indices[cluster_index].indices.size();
                    //search for the area density
                    //ROS_INFO("segment_size: %d", segment_size);
                    if(segment_size>500){
                        double max_x = -1000.0;
                        double max_y = -1000.0;
                        double max_z = -1000.0;
                        double min_x = 1000.0;
                        double min_y = 1000.0;
                        double min_z = 1000.0;
                        for(int i=0;i<segment_size;i++){
                            if(static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].x > max_x) max_x = static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].x;
                            if(static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].y > max_y) max_y = static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].y;
                            if(static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].z > max_z) max_z = static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].z;
                            if(static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].x < min_x) min_x = static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].x;
                            if(static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].y < min_y) min_y = static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].y;
                            if(static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].z < min_z) min_z = static_obj_downsized->points[cluster_indices[cluster_index].indices[i]].z;
                        }
                        //calculate intensity
                        
                        //draw bounding box
                        draBoundingBox(bounding_box_pc,min_x,min_y,min_z,max_x,max_y,max_z,0,255,0);

                    }
                }
            }
            
            //search for dynamic shape
            if(dynamic_pc->points.size()>0){
           
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_pc_downsized(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
                downSizeFilter.setInputCloud (dynamic_pc);
                downSizeFilter.setLeafSize (map_resolution, map_resolution, map_resolution);
                downSizeFilter.filter (*dynamic_pc_downsized);

                // pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_pc_downsized(new pcl::PointCloud<pcl::PointXYZRGB>());
                // downSizeFilter.setInputCloud (pointcloud_in);
                // downSizeFilter.filter (*pointcloud_in_downsized);

                //step 1 dynamic point cloud segmentation
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtreeDynamicMap (new pcl::search::KdTree<pcl::PointXYZRGB>);
                //pcl::KdTree<pcl::PointXYZRGB>::Ptr kdtreeDynamicMap(new pcl::KdTree<pcl::PointXYZRGB>);
                if(dynamic_pc_downsized->points.size()==0)
                {
                    ROS_WARN("ZERO 401");
                }
                // ROS_INFO("ZERO 401 %d",dynamic_pc_downsized->points.size()); 
                kdtreeDynamicMap->setInputCloud (dynamic_pc_downsized);
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
                ec.setClusterTolerance (0.1); 
                ec.setMinClusterSize(20);
                ec.setMaxClusterSize(100000);//设置点云的搜索机制
                ec.setInputCloud(dynamic_pc_downsized);
                ec.extract(cluster_indices);

                for (int cluster_index=0;cluster_index<cluster_indices.size();cluster_index++)
                {
                    int segment_size = cluster_indices[cluster_index].indices.size();
                    //search for the area density
                    //ROS_INFO("segment_size: %d", segment_size);
                    if(segment_size>2000){
                        double max_x = -1000.0;
                        double max_y = -1000.0;
                        double max_z = -1000.0;
                        double min_x = 1000.0;
                        double min_y = 1000.0;
                        double min_z = 1000.0;
                        for(int i=0;i<segment_size;i++){
                            if(dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].x > max_x) max_x = dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].x;
                            if(dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].y > max_y) max_y = dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].y;
                            if(dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].z > max_z) max_z = dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].z;
                            if(dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].x < min_x) min_x = dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].x;
                            if(dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].y < min_y) min_y = dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].y;
                            if(dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].z < min_z) min_z = dynamic_pc_downsized->points[cluster_indices[cluster_index].indices[i]].z;
                        }
                        //calculate intensity
                        pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;
                        double expansion_size = 0.2;
                        cropBoxFilter.setMin(Eigen::Vector4f(min_x-expansion_size, min_y-expansion_size, min_z-3*expansion_size, 1.0));
                        cropBoxFilter.setMax(Eigen::Vector4f(max_x+expansion_size, max_y+expansion_size, max_z+3*expansion_size, 1.0));
                        cropBoxFilter.setNegative(true);  
                        if(static_pc->points.size()==0)
                        {
                            ROS_WARN("ZERO 440");
                        }
                        // ROS_INFO("ZERO 440 %d",static_pc->points.size());  
                        cropBoxFilter.setInputCloud(static_pc);
                        //cropBoxFilter.filter(*static_pc);
                        
                        //draw bounding box
                        draBoundingBox(bounding_box_pc,min_x,min_y,min_z,max_x,max_y,max_z,255,0,0);

                    }
                }

               
            }
            
          
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            if(total_frame%100==0)
                ROS_INFO("average point cloud filtering time %f ", elapsed_seconds.count() * 1000 / total_frame);

            // publish point cloud
            sensor_msgs::PointCloud2 static_pc_msg;
            //pcl::toROSMsg(*static_pc, static_pc_msg);
            pcl::toROSMsg(*static_pc, static_pc_msg);
            static_pc_msg.header.stamp = pointcloud_time;
            static_pc_msg.header.frame_id = "/camera_depth_optical_frame";
            pubStaticPointCloud.publish(static_pc_msg);
            
            //dynamic pc
            sensor_msgs::PointCloud2 dynamic_pc_msg;
            pcl::toROSMsg(*dynamic_pc + *bounding_box_pc, dynamic_pc_msg);
            dynamic_pc_msg.header.stamp = pointcloud_time;
            dynamic_pc_msg.header.frame_id = "/camera_depth_optical_frame";
            pubDynamicPointCloud.publish(dynamic_pc_msg);

            cv_bridge::CvImage out_msg;
            out_msg.header.frame_id  = "/camera_depth_optical_frame"; 
            out_msg.header.stamp  = pointcloud_time; 
            out_msg.encoding = sensor_msgs::image_encodings::MONO8; 
            out_msg.image    = dilated_image; 
            pubimage.publish(out_msg.toImageMsg());

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    //Eigen::Matrix<double,double>

    nh.getParam("/map_resolution", map_resolution);
    map_resolution = 0.01;
    nh.getParam("/color_height", color_height);
    nh.getParam("/color_width", color_width);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 100, velodyneHandler);
    image_transport::Subscriber colorImageSub = it.subscribe("/solo_node/mask", 100, ColorImageHandler);

    pubStaticPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_static", 100);
    pubDynamicPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_dynamic", 100);
    pubimage = nh.advertise<sensor_msgs::Image>("dilated_image", 100);
    std::thread image2pc_process{image2pc};

    ros::spin();

    return 0;
}
