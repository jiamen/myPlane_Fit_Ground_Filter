//
// Created by zlc on 2021/3/8.
//


#ifndef _PLANE_GROUND_FILTER_PLANE_GROUND_FILTER_CORE_H_
#define _PLANE_GROUND_FILTER_PLANE_GROUND_FILTER_CORE_H_

#pragma once


#include <ros/ros.h>

//For disable PCL compile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>


// Use eigen lib
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>

namespace velodyne_pointcloud
{
// Euclidean Velodyne coordinate, including intensity and ring number. 欧氏速度坐标，包括强度和环数。
struct PointXYZIR
{
    PCL_ADD_POINT4D;        // quad-word XYZ
    float intensity;        // laser intensity reading
    uint16_t ring;          // laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

};  // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float,x,x)(float,y,y)(float,z,z)(float,intensity, intensity)(uint16_t,ring,ring));


// Customed Point Struct for holding clustered points. 用于容纳聚集点的自定义点结构。
namespace plane_ground_filter
{
// Euclidean Velodyne coordinate, including intensity and ring number, and label. 欧氏速度坐标，包括强度和环数以及标签。
struct PointXYZIRL
{
    PCL_ADD_POINT4D;        // quad-word XYZ
    float intensity;        // laser intensity reading
    uint16_t ring;          // laser ring number
    uint16_t label;         // point label
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // ensure proper alignment
} EIGEN_ALIGN16;

};  // namespace plane_ground_filter


#define SLRPointXYZIRL plane_ground_filter::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCLoud<SLRPointXYZIRL>

POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::PointXYZIRL,
                                  (float,x,x)(float,y,y)(float,z,z)(float,intensity, intensity)(uint16_t,ring,ring)(uint16_t,label,label));

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;


class PlaneGroundFilter
{
private:
    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;    // 发布分割出来地面点云，去除地面的点云，全部点云
    std::string point_topic_;

    int sensor_model_;          // 传感器模型
    double sensor_height_, clip_height_, min_distance_, max_distance_;      // 除了地面点云需要去除，也需要去除过高和过近过远的点
    int num_seg_ = 1;           //
    int num_iter_, num_lpr_;    // num_lpr_个最小点
    double th_seeds_, th_dist_;


    float d_, th_dist_d_;       // 判断点是否属于地平面的阈值 Th_dist
    MatrixXf normal_;           // 地平面法向量


    // Model parameter for ground plane fitting
    pcl::PointCloud<VPoint>::Ptr g_seeds_pc;        // 种子点云
    pcl::PointCloud<VPoint>::Ptr g_ground_pc;       // 地面点云
    pcl::PointCloud<VPoint>::Ptr g_not_ground_pc;   // 非地面点云
    pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc;  // 所有点云，带标签数据


    void estimate_plane_(void);         // 拟合地平面，确定阈值Th_dist
    void extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted);       // 种子点集的选取
    void post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);   // 进一步处理，去除点云中过高、过远、过近的点
    void clip_above(const pcl::PointCloud<VPoint>::Ptr in,
                    const pcl::PointCloud<VPoint>::Ptr out);
    void remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,
                             const pcl::PointCloud<VPoint>::Ptr out);
    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr);


public:
    PlaneGroundFilter(ros::NodeHandle& nh);
    ~PlaneGroundFilter();
    void Spin();
    // spin函数，一旦进入spin函数，它就不会返回了，相当于它在自己的函数里面死循环了。
    // 只要回调函数队列里面有callback函数在，它就会马上去执行callback函数。如果没有的话，它就会阻塞，不会占用CPU。
};



#endif //PLANE_GROUND_FILTER_PLANE_GROUND_FILTER_CORE_H
