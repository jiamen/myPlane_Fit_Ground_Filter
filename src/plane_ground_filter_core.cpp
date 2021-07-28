//
// Created by zlc on 2021/3/8.
//

#include "../include/plane_ground_filter_core.h"

/*
 * 函数功能：根据z轴对点云进行排序
 * */
bool point_cmp(VPoint a, VPoint b)
{
    return a.z < b.z;
}

// 构造函数
PlaneGroundFilter::PlaneGroundFilter(ros::NodeHandle &nh)
{
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &PlaneGroundFilter::point_cb, this);

    // init publisher
    std::string no_ground_topic, ground_topic, all_points_topic;

    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);

    nh.getParam("clip_height", clip_height_);
    ROS_INFO("clip_height: %f", clip_height_);
    nh.getParam("sensor_height", sensor_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);
    nh.getParam("min_distance", min_distance_);
    ROS_INFO("min_distance: %f", min_distance_);
    nh.getParam("max_distance", max_distance_);
    ROS_INFO("max_distance: %f", max_distance_);

    nh.getParam("sensor_model", sensor_model_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    nh.getParam("num_iter", num_iter_);
    ROS_INFO("num_iter: %d", num_iter_);
    nh.getParam("num_lpr", num_lpr_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    nh.getParam("th_seeds", th_seeds_);
    ROS_INFO("th_seeds: %f", th_seeds_);
    nh.getParam("th_dist", th_dist_);
    ROS_INFO("th_dist: %f", th_dist_);

    pub_ground_     = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_  = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);

    g_seeds_pc      = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_ground_pc     = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_not_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);

    ros::spin();
}

PlaneGroundFilter::~PlaneGroundFilter() {  }

void PlaneGroundFilter::Spin()
{

}

void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted)
{
    // LPR is the mean of low point representative.
    double sum = 0;     //
    int cnt = 0;        // 点云计数

    // Calculate the mean height value. 因为已经排过序了，所以直接选取前最小的num_lpr个点即可
    for( int i=0; i<p_sorted.points.size() && cnt<num_lpr_; i ++ )
    {
        sum += p_sorted.points[i].z;
        cnt ++;
    }

    double lpr_height = cnt != 0 ? sum / cnt : 0;   // in case divide by 0
    g_seeds_pc->clear();

    // iterate pointcloud, filter those height is less than lpr.height + th_seeds_.  得到初始地面种子点
    for( int i=0; i<p_sorted.points.size(); i ++ )
    {
        if ( p_sorted.points[i].z < lpr_height + th_seeds_ )
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }

    // return seeds points
}


// 根据ax+by+cz+d = 0，拟合平面，最重要的是确定点与平面的投影距离阈值Th_dist，并以此判断点是否在平面上
void PlaneGroundFilter::estimate_plane_(void)
{
    // Create covariance matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;

    // 计算均值和协方差矩阵
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);

    // Singular Value Decomposition: SVD  奇异值分解
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);

    // use the least singular vector as normal.     取前三维作为主要方向：U矩阵m*m => m*r r=3  m是g_ground_pc点云的点数
    normal_ = (svd.matrixU().col(2));

    // mean ground seeds value.
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();     // 要XYZIR前三维，XYZ


    // according to normal.T * [x, y, z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threshold to 'th_dist - d'
    // 点云中的点到这个平面的正交投影距离小于阈值 Th_dist, 则认为该点属于地面，否则属于非地面
    th_dist_d_ = th_dist_ - d_;

    // return the equation parameters.
}


// 进一步滤除雷达过近处和过高处的点，因为雷达安装位置的原因，近处的车身反射会对Detection造成影响，需要滤除;
// 过高的目标，如大树、高楼，对于无人车的雷达感知意义也不大，我们对过近过高的点进行切除
void PlaneGroundFilter::clip_above(const pcl::PointCloud<VPoint>::Ptr in,
                                   const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;

#pragma omp for
    for( size_t i=0; i < in->points.size(); i ++ )
    {
        if( in->points[i].z > clip_height_ )
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);       // true to remove the indices
    cliper.filter(*out);
}

// 去除过近或者过远的点
void PlaneGroundFilter::remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,
                                            const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;
    pcl::PointIndices indices;

#pragma omp for
    for( size_t i=0; i < in->points.size(); i ++ )
    {
        double distance = sqrt(in->points[i].x*in->points[i].x + in->points[i].y*in->points[i].y);

        if( (distance < min_distance_) || (distance > max_distance_) )
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);       // true to remove the indices
    cliper.filter(*out);
}

// 后处理：进一步滤除雷达过近处和过高处的点
void PlaneGroundFilter::post_process(const pcl::PointCloud<VPoint>::Ptr in,
                                     const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::PointCloud<VPoint>::Ptr cliped_pc_str(new pcl::PointCloud<VPoint>);
    clip_above(in, cliped_pc_str);
    // pcl::PointCloud<VPoint>::Ptr remove_close(new pcl::PointCloud<VPoint>);
    remove_close_far_pt(cliped_pc_str, out);
}


// 主要处理函数
void PlaneGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    // 1. Msg to pointcloud  数据类型转换
    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn);

    pcl::PointCloud<VPoint> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn_org);
    // For mark ground points and hold all points.
    SLRPointXYZIRL point;

    // 将输入的所有点存放到g_all_pc中
    for(size_t i=0; i < laserCloudIn.points.size(); i ++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u;       // 0 means uncluster.
        g_all_pc->points.push_back(point);
    }
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);


    // 2. Sort on Z-axis value.  按照z轴数值大小对输入点云进行排序
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);


    // 3. Error point removal  清除一部分错误点
    // As there are some error mirror reflection under the ground,
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for( int i=0; i < laserCloudIn.points.size(); i ++ )
    {
        if( laserCloudIn.points[i].z < -1.5*sensor_height_ )
        {
            it ++;
        }
        else
        {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);


    // 4. Extract init ground seeds.  提取初始地面点
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;


    // 5. Ground plane fitter mainloop
    for(int i=0; i<num_iter_; i ++)
    {
        estimate_plane_();          // 用上面 估计的g_ground_pc点云 来拟合地平面
        g_ground_pc->clear();       // 准备重新来得到地面点云，用以进行下一次平面拟合
        //g_not_ground_pc->clear();

        // pointcloud to matrix  点云数据转换为矩阵存储 n*3维度表示
        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for(auto p : laserCloudIn_org.points)
        {
            points.row(j ++) << p.x, p.y, p.z;
        }

        // ground plane model  所有点与地平面法线的点乘，得到与地平面的距离
        VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); ++r)
        {
            if( result[r] < th_dist_d_ )    // 距离小于阈值的，就划分为地平面
            {
                g_all_pc->points[r].label = 1u;     // mean ground 表示地面，在所有点云中进行标签标识，加以区分
                g_ground_pc->points.push_back(laserCloudIn_org[r]);     // 单独存放地面点云
            }
            else
            {
                g_all_pc->points[r].label = 0u;     // mean not ground and non clustered
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]); // 单独存放非地面点云
            }
        }
    }

    // 6. 对去除地面的点云进行后处理
    pcl::PointCloud<VPoint>::Ptr final_no_ground(new pcl::PointCloud<VPoint>);
    post_process(g_not_ground_pc, final_no_ground);

    // ROS_INFO_STREAM("origin: "<<g_not_ground_pc->points.size()<<" post_process: "<<final_no_ground->points.size());

    // publish ground points        发布地面点云
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_ptr->header.stamp;           // 时间戳
    ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;     // 帧Id
    pub_ground_.publish(ground_msg);

    // publish not ground points    发布非地面点云
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*final_no_ground, groundless_msg);
    //pcl::toROSMsg(*g_not_ground_pc, groundless_msg);
    groundless_msg.header.stamp = in_cloud_ptr->header.stamp;           // 时间戳
    groundless_msg.header.frame_id = in_cloud_ptr->header.frame_id;     // 帧Id
    pub_no_ground_.publish(groundless_msg);

    // publish all points           发布所有点云
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_ptr->header.stamp;
    all_points_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_all_points_.publish(all_points_msg);
    g_all_pc->clear();
}

