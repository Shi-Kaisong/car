#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace vio_odom {

class VioOdomNodelet : public nodelet::Nodelet {
public:
    VioOdomNodelet();

private:
    virtual void onInit();

    void loop_pose_callback(const nav_msgs::OdometryPtr &msg);
    void loop_pointclound_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_in);
    void car_odom_callback(const nav_msgs::OdometryPtr &msg);

    // Private attributes
    ros::Publisher pub_odom, pub_pointcloud;
    ros::Subscriber sub_pointcloud, sub_odom, sub_car_odom;

    Eigen::Vector3d t_vio_base_link; // The translation from VIO origin to base_link origin in VIO frame
    double linear_vx, linear_vy, angular_vz;
    double frame_z;
    geometry_msgs::Quaternion vio_quat;
    bool height_charge, inv;
};

} // namespace vio_odom
