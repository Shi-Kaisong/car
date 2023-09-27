#include <ros/ros.h>
#include <nodelet/loader.h>
#include "vio_odom_nodelet.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vio_odom");
    ros::NodeHandle private_nh("~");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    // 加载VioOdomNodelet
    std::string nodelet_name_vio_odom = ros::this_node::getName();
    nodelet.load(nodelet_name_vio_odom, "vio_odom/VioOdomNodelet", remap, nargv);

    boost::shared_ptr<ros::MultiThreadedSpinner> spinner;
    
    spinner.reset(new ros::MultiThreadedSpinner(2));

    spinner->spin();
    return 0;
}
