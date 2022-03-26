#include "ros/ros.h"
#include "wind-dynamics/dryden_model.h"
#include "geometry_msgs/Vector3.h"
#include "Eigen/Geometry"
#define dt 0.01

int main(int argc, char **argv){
    dryden_model::DrydenWind wind;
    wind.initialize(0.4, 0.4, 0.4, 0.5, 0.5, 0.5, 2.0);
    ros::init(argc, argv, "wind_generator");
    ros::NodeHandle nh("wind_generator");
    ros::Publisher wind_pub = nh.advertise<geometry_msgs::Vector3>("wind", 1, false);
    ros::Timer timer = nh.createTimer(ros::Duration(dt), [&](const ros::TimerEvent&){
        Eigen::Vector3d wind_vector = wind.getWind(dt);
        geometry_msgs::Vector3 wind_msg;
        wind_msg.x = wind_vector.x();
        wind_msg.y = wind_vector.y();
        wind_msg.z = wind_vector.z();
        wind_pub.publish(wind_msg);
    }, false, true);
    ros::spin();
}