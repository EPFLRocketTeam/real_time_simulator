#include "ros/ros.h"
#include "wind-dynamics/dryden_model.h"
#include "geometry_msgs/Vector3.h"
#include "Eigen/Geometry"
#include "atmosphere.hpp"
#include "real_time_simulator/SetWind.h"
#include "real_time_simulator/WindType.h"
#define dt 0.01
int main(int argc, char **argv){
    dryden_model::DrydenWind wind_dryden;
    wind_dryden.initialize(0.4, 0.4, 0.4, 0.5, 0.5, 0.5, 2.0);
    ros::init(argc, argv, "wind_generator");
    ros::NodeHandle nh("wind_generator");
    ros::Publisher dryden_wind_pub = nh.advertise<geometry_msgs::Vector3>("/wind", 1, false);


    ros::Timer dryden_timer = nh.createTimer(ros::Duration(dt), [&](const ros::TimerEvent&){
        Eigen::Vector3d wind_vector = wind_dryden.getWind(dt);
        geometry_msgs::Vector3 wind_msg;
        wind_msg.x = wind_vector.x();
        wind_msg.y = wind_vector.y();
        wind_msg.z = wind_vector.z();
        dryden_wind_pub.publish(wind_msg);
    }, false, true);

    atmosphere_models::turbulence_models::discrete::DiscreteGustModel wind_gust;
    wind_gust.setup(dt, 135, 0.4);
    ros::Publisher gust_wind_pub = nh.advertise<geometry_msgs::Vector3>("wind_gust", 1, false);
    ros::Timer gust_timer = nh.createTimer(ros::Duration(dt), [&](const ros::TimerEvent&){
        Eigen::Vector3d wind_vector;
        wind_gust.getGustVelNED(wind_vector);
        geometry_msgs::Vector3 wind_msg;
        wind_msg.x = wind_vector.x();
        wind_msg.y = wind_vector.y();
        wind_msg.z = wind_vector.z();
        gust_wind_pub.publish(wind_msg);
    }, false, true);
    ros::spin();
}