#pragma once

#include "ros/ros.h"
#include "dryden_model.h"
#include "geometry_msgs/Vector3.h"
#include "Eigen/Geometry"
#include "atmosphere.hpp"
#include "real_time_simulator/SetWind.h"

enum WindType {
    OFF,
    STATIC,
    REALISTIC 
};

class WindGenerator{
    public:

        WindGenerator(double dt, ros::NodeHandle nh, double altitude);

    private:

        ros::NodeHandle nh;
        ros::Timer main_timer;
        ros::Publisher wind_publisher;
        ros::ServiceServer wind_setter;
        double dt;
        double altitude;

        dryden_model::DrydenWind wind_dryden;
        atmosphere_models::turbulence_models::discrete::DiscreteGustModel wind_gust;

        geometry_msgs::Vector3 wind_components;
        geometry_msgs::Vector3 noise_components;

    bool setWindCallback(real_time_simulator::SetWindRequest& req, real_time_simulator::SetWindResponse& res);

    bool setWindModel(uint8_t wind_type);

};

