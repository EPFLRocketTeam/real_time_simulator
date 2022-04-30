#pragma once

#include "ros/ros.h"
#include "dryden_model.h"
#include "geometry_msgs/Vector3.h"
#include "Eigen/Geometry"
#include "atmosphere.hpp"
#include "rocket_utils/SetWind.h"

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

    bool setWindCallback(rocket_utils::SetWindRequest& req, rocket_utils::SetWindResponse& res);

    bool setWindModel(uint8_t wind_type);

};

