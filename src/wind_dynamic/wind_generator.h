/**
 * @file wind_generator.h
 * @author Sven Becker (sven.becker@epfl.ch)
 * @brief Implementation of chapter 4.1 of https://drive.google.com/file/d/1psKMbYIDg3n1MyOD7myFiBktjy46THxa/view?usp=sharing
 * @version 1.0
 * @date 2022-09-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef _WIND_GENERATION_HEADER_
#define _WIND_GENERATION_HEADER_ 
#include "ros/ros.h"
#include "dryden_model.h"
#include "geometry_msgs/Vector3.h"
#include "Eigen/Geometry"
#include "atmosphere.hpp"
#include "real_time_simulator/SetWind.h"

enum WindType {
    // No wind-speed
    OFF,
    // Wind in static direction of static magnitude
    STATIC,
    // Directed wind of stochastic direction and magnitude (Dryden + discrete gust superimposed)
    REALISTIC 
};

class WindGenerator{
    public:
        /**
         * @brief Construct a new Wind Generator object
         * 
         * @param dt Period at wind is published (and wind models are evaluated)
         * @param nh Nodehandle
         * @param altitude Altitude above ground in m (for model equations)
         */
        WindGenerator(double dt, ros::NodeHandle nh, double altitude);

    private:
        
        ros::NodeHandle nh;

        // Controls when to evaluate the wind models and to publish their results
        ros::Timer main_timer;

        // Publish wind-data
        ros::Publisher wind_publisher;

        // Change parameters of wind model or wind model itself
        ros::ServiceServer wind_setter;

        double dt;

        double altitude;

        // Wind model according to Dryden
        dryden_model::DrydenWind wind_dryden;

        // Wind model according to discrete gust model
        atmosphere_models::turbulence_models::discrete::DiscreteGustModel wind_gust;

        // Components of wind that are static or are (almost) reached once-in-a-while with discrete gust model
        geometry_msgs::Vector3 wind_components;

        // Components of Dryden wind noise
        geometry_msgs::Vector3 noise_components;

    /**
     * @brief Change current wind according to service request
     * 
     * @param req Wind type, vector of wind magnitude  (not relevant for type "OFF"), and noise components (only relevant for type "REALISTIC")
     * @param res -
     * @return true always
     * @return false never
     */
    bool setWindCallback(real_time_simulator::SetWindRequest& req, real_time_simulator::SetWindResponse& res);

    
    /**
     * @brief Set the desired model for publishing wind data according to mode. Uses values stored in instance(e.g., from setWindCallback) to configure models.
     * 
     * @param wind_type One of enum WindType's members
     * @return true If model could successfully be set
     * @return false  If model could not successfully be set
     */
    bool setWindModel(uint8_t wind_type);

};
#endif //_WIND_GENERATION_HEADER_
