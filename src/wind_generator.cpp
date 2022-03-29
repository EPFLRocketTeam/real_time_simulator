#include "ros/ros.h"
#include "wind-dynamics/dryden_model.h"
#include "geometry_msgs/Vector3.h"
#include "Eigen/Geometry"
#include "atmosphere.hpp"
#include "real_time_simulator/SetWind.h"
#include "real_time_simulator/WindType.h"



class WindGenerator{
    public:
    WindGenerator(double dt, ros::NodeHandle nh, double altitude = 2.0){
        this->nh = nh;
        this->dt = dt;
        this->altitude = altitude;
        this->wind_publisher = this->nh.advertise<geometry_msgs::Vector3>("/wind_speed", 10, false);
        this->wind_setter = this->nh.advertiseService("set_wind", &WindGenerator::setWindCallback, this);
        real_time_simulator::SetWindRequest req;
        real_time_simulator::SetWindResponse res;
        req.wind_type = real_time_simulator::WindType::OFF;
        this->setWindCallback(req, res);
    }
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

    bool setWindCallback(real_time_simulator::SetWindRequest& req, real_time_simulator::SetWindResponse& res){
        this->main_timer.stop();
        wind_components = req.wind_components;
        noise_components = req.noise_components;
        switch (req.wind_type)
        {
        case real_time_simulator::WindType::OFF:
        {
            wind_components.x = 0;
            wind_components.y = 0;
            wind_components.z = 0;
            this->main_timer = nh.createTimer(ros::Duration(this->dt), [&](const ros::TimerEvent&){
                this->wind_publisher.publish(wind_components);
            });
        }
            
        case real_time_simulator::WindType::STATIC:
            this->main_timer = nh.createTimer(ros::Duration(this->dt), [&](const ros::TimerEvent&){
                this->wind_publisher.publish(wind_components);
            });
        case real_time_simulator::WindType::REALISTIC:
        {
            dryden_model::DrydenWind wind_dryden;
            wind_dryden.initialize(0.0,0.0,0.0, noise_components.x, noise_components.y, noise_components.z, this->altitude);
            atmosphere_models::turbulence_models::discrete::DiscreteGustModel wind_gust;
            wind_gust.setup(
                this->dt, std::atan2(wind_components.x, wind_components.y) * 180.0 / M_PI, 
                std::sqrt(wind_components.x*wind_components.x+wind_components.y*wind_components.y)
                );
            this->wind_dryden = wind_dryden;
            this->wind_gust = wind_gust;
            this->main_timer = this->nh.createTimer(ros::Duration(this->dt), [&](const ros::TimerEvent&){
                Eigen::Vector3d wind_vector_dryden = wind_dryden.getWind(this->dt);
                Eigen::Vector3d wind_vector_gust;
                wind_gust.getGustVelNED(wind_vector_gust);
                
                Eigen::Vector3d wind_vector = wind_vector_dryden + wind_vector_gust;
                geometry_msgs::Vector3 wind_msg;
                wind_msg.x = wind_vector.x();
                wind_msg.y = wind_vector.y();
                wind_msg.z = wind_vector.z();
                this->wind_publisher.publish(wind_msg);
            });
        }
            
        
        default:
            ROS_WARN_STREAM("Unrecognized wind type number " << req.wind_type);
            return false;
           
        }
        return true;
    }

};


int main(int argc, char **argv){
    double dt = 0.1;
    
    ros::init(argc, argv, "wind_generator");
    ros::NodeHandle nh("wind_generator");
    WindGenerator wg(dt, nh);
    ros::spin();
}