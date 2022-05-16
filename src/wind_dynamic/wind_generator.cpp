#include "wind_generator.hpp"


WindGenerator::WindGenerator(double dt, ros::NodeHandle nh, double altitude = 2.0){
    this->nh = nh;
    this->dt = dt;
    this->altitude = altitude;
    this->wind_publisher = this->nh.advertise<geometry_msgs::Vector3>("/wind_speed", 10, false);
    this->wind_setter = this->nh.advertiseService("set_wind", &WindGenerator::setWindCallback, this);
    real_time_simulator::SetWindRequest req;
    real_time_simulator::SetWindResponse res;
    req.wind_type = WindType::OFF;
    this->setWindCallback(req, res);

    // Getting wind parameters
    double windSpeed, windDirection, windVariability;
    nh.getParam("/environment/wind_speed", windSpeed);
    nh.getParam("/environment/wind_direction", windDirection);
    nh.getParam("/environment/wind_variation", windVariability);

    windDirection *= M_PI/180;

    // Setting wind speed in NED frame
    wind_components.x = windSpeed*cos(windDirection);
    wind_components.y = windSpeed*sin(windDirection);
    wind_components.z = 0;

    // Setting variability of wind in NED frame
    noise_components.x = windVariability*windSpeed*cos(windDirection);
    noise_components.y = windVariability*windSpeed*sin(windDirection);
    noise_components.z = 0;

    setWindModel(WindType::REALISTIC);
}
    

bool WindGenerator::setWindCallback(real_time_simulator::SetWindRequest& req, real_time_simulator::SetWindResponse& res){
    this->main_timer.stop();
    wind_components = req.wind_components;
    noise_components = req.noise_components;
    
    setWindModel(req.wind_type);
    
    return true;
}

bool WindGenerator::setWindModel(uint8_t wind_type){

    switch (wind_type){
        case WindType::OFF:
        {
            wind_components.x = 0;
            wind_components.y = 0;
            wind_components.z = 0;
            this->main_timer = nh.createTimer(ros::Duration(this->dt), [&](const ros::TimerEvent&){
                this->wind_publisher.publish(wind_components);
            });
            break;
        }
            
        case WindType::STATIC:
        {
            this->main_timer = nh.createTimer(ros::Duration(this->dt), [&](const ros::TimerEvent&){
                this->wind_publisher.publish(wind_components);
            });
            break;
        }

        case WindType::REALISTIC:
        {
            dryden_model::DrydenWind wind_dryden;
            wind_dryden.initialize(0.0,0.0,0.0, noise_components.x, noise_components.y, noise_components.z, this->altitude);
            atmosphere_models::turbulence_models::discrete::DiscreteGustModel wind_gust;
            wind_gust.setup(
                this->dt, std::atan2(wind_components.y, wind_components.x) * 180.0 / M_PI, 
                std::sqrt(wind_components.x*wind_components.x+wind_components.y*wind_components.y)
                );
            this->wind_dryden = wind_dryden;
            this->wind_gust = wind_gust;
            this->main_timer = this->nh.createTimer(ros::Duration(this->dt), [&](const ros::TimerEvent&){
                Eigen::Vector3d wind_vector_dryden = this->wind_dryden.getWind(this->dt);
                Eigen::Vector3d wind_vector_gust;
                this->wind_gust.getGustVelNED(wind_vector_gust);
                
                Eigen::Vector3d wind_vector = wind_vector_dryden + wind_vector_gust;
                geometry_msgs::Vector3 wind_msg;
                wind_msg.x = wind_vector.x();
                wind_msg.y = wind_vector.y();
                wind_msg.z = 0; // wind_vector.z();
                this->wind_publisher.publish(wind_msg);
            });
            break;
        }
            
        
        default:
            ROS_WARN_STREAM("Unrecognized wind type number " << wind_type);
            return false;
    }
    return true;
}


int main(int argc, char **argv){
    double dt = 10e-3;
    
    ros::init(argc, argv, "wind_generator");
    ros::NodeHandle nh("wind_generator");
    WindGenerator wg(dt, nh);
    ros::spin();
}