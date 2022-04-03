#pragma once

#include "ros/ros.h"

#include <Eigen/Dense>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

#include "real_time_simulator/Control.h"

class Actuator{

    public:

        using control = Eigen::Matrix<double, 3, 2>;

        // Subscriber to actuator command and publisher to actuator real-time state
        ros::Publisher actuatorPublisher;
        ros::Subscriber actuatorSubscriber;

        // void computeStateActuator();
        virtual control getActuatorWrench(const Eigen::Matrix<double, 14, 1> &rocketState){
            control wrench;
            return wrench;
        }

};