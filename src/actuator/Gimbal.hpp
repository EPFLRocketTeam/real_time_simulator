#pragma once

#include "Actuator.hpp"
#include "real_time_simulator/Gimbal.h"
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>


#define DEG2RAD M_PI / 180

using namespace Eigen;
using namespace boost::numeric::odeint;
using stateType = Matrix<double, 5, 1>; // theta_1_dot, theta_2_dot, theta_1, theta_2, thrust

class Gimbal : public Actuator{

    public:

        ros::Timer integrator_thread;

        // Ranges of two gimbal angles and thrust
        Vector3d minRange;
        Vector3d maxRange;

        Vector3d positionCM; // Vector from center of mass to gimbal joint

        real_time_simulator::Gimbal gimbalCommand;
        stateType gimbalState;
        stateType gimbalStateOut;

        // Dynamic parameters (2nd order linear system)
        double inertia, damping, stifness, reactivity;

        double timestep;
        double previousTime;

        using stepper_type = runge_kutta_dopri5<stateType, double, stateType, double, vector_space_algebra>;
        stepper_type stepper;

        Gimbal(ros::NodeHandle &nh, double integrationTimestep){

            minRange << -10*DEG2RAD, -10*DEG2RAD, 500;
            maxRange << 10*DEG2RAD, 10*DEG2RAD, 1000;

            gimbalState << 0, 0, 0, 0, maxRange[2];

            positionCM << 0, 0, -2;

            inertia = 0.1;
            damping = 1;
            stifness = 2;
            reactivity = 15;

            actuatorPublisher = nh.advertise<real_time_simulator::Gimbal>("gimbal_state", 1);
            actuatorSubscriber = nh.subscribe("command_gimbal", 1,
                                            &Gimbal::gimbalCommandCallback, this);

            previousTime = 0;
            timestep = integrationTimestep;
            
            // Thread to integrate state. Duration defines interval time in seconds
            integrator_thread = nh.createTimer(
                    ros::Duration(timestep), [&](const ros::TimerEvent &) {
                        sendFeedback();
                    });

        }

        Actuator::control getActuatorWrench(const Eigen::Matrix<double, 14, 1> &rocketState){
            
            // Reduce thrust to zero when there are no propellant left
            if(rocketState[13] <= 0) gimbalCommand.thrust = 0;

            // Update state of actuator
            computeStateActuator();

            double outerAngle = gimbalState[2];
            double innerAngle = gimbalState[3];

            // Thrust is rotated by outer gimbal angle first (around body x axis),
            // then by inner gimbal angle (around rotated y axis)
            Vector3d thrustDirection;
            thrustDirection << sin(innerAngle),
                                -cos(innerAngle) * sin(outerAngle),
                                cos(innerAngle) * cos(outerAngle); 

            Vector3d thrustVector = thrustDirection * gimbalState[4];
                    
            
            Actuator::control gimbalWrench;
            gimbalWrench.col(0) = thrustVector;
            gimbalWrench.col(1) = positionCM.cross(thrustVector);

            //std::cout << gimbalWrench << std::endl << std::endl;

            return gimbalWrench;
        }

    private:

        void sendFeedback(){
            
            real_time_simulator::Gimbal gimbalFeedback;

            gimbalFeedback.outer_angle = gimbalState[2];
            gimbalFeedback.inner_angle = gimbalState[3];
            gimbalFeedback.thrust = gimbalState[4];

            actuatorPublisher.publish(gimbalFeedback);
        }

        void computeStateActuator(){

            auto dynamics = [this](const stateType &x , stateType &xdot , const double t) -> void {
                    actuatorDynamic(x, xdot, t);
                };

            stepper.do_step(dynamics, gimbalState, 0, gimbalStateOut, 0 + timestep);
            gimbalState = gimbalStateOut;

            previousTime += timestep;
        }

        // Dynamic equation of the gimbal
        void actuatorDynamic(const stateType &x , stateType &xdot , const double t){

            Vector2d angleCommand;
            angleCommand << gimbalCommand.outer_angle, gimbalCommand.inner_angle;
            
            // Angular acceleration proportional to angle error
            xdot.head(2) = ( stifness*(angleCommand - x.segment(2, 2)) - damping*x.head(2)) / inertia;

            xdot.segment(2, 2) = x.head(2);

            xdot[4] = reactivity*( gimbalCommand.thrust - x[4] );
        }

        // Callback function to store last received gimbal command
        void gimbalCommandCallback(const real_time_simulator::Gimbal::ConstPtr &command) {

            gimbalCommand.outer_angle = command->outer_angle;
            gimbalCommand.inner_angle = command->inner_angle;
            gimbalCommand.thrust = command->thrust;
        }


};