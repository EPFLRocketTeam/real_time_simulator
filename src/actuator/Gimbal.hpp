#pragma once

#include "Actuator.hpp"
#include "real_time_simulator/Gimbal.h"
#include "real_time_simulator/Control.h"
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
        double inertia, damping, stiffness, reactivity;

        double timestep;
        double previousTime;

        using stepper_type = runge_kutta_dopri5<stateType, double, stateType, double, vector_space_algebra>;
        stepper_type stepper;

        Gimbal(ros::NodeHandle &nh, double integrationTimestep, XmlRpc::XmlRpcValue gimbalParam){

            minRange << gimbalParam["minRange"][0], gimbalParam["minRange"][1], gimbalParam["minRange"][2];
            maxRange << gimbalParam["maxRange"][0], gimbalParam["maxRange"][1], gimbalParam["maxRange"][2];
            minRange.head(2) *= DEG2RAD;
            maxRange.head(2) *= DEG2RAD;

            gimbalState << 0, 0, 0, 0, maxRange[2];
            gimbalCommand.thrust = maxRange[2];

            positionCM << gimbalParam["positionCM"][0], gimbalParam["positionCM"][1], gimbalParam["positionCM"][2];

            inertia = gimbalParam["inertia"];
            damping = gimbalParam["damping"];
            stiffness = gimbalParam["stiffness"];
            reactivity = gimbalParam["reactivity"];

            // Create state and command message with gimbal ID
            int id = gimbalParam["id"];
            actuatorPublisher = nh.advertise<real_time_simulator::Gimbal>("gimbal_state_"+std::to_string(id), 1);
            actuatorSubscriber = nh.subscribe("gimbal_command_"+std::to_string(id), 1,
                                            &Gimbal::gimbalCommandCallback, this);

            previousTime = 0;
            timestep = integrationTimestep;
            
            // Thread to integrate gimbal state. Duration defines interval time in seconds
            double sensorFrequency = gimbalParam["sensorFrequency"];

            integrator_thread = nh.createTimer(
                    ros::Duration(1.0/sensorFrequency), [&](const ros::TimerEvent &) {
                        sendFeedback();
                    });

        }

        Actuator::control getActuatorWrench(const Eigen::Matrix<double, 14, 1> &rocketState){
            
            // Reduce thrust to zero when there are no propellant left
            if(rocketState[13] <= 0) gimbalCommand.thrust = 0;

            // Update state of actuator
            computeStateActuator();

            Actuator::control gimbalWrench = computeActuatorWrench();

            return gimbalWrench;
        }

    private:

        Actuator::control computeActuatorWrench(){

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

            return gimbalWrench;
        }

        void sendFeedback(){

            // Send latest gimbal state as feedback
            real_time_simulator::Gimbal gimbalStateMsg;

            gimbalStateMsg.outer_angle = gimbalState[2];
            gimbalStateMsg.inner_angle = gimbalState[3];
            gimbalStateMsg.thrust = gimbalState[4];

            actuatorPublisher.publish(gimbalStateMsg);
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
            xdot.head(2) = ( stiffness*(angleCommand - x.segment(2, 2)) - damping*x.head(2)) / inertia;

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