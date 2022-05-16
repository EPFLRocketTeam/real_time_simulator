#pragma once

#include "Actuator.hpp"
#include "rocket_utils/ControlMomentGyro.h"
#include "rocket_utils/Control.h"
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>


#define DEG2RAD M_PI / 180

using namespace Eigen;
using namespace boost::numeric::odeint;
using stateType = Matrix<double, 5, 1>; // theta_1_dot, theta_2_dot, theta_1, theta_2, torque

class ControlMomentGyro : public Actuator{

    public:

        ros::Timer integrator_thread;

        // Ranges of two CMG angles and torque
        Vector3d minRange;
        Vector3d maxRange;

        Vector3d positionCM; // Vector from center of mass to CMG joint

        rocket_utils::ControlMomentGyro cmgCommand;
        stateType cmgState;
        stateType cmgStateOut;

        // Dynamic parameters (2nd order linear system)
        double inertia, damping, stiffness, reactivity;

        double timestep;
        double previousTime;

        using stepper_type = runge_kutta_dopri5<stateType, double, stateType, double, vector_space_algebra>;
        stepper_type stepper;

        ControlMomentGyro(ros::NodeHandle &nh, double integrationTimestep, XmlRpc::XmlRpcValue cmgParam){

            minRange << cmgParam["minRange"][0], cmgParam["minRange"][1], cmgParam["minRange"][2];
            maxRange << cmgParam["maxRange"][0], cmgParam["maxRange"][1], cmgParam["maxRange"][2];
            minRange.head(2) *= DEG2RAD;
            maxRange.head(2) *= DEG2RAD;

            cmgState << 0, 0, 0, 0, 0;

            positionCM << cmgParam["positionCM"][0], cmgParam["positionCM"][1], cmgParam["positionCM"][2];

            inertia = cmgParam["inertia"];
            damping = cmgParam["damping"];
            stiffness = cmgParam["stiffness"];
            reactivity = cmgParam["reactivity"];

            // Create state and command message with CMG ID
            int id = cmgParam["id"];
            actuatorPublisher = nh.advertise<rocket_utils::ControlMomentGyro>("cmg_state_"+std::to_string(id), 1);
            actuatorSubscriber = nh.subscribe("cmg_command_"+std::to_string(id), 1,
                                            &ControlMomentGyro::cmgCommandCallback, this);

            previousTime = 0;
            timestep = integrationTimestep;
            
            // Thread to integrate CMG state. Duration defines interval time in seconds
            double sensorFrequency = cmgParam["sensorFrequency"];

            integrator_thread = nh.createTimer(
                    ros::Duration(1.0/sensorFrequency), [&](const ros::TimerEvent &) {
                        sendFeedback();
                    });
        }

        Actuator::control getActuatorWrench(const Eigen::Matrix<double, 14, 1> &rocketState){

            // Update state of actuator
            computeStateActuator();

            Actuator::control cmgWrench = computeActuatorWrench();

            return cmgWrench;
        }

    private:

        Actuator::control computeActuatorWrench(){

            double outerAngle = cmgState[2];
            double innerAngle = cmgState[3];

            // Torque is rotated by outer CMG angle first (around body x axis),
            // then by inner CMG angle (around rotated y axis)
            Vector3d torqueDirection;
            torqueDirection << sin(innerAngle),
                                -cos(innerAngle) * sin(outerAngle),
                                cos(innerAngle) * cos(outerAngle); 

            // Torque felt by rocket is the opposite of torque applied to CMG !
            Vector3d torqueVector = (-1) * torqueDirection * cmgState[4];
                    
            
            Actuator::control cmgWrench;
            cmgWrench.setZero();
            cmgWrench.col(1) = torqueVector;

            return cmgWrench;
        }

        void sendFeedback(){

            // Send latest CMG state as feedback
            rocket_utils::ControlMomentGyro cmgStateMsg;

            cmgStateMsg.outer_angle = cmgState[2];
            cmgStateMsg.inner_angle = cmgState[3];
            cmgStateMsg.torque = cmgState[4];

            cmgStateMsg.header.stamp = ros::Time::now();

            actuatorPublisher.publish(cmgStateMsg);
        }

        void computeStateActuator(){

            auto dynamics = [this](const stateType &x , stateType &xdot , const double t) -> void {
                    actuatorDynamic(x, xdot, t);
                };

            stepper.do_step(dynamics, cmgState, 0, cmgStateOut, 0 + timestep);
            cmgState = cmgStateOut;

            cmgState(2) = std::min(std::max(cmgStateOut(2), minRange(0)), maxRange(0));
            cmgState(3) = std::min(std::max(cmgStateOut(3), minRange(1)), maxRange(1));
            cmgState(4) = std::min(std::max(cmgStateOut(4), minRange(2)), maxRange(2));

            previousTime += timestep;
        }

        // Dynamic equation of the CMG
        void actuatorDynamic(const stateType &x , stateType &xdot , const double t){

            Vector2d angleCommand;
            angleCommand << cmgCommand.outer_angle, cmgCommand.inner_angle;
            
            // Angular acceleration proportional to angle error
            xdot.head(2) = ( stiffness*(angleCommand - x.segment(2, 2)) - damping*x.head(2)) / inertia;

            xdot.segment(2, 2) = x.head(2);

            xdot[4] = reactivity*( cmgCommand.torque - x[4] );
        }

        // Callback function to store last received CMG command
        void cmgCommandCallback(const rocket_utils::ControlMomentGyro::ConstPtr &command) {

            cmgCommand.outer_angle = command->outer_angle;
            cmgCommand.inner_angle = command->inner_angle;
            cmgCommand.torque = command->torque;

            if(cmgCommand.inner_angle > maxRange(0))  cmgCommand.inner_angle = maxRange(0);
            if(cmgCommand.inner_angle < minRange(0))  cmgCommand.inner_angle = minRange(0);

            if(cmgCommand.outer_angle > maxRange(1))  cmgCommand.outer_angle = maxRange(1);
            if(cmgCommand.outer_angle < minRange(1))  cmgCommand.outer_angle = minRange(1);

            if(cmgCommand.torque > maxRange(2))  cmgCommand.torque = maxRange(2);
            if(cmgCommand.torque < minRange(2))  cmgCommand.torque = minRange(2);
        }


};