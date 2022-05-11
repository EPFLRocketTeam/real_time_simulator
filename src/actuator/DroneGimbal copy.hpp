#pragma once

#include "Actuator.hpp"
#include "rocket_utils/GimbalControl.h"
#include "rocket_utils/Control.h"
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>


#define DEG2RAD M_PI / 180

using namespace Eigen;
using namespace boost::numeric::odeint;
using gimbalStateType = Matrix<double, 5, 1>; // theta_1, theta_1_dot, theta_2, theta_2_dot, thrust

class DroneGimbal : public Actuator{

    public:

        ros::Timer integrator_thread;

        // Ranges of two gimbal angles and thrust
        Vector3d minRange;
        Vector3d maxRange;
        Vector3d minVel;
        Vector3d maxVel;

        Vector3d positionCM; // Vector from center of mass to gimbal joint

        double K;

        rocket_utils::GimbalControl gimbalCommand;
        gimbalStateType gimbalState;
        gimbalStateType gimbalStateOut;

        // Dynamic parameters (2nd order linear system)
        double inertia, damping, stiffness, reactivity;

        double timestep;
        double previousTime;

        using stepper_type = runge_kutta_dopri5<gimbalStateType, double, gimbalStateType, double, vector_space_algebra>;
        stepper_type stepper;

        DroneGimbal(ros::NodeHandle &nh, double integrationTimestep, XmlRpc::XmlRpcValue gimbalParam){
            std::cout << "A" << std::endl;
            minRange << gimbalParam["minRange"][0], gimbalParam["minRange"][1], gimbalParam["minRange"][2];
            maxRange << gimbalParam["maxRange"][0], gimbalParam["maxRange"][1], gimbalParam["maxRange"][2];
            minRange.head(2) *= DEG2RAD;
            maxRange.head(2) *= DEG2RAD;
            std::cout << "B" << std::endl;
            minVel << gimbalParam["minVel"][0], gimbalParam["minVel"][1];
            maxVel << gimbalParam["maxVel"][0], gimbalParam["maxVel"][1];
            minVel.head(2) *= DEG2RAD;
            maxVel.head(2) *= DEG2RAD;
            std::cout << "C" << std::endl;
            K = gimbalParam["K"];
            std::cout << "C" << std::endl;
            gimbalState << 0, 0, 0, 0, 0;
            gimbalCommand.thrust = 0;

            positionCM << gimbalParam["positionCM"][0], gimbalParam["positionCM"][1], gimbalParam["positionCM"][2];

            

            // Create state and command message with gimbal ID
            int id = gimbalParam["id"];
            actuatorPublisher = nh.advertise<rocket_utils::GimbalControl>("drone_gimbal_state_"+std::to_string(id), 1);
            actuatorSubscriber = nh.subscribe("drone_gimbal_command_"+std::to_string(id), 1,
                                            &DroneGimbal::gimbalCommandCallback, this);

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
            if(rocketState[13] <= -1) gimbalCommand.thrust = 0;

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
            rocket_utils::GimbalControl gimbalStateMsg;
            gimbalStateMsg.header.stamp = ros::Time::now();

            gimbalStateMsg.outer_angle = gimbalState[0];
            gimbalStateMsg.inner_angle = gimbalState[2];
            gimbalStateMsg.thrust = gimbalState[4];

            actuatorPublisher.publish(gimbalStateMsg);
        }

        void computeStateActuator(){

            auto dynamics = [this](const gimbalStateType &x , gimbalStateType &xdot , const double t) -> void {
                    actuatorDynamic(x, xdot, t);
                };

            stepper.do_step(dynamics, gimbalState, 0, gimbalStateOut, 0 + timestep);
            gimbalState = gimbalStateOut;

            // gimbalState(2) = std::min(std::max(gimbalStateOut(2), minRange(0)), maxRange(0));
            // gimbalState(3) = std::min(std::max(gimbalStateOut(3), minRange(1)), maxRange(1));
            // gimbalState(4) = std::min(std::max(gimbalStateOut(4), minRange(2)), maxRange(2));

            previousTime += timestep;
        }

        // Dynamic equation of the gimbal
        void actuatorDynamic(const gimbalStateType &x , gimbalStateType &xdot , const double t){

            Vector2d angleCommand;
            angleCommand << gimbalCommand.outer_angle, gimbalCommand.inner_angle;
            angleCommand[0] = DEG2RAD * 200;
            
            
            xdot[0] = x[1];
            double t1(0);
            if(x[1] > 0) t1 = x[1] / K;
            // if (fabs(angleCommand[0] - x[0]) > a && (maxVel[0] - x[1]) < 1e-2) {xdot[1] = 0;std::cout << "----------------------------A";}
            // else if (fabs(angleCommand[0] - x[0]) > a && (maxVel[0]-x[1]  ) > 1e-2) {xdot[1] = K;std::cout << "#####B";}
            // else if (fabs(angleCommand[0] - x[0]) < a  && fabs(angleCommand[0] - x[0]) > 1e-2) {xdot[1] = -K;std::cout << "*****C";}
            // else {xdot[1] = 0; xdot[0] = 0;}
            if ((-K / 2. * t1 * t1 + x[1] * t1 + x[0] - angleCommand[0]) > 0  && fabs(angleCommand[0] - x[0]) > 0) {xdot[1] = -K; std::cout << "*************************C";}
            else if ((-K / 2. * t1 * t1 + x[1] * t1 + x[0] - angleCommand[0]) < 0 && fabs(maxVel[0] - x[1]) < 1e-2) {xdot[1] = 0;std::cout << "-------A";}
            else if ((-K / 2. * t1 * t1 + x[1] * t1 + x[0] - angleCommand[0]) < 0 && fabs(maxVel[0] - x[1]) > 1e-2) {xdot[1] = K;std::cout << "#####B";}
            else if ((angleCommand[0] - x[0]) < 0) {xdot[1] = 0; xdot[0] = 0;}
            // else {xdot[1] = 0; xdot[0] = 0;}
            xdot.segment(2,2) << 0,0;
            std::cout << std::endl << "a" <<x[0] << x[1] << std::endl;;  //<< x <<std::endl<< xdot <<std::endl<< maxVel <<std::endl;

            xdot[4] = reactivity*( gimbalCommand.thrust - x[4] );
        }

        // Callback function to store last received gimbal command
        void gimbalCommandCallback(const rocket_utils::GimbalControl::ConstPtr &command) {

            gimbalCommand.outer_angle = command->outer_angle;
            gimbalCommand.inner_angle = command->inner_angle;
            gimbalCommand.thrust = command->thrust;

            if(gimbalCommand.inner_angle > maxRange(0))  gimbalCommand.inner_angle = maxRange(0);
            if(gimbalCommand.inner_angle < minRange(0))  gimbalCommand.inner_angle = minRange(0);

            if(gimbalCommand.outer_angle > maxRange(1))  gimbalCommand.outer_angle = maxRange(1);
            if(gimbalCommand.outer_angle < minRange(1))  gimbalCommand.outer_angle = minRange(1);

            if(gimbalCommand.thrust > maxRange(2))  gimbalCommand.thrust = maxRange(2);
            if(gimbalCommand.thrust < minRange(2))  gimbalCommand.thrust = minRange(2);
        }


};