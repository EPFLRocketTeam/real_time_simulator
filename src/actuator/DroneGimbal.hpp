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

        Vector2d K={};
        Vector2d u={};
        Matrix<double,3,3> inertia_i, inertia_o;

        rocket_utils::GimbalControl gimbalCommand;
        gimbalStateType gimbalState;
        gimbalStateType gimbalStateOut;

        // Dynamic parameters (2nd order linear system)
        double inertia, damping, stiffness, reactivity;

        double timestep;
        double propellerRadius;
        double previousTime;
        int setPointDirection[2] = {1,1};

        using stepper_type = runge_kutta_dopri5<gimbalStateType, double, gimbalStateType, double, vector_space_algebra>;
        stepper_type stepper;

        DroneGimbal(ros::NodeHandle &nh, double integrationTimestep, XmlRpc::XmlRpcValue gimbalParam){
            minRange << gimbalParam["minRange"][0], gimbalParam["minRange"][1], gimbalParam["minRange"][2];
            maxRange << gimbalParam["maxRange"][0], gimbalParam["maxRange"][1], gimbalParam["maxRange"][2];
            minRange.head(2) *= DEG2RAD;
            maxRange.head(2) *= DEG2RAD;            
            minVel << gimbalParam["minVel"][0], gimbalParam["minVel"][1];
            maxVel << gimbalParam["maxVel"][0], gimbalParam["maxVel"][1];
            minVel *= DEG2RAD;
            maxVel *= DEG2RAD;
            K << gimbalParam["K"]["outer"], gimbalParam["K"]["inner"];
            K *= DEG2RAD;
            reactivity = gimbalParam["reactivity"];
            propellerRadius = gimbalParam["propellerRadius"];
            gimbalState << 0, 0, 0, 0, 0;
            u << 0, 0;
            gimbalCommand.inner_angle = 0;
            gimbalCommand.outer_angle = 0;
            gimbalCommand.thrust = 0;
            for(size_t i=0; i<9; i++){
                inertia_i(i/3, i%3) = gimbalParam["inertia"]["inner"][i];
                inertia_o(i/3, i%3) = gimbalParam["inertia"]["outer"][i];
            }


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
            
            
            


            Actuator::control gimbalWrench = computeActuatorWrench(rocketState);

            return gimbalWrench;
        }

    private:

        Actuator::control computeActuatorWrench(const Eigen::Matrix<double, 14, 1> &rocketState){

            double outerAngle = gimbalState[0];
            double innerAngle = gimbalState[2];

            // Thrust is rotated by outer gimbal angle first (around body x axis),
            // then by inner gimbal angle (around rotated y axis)
            Vector3d thrustDirection;
            thrustDirection << sin(innerAngle),
                                -cos(innerAngle) * sin(outerAngle),
                                cos(innerAngle) * cos(outerAngle); 
                                

            Vector3d thrustVector = thrustDirection * gimbalState[4];
            Vector3d omega_i, omega_o, alpha_i, alpha_o;
            Vector3d Mi, Mo;
            Matrix3d Ry, Ry90, Rx;

            // See report for deduction (the identifiers here are identical to the formulas' symbols)
            Ry = AngleAxisd(gimbalState(2), Vector3d::UnitY());
            Ry90 = AngleAxisd(gimbalState(2) + M_PI / 2, Vector3d::UnitY());
            Rx = AngleAxisd(gimbalState(0), Vector3d::UnitX());
            
            omega_o << gimbalState(1), 0, 0;
            alpha_o << u(0), 0, 0;

            omega_i << 0, gimbalState(2), 0;
            omega_i += Ry.transpose() * omega_o;
            alpha_i << 0, u(1), 0;
            alpha_i += Ry90.transpose() * gimbalState(3) * omega_o + Ry.transpose() * alpha_o;

            Mi = inertia_i * alpha_i + omega_i.cross(inertia_i * omega_i);
            Mo = Ry * Mi + inertia_o * alpha_o + omega_o.cross(inertia_o * omega_o);

            
            Actuator::control gimbalWrench;
            Matrix3d R_body_to_world = Quaterniond(rocketState(9,0),rocketState(6,0),rocketState(7,0),rocketState(8,0)).toRotationMatrix();
            double z = rocketState(2) + R_body_to_world.row(2) * positionCM - positionCM(2);
            if (4*z>propellerRadius){
                Vector3d thrustVectorW; 
                thrustVectorW = R_body_to_world * thrustVector;
                thrustVectorW(2) /= 1-std::pow(propellerRadius / (4 * z), 2);
                thrustVector = R_body_to_world.transpose() * thrustVectorW;
            }



            gimbalWrench.col(0) = thrustVector;
            gimbalWrench.col(1) = positionCM.cross(thrustVector) + Rx * Mo;
            
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
            Vector2d angleCommand;
            angleCommand << gimbalCommand.outer_angle, gimbalCommand.inner_angle;

            
            
            double t1;

            // Inner Servo
            t1 = 0.;
            if(setPointDirection[0] < 0) gimbalState.head(2) = -1 * gimbalState.head(2);
            if(gimbalState(1) > 0) t1 = gimbalState(1) / K(0);
            if ((-K(0) / 2. * t1 * t1 + gimbalState(1) * t1 + gimbalState(0) - angleCommand[0]) > 0  && t1 > 1e-5) u(0) = -K(0);
            else if ((-K(0) / 2. * t1 * t1 + gimbalState(1) * t1 + gimbalState(0) - angleCommand[0]) < 0 && (maxVel[0] - gimbalState(1)) < 0) u(0) = 0;
            else if ((-K(0) / 2. * t1 * t1 + gimbalState(1) * t1 + gimbalState(0) - angleCommand[0]) < 0 && (maxVel[0] - gimbalState(1)) > 0) u(0) = K(0);
            else if ((angleCommand[0] - gimbalState(0)) < 0) {u(0) = 0; gimbalState(1) = 0;}
            //std::cout << maxVel[0] << std::endl<< gimbalState(1) << std::endl;

            // Outer Servo
            t1 = 0.;
            if(setPointDirection[1] < 0) gimbalState.segment(2,2) = -1 * gimbalState.segment(2,2);
            if(gimbalState(3) > 0) t1 = gimbalState(3) / K(1);
            if ((-K(1) / 2. * t1 * t1 + gimbalState(3) * t1 + gimbalState(2) - angleCommand[1]) > 0  && t1 > 1e-5) u(1) = -K(1);
            else if ((-K(1) / 2. * t1 * t1 + gimbalState(3) * t1 + gimbalState(2) - angleCommand[1]) < 0 && (maxVel[1] - gimbalState(3)) < 0) u(1) = 0;
            else if ((-K(1) / 2. * t1 * t1 + gimbalState(3) * t1 + gimbalState(2) - angleCommand[1]) < 0 && (maxVel[1] - gimbalState(3)) > 0) u(1) = K(1);
            else if ((angleCommand[1] - gimbalState(2)) < 0) {u(1) = 0; gimbalState(3) = 0;}
            // std::cout << u << std::endl;
            // std::cout << gimbalState << std::endl;
            // return;
            auto dynamics = [this](const gimbalStateType &x , gimbalStateType &xdot , const double t) -> void {
                                actuatorDynamic(x, xdot, t, u);
                            };
            stepper.do_step(dynamics, gimbalState, 0, gimbalStateOut, 0 + timestep);
            gimbalState = gimbalStateOut;

            if(setPointDirection[0] < 0) gimbalState.head(2) = -1 * gimbalState.head(2);
            if(setPointDirection[1] < 0) gimbalState.segment(2,2) = -1 * gimbalState.segment(2,2);

            // gimbalState(2) = std::min(std::max(gimbalStateOut(2), minRange(0)), maxRange(0));
            // gimbalState(3) = std::min(std::max(gimbalStateOut(3), minRange(1)), maxRange(1));
            // gimbalState(4) = std::min(std::max(gimbalStateOut(4), minRange(2)), maxRange(2));

            previousTime += timestep;
        }

        // Dynamic equation of the gimbal
        void actuatorDynamic(const gimbalStateType &x , gimbalStateType &xdot , const double t, Vector2d& u){

            xdot[0] = x[1];
            xdot[1] = u(0);
            xdot[2] = x[3];
            xdot[3] = u(1);           

            xdot[4] = reactivity*( gimbalCommand.thrust - x[4] );
        }

        // Callback function to store last received gimbal command
        void gimbalCommandCallback(const rocket_utils::GimbalControl::ConstPtr &command) {

            gimbalCommand.outer_angle = command->outer_angle;
            gimbalCommand.inner_angle = command->inner_angle;
            gimbalCommand.thrust = command->thrust;
            if (gimbalCommand.inner_angle < gimbalState(2)){
                setPointDirection[1] = -1;
                gimbalCommand.inner_angle *= -1;
            }
            else setPointDirection[1] = 1;
            if (gimbalCommand.outer_angle < gimbalState(0)){
                setPointDirection[0] = -1;
                gimbalCommand.outer_angle *= -1;
            }
            else setPointDirection[0] = 1;


            if(gimbalCommand.inner_angle > maxRange(0))  gimbalCommand.inner_angle = maxRange(0);
            if(gimbalCommand.inner_angle < minRange(0))  gimbalCommand.inner_angle = minRange(0);

            if(gimbalCommand.outer_angle > maxRange(1))  gimbalCommand.outer_angle = maxRange(1);
            if(gimbalCommand.outer_angle < minRange(1))  gimbalCommand.outer_angle = minRange(1);

            if(gimbalCommand.thrust > maxRange(2))  gimbalCommand.thrust = maxRange(2);
            if(gimbalCommand.thrust < minRange(2))  gimbalCommand.thrust = minRange(2);
        }


};