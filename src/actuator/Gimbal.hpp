#include <Eigen/Dense>

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

        Vector3d minRange;
        Vector3d maxRange;

        Vector3d positionCM;

        real_time_simulator::Gimbal gimbalCommand;
        stateType gimbalState;
        stateType gimbalStateOut;

        double inertia, damping, stifness, reactivity;

        double timestep;
        double previousTime;

        using stepper_type = runge_kutta_dopri5<stateType, double, stateType, double, vector_space_algebra>;
        stepper_type stepper;

        Gimbal(ros::NodeHandle &nh){

            minRange << -10*DEG2RAD, -10*DEG2RAD, 500;
            maxRange << 10*DEG2RAD, 10*DEG2RAD, 1000;

            positionCM << 0, 0, 2;

            inertia = 0.1;
            damping = 1;
            stifness = 2;
            reactivity = 15;

            actuatorPublisher = nh.advertise<real_time_simulator::Gimbal>("gimbal_state", 1);
            actuatorSubscriber = nh.subscribe("command_gimbal", 1,
                                            &Gimbal::gimbalCommandCallback, this);

            timestep = 1.0/50;
            
            // Thread to integrate state. Duration defines interval time in seconds
            integrator_thread = nh.createTimer(
                    ros::Duration(timestep), [&](const ros::TimerEvent &) {
                        sendFeedback();
                    });

        }

    //private:

        void sendFeedback(){

            computeStateActuator();
            real_time_simulator::Gimbal gimbalFeedback;

            gimbalFeedback.inner_angle = gimbalState[2];
            gimbalFeedback.outer_angle = gimbalState[3];
            gimbalFeedback.thrust = gimbalState[4];

            actuatorPublisher.publish(gimbalFeedback);
        }

    private:    

        void computeStateActuator(){

            auto dynamics = [this](const stateType &x , stateType &xdot , const double t) -> void {
                    actuatorDynamic(x, xdot, t);
                };

            stepper.do_step(dynamics, gimbalState, 0, gimbalStateOut, 0 + 0.1);
            gimbalState = gimbalStateOut;
        }

        // Dynamic equation of the gimbal
        void actuatorDynamic(const stateType &x , stateType &xdot , const double t){

            Vector2d angleCommand;
            angleCommand << gimbalCommand.inner_angle, gimbalCommand.outer_angle;
            
            // Angular acceleration proportional to angle error
            xdot.head(2) = ( stifness*(angleCommand - x.segment(2, 2)) - damping*x.head(2)) / inertia;

            xdot.segment(2, 2) = x.head(2);

            xdot[4] = reactivity*( gimbalCommand.thrust - x[4] );
        }

        // Callback function to store last received gimbal command
        void gimbalCommandCallback(const real_time_simulator::Gimbal::ConstPtr &command) {

            gimbalCommand.inner_angle = command->inner_angle;
            gimbalCommand.outer_angle = command->outer_angle;
            gimbalCommand.thrust = command->thrust;
        }




};