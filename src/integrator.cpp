/*
* Node integrating differential equations of a rocket's flight to simulate position, velocity, quaternion, angular rate and mass  (14 states) in real time
* Also manage the finite state machine to coordinate other nodes
*
* Inputs: 
*   - Commands from the GUI to start the simulation:      \commands
*   - 3D force and torque from the rocket engine:         \control_pub
*   - 3D force and torque from the aerodynamics.py node:  \rocket_aero
*   - 3D force and torque from the disturbance.py node:   \disturbance_pub

* Parameters:
*   - Rocket model: 		/config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
*   - Sensor error model: /config/perturbations_parameters.yaml
*
* Outputs:
*   - Finite state machine and time since start of simulation : \fsm_pub
*   - Rocket full state (14 states) at 100 Hz:                  \rocket_state
*   - Simulated IMU and barometer sensor data:                  \sensor_pub
*
*/

#include "ros/ros.h"

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Sensor.h"

#include "real_time_simulator/Control.h"

#include "std_msgs/String.h"
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


#include <chrono>
#include <random>

#include "rocket.hpp"

using namespace std;
using namespace boost::numeric::odeint;
using namespace Eigen;

class IntegratorNode {
private:
    typedef runge_kutta_dopri5<double> stepper_type;
    using stepper_type2 = runge_kutta_dopri5<Rocket::state, double, Rocket::state, double, vector_space_algebra>;
    stepper_type2 stepper;

    Rocket::state X;
    Rocket::state xout;

    Rocket rocket;

    //last updated rocket force & torque
    Rocket::control rocket_control;
    //last updated aerodynamic force & torque
    Rocket::control aero_control;
    //last updated perturbations force & torque
    Rocket::control perturbation_control;
    //last requested fsm
    real_time_simulator::FSM current_fsm;
    double time_zero;
    double rail_length;

    // subscribers
    ros::Subscriber command_sub;
    ros::Subscriber rocket_control_sub;
    ros::Subscriber rocket_aero_sub;
    ros::Subscriber rocket_perturbation_sub;

    // publishers
    ros::Publisher rocket_state_pub;
    ros::Publisher rocket_sensor_pub;
    ros::Publisher fsm_pub;

    std::string start_trigger;
public:
    float integration_period = 10e-3;

    IntegratorNode(ros::NodeHandle &nh) {
        // Initialize publishers and subscribers
        initTopics(nh);

        /* ---------- Variable initialization  ---------- */
        // Initialize rocket class with useful parameters
        rocket.init(nh);

        // Initialize fsm
        current_fsm.time_now = 0;
        current_fsm.state_machine = real_time_simulator::FSM::IDLE;

        nh.param<double>("rail_length", rail_length, 0);

        nh.param<std::string>("start_trigger", start_trigger, "command");

        // Initialize external forces
        aero_control << 0, 0,
                0, 0,
                0, 0;

        rocket_control << 0, 0,
                0, 0,
                rocket.maxThrust[2], 0;

        //Get initial orientation and convert in Radians
        double roll = 0.0, zenith = 0.0, azimuth = 0.0;
        nh.param<double>("rocket_roll", roll, 0.0);
        nh.param<double>("rail_zenith", zenith, 0.0);
        nh.param<double>("rail_azimuth", azimuth, 0.0);

        roll *= M_PI / 180;
        zenith *= M_PI / 180;
        azimuth *= M_PI / 180;

        typedef EulerSystem<EULER_Z, EULER_Y, EULER_Z> Rail_system;
        typedef EulerAngles<double, Rail_system> angle_type;

        angle_type init_angle(azimuth, zenith, roll);

        Quaterniond q(init_angle);

        // Init state X
        X << 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, rocket.propellant_mass;
        X.segment(6, 4) = q.coeffs();

        // Init sensors
        rocket.sensor_acc = (q.inverse())._transformVector(
                Eigen::Vector3d::UnitZ() * 3.986e14 / pow(6371e3 + rocket.h0, 2));
        rocket.sensor_gyro << 0.0, 0.0, 0.0;
        rocket.sensor_baro = 0;

        xout = X;
    }

    void initTopics(ros::NodeHandle &nh) {
        // Subscribe to commands
        command_sub = nh.subscribe("/commands", 10, &IntegratorNode::processCommand, this);
        // Subscribe to control message from control node
        rocket_control_sub = nh.subscribe("/control_measured", 100,
                                          &IntegratorNode::rocketControlCallback, this);
        // Subscribe to aero message
        rocket_aero_sub = nh.subscribe("/rocket_aero", 100, &IntegratorNode::rocketAeroCallback, this);
        // Subscribe to perturbations message
        rocket_perturbation_sub = nh.subscribe("/disturbance_pub", 100,
                                               &IntegratorNode::rocketPerturbationCallback, this);

        // Create state publisher
        rocket_state_pub = nh.advertise<real_time_simulator::State>("/rocket_state", 10);
        // Create fake sensors publisher
        rocket_sensor_pub = nh.advertise<real_time_simulator::Sensor>("/simu_sensor_pub", 10);
        // Create timer publisher and associated thread (100Hz)
        fsm_pub = nh.advertise<real_time_simulator::FSM>("/fsm_pub", 10);
    }

    void step() {
        // State machine ------------------------------------------
        if (current_fsm.state_machine == real_time_simulator::FSM::IDLE) {

        } else {
            // Update current time
            current_fsm.time_now = ros::Time::now().toSec() - time_zero;

            if (current_fsm.state_machine == real_time_simulator::FSM::RAIL) {
                auto dynamics_rail = [this](const Rocket::state &x, Rocket::state &xdot, const double &t) -> void {
                    rocket.railDynamics(x, xdot, rocket_control, aero_control, t);
                };
                stepper.do_step(dynamics_rail, X, 0, xout, 0 + integration_period);

                // End of rail -> Launch state
                if (X(2) > rail_length) {
                    current_fsm.state_machine = real_time_simulator::FSM::LAUNCH;
                }
            } else if (current_fsm.state_machine == real_time_simulator::FSM::LAUNCH) {
                auto dynamics_flight = [this](const Rocket::state &x, Rocket::state &xdot, const double &t) -> void {
                    rocket.flightDynamics(x, xdot, rocket_control, aero_control, perturbation_control, t);
                };
                stepper.do_step(dynamics_flight, X, 0, xout, 0 + integration_period);

                // End of burn -> no more thrust
                if (X(13) < 0) {
                    current_fsm.state_machine = real_time_simulator::FSM::COAST;
                }
            } else if (current_fsm.state_machine == real_time_simulator::FSM::COAST) {
                rocket_control << 0, 0,
                        0, 0,
                        0, 0;
                auto dynamics_flight = [this](const Rocket::state &x, Rocket::state &xdot, const double &t) -> void {
                    rocket.flightDynamics(x, xdot, rocket_control, aero_control, perturbation_control, t);
                };
                stepper.do_step(dynamics_flight, X, 0, xout, 0 + integration_period);
            }

            X = xout;

            rocket.updateCM(X(13));
        }

        // Parse state and publish it on the /fast_rocket_state topic
        real_time_simulator::State current_state;

        current_state.pose.position.x = X(0);
        current_state.pose.position.y = X(1);
        current_state.pose.position.z = X(2);

        current_state.twist.linear.x = X(3);
        current_state.twist.linear.y = X(4);
        current_state.twist.linear.z = X(5);

        current_state.pose.orientation.x = X(6);
        current_state.pose.orientation.y = X(7);
        current_state.pose.orientation.z = X(8);
        current_state.pose.orientation.w = X(9);

        current_state.twist.angular.x = X(10);
        current_state.twist.angular.y = X(11);
        current_state.twist.angular.z = X(12);

        current_state.propeller_mass = X(13);

        rocket_state_pub.publish(current_state);

        // Publish time + state machine    
        fsm_pub.publish(current_fsm);

        //std::cout << "Fast integration time: " << 1000*(ros::Time::now().toSec()-time_now) << "ms \n";
    }

// Callback function to store last received control
    void rocketControlCallback(const real_time_simulator::Control::ConstPtr &control_law) {
        rocket_control << control_law->force.x, control_law->torque.x,
                control_law->force.y, control_law->torque.y,
                control_law->force.z, control_law->torque.z;
        if (current_fsm.state_machine == real_time_simulator::FSM::IDLE && start_trigger == "Control") {
            time_zero = ros::Time::now().toSec();
            current_fsm.state_machine = real_time_simulator::FSM::LAUNCH;
        }
    }

// Callback function to store last received aero force and torque
    void rocketAeroCallback(const real_time_simulator::Control::ConstPtr &rocket_aero) {
        aero_control << rocket_aero->force.x, rocket_aero->torque.x,
                rocket_aero->force.y, rocket_aero->torque.y,
                rocket_aero->force.z, rocket_aero->torque.z;
    }

// Callback function to store last received aero force and torque
    void rocketPerturbationCallback(const real_time_simulator::Control::ConstPtr &perturbation) {
        perturbation_control << perturbation->force.x, perturbation->torque.x,
                perturbation->force.y, perturbation->torque.y,
                perturbation->force.z, perturbation->torque.z;
    }

    void processCommand(const std_msgs::String &command) {
        if (command.data == "Coast") {
            current_fsm.state_machine = real_time_simulator::FSM::COAST;
        } else if (command.data == "Stop") {
            current_fsm.state_machine = real_time_simulator::FSM::STOP;
        }
        else if (start_trigger == "Command") {
            //received launch command
            time_zero = ros::Time::now().toSec();
            if (rail_length == 0) current_fsm.state_machine = real_time_simulator::FSM::LAUNCH;
            else current_fsm.state_machine = real_time_simulator::FSM::RAIL;
        }
    }

void sendFakeSensor() {
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> acc_noise(rocket.acc_bias, rocket.acc_noise);
    std::normal_distribution<double> gyro_noise(rocket.gyro_bias, rocket.gyro_noise);
    std::normal_distribution<double> baro_noise(rocket.baro_bias, rocket.baro_noise);

    real_time_simulator::Sensor sensor_msg;

    sensor_msg.IMU_acc.x = rocket.sensor_acc(0) + acc_noise(generator);
    sensor_msg.IMU_acc.y = rocket.sensor_acc(1) + acc_noise(generator);
    sensor_msg.IMU_acc.z = rocket.sensor_acc(2) + acc_noise(generator);

    sensor_msg.IMU_gyro.x = rocket.sensor_gyro(0) + gyro_noise(generator);
    sensor_msg.IMU_gyro.y = rocket.sensor_gyro(1) + gyro_noise(generator);
    sensor_msg.IMU_gyro.z = rocket.sensor_gyro(2) + gyro_noise(generator);

    sensor_msg.baro_height = rocket.sensor_baro + baro_noise(generator);

    rocket_sensor_pub.publish(sensor_msg);
}

};


int main(int argc, char **argv) {
    /* ---------- ROS intitialization ---------- */
    // Init ROS fast integrator node
    ros::init(argc, argv, "integrator");
    ros::NodeHandle nh("integrator");

    IntegratorNode integrator_node(nh);

    // Thread to integrate state. Duration defines interval time in seconds
    ros::Timer integrator_thread = nh.createTimer(
            ros::Duration(integrator_node.integration_period), [&](const ros::TimerEvent &) {
                integrator_node.step();
            });

    double sensor_period;
    nh.getParam("/perturbation/sensor_period", sensor_period);
    ros::Timer sensor_thread = nh.createTimer(ros::Duration(sensor_period), [&](const ros::TimerEvent &) {
        integrator_node.sendFakeSensor();
    });

    // Automatic callback of service and publisher from here
    ros::spin();
}
