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

#include "rocket_utils/FSM.h"
#include "rocket_utils/State.h"
#include "rocket_utils/Sensor.h"

#include "rocket_utils/Control.h"

#include "std_msgs/String.h"
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


#include <chrono>
#include <random>
#include <vector>

#include "rocket.hpp"

using namespace std;
using namespace boost::numeric::odeint;
using namespace Eigen;

enum LaunchTriggerType{
    COMMAND,
    THRUST
};

class IntegratorNode {
private:
    typedef runge_kutta_dopri5<double> stepper_type;
    using stepper_type2 = runge_kutta_dopri5<Rocket::state, double, Rocket::state, double, vector_space_algebra>;
    stepper_type2 stepper;

    Rocket::state X;
    Rocket::state xout;

    Rocket rocket;

    //last updated aerodynamic force & torque
    Actuator::control aero_control;
    //last updated perturbations force & torque
    Actuator::control perturbation_control;
    //last requested fsm
    rocket_utils::FSM current_fsm;
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
    ros::Publisher rocket_forces_pub;
    ros::Publisher fsm_pub;

public:

    double integration_period = 1e-3;

    LaunchTriggerType launch_trigger_type;

    IntegratorNode(ros::NodeHandle &nh) {
        // Initialize publishers and subscribers
        initTopics(nh);

        /* ---------- Variable initialization  ---------- */
        // Initialize rocket class with useful parameters
        rocket.init(nh, integration_period);

        // Initialize fsm
        current_fsm.state_machine = "Idle";

        nh.param<double>("/environment/rail_length", rail_length, 0);


        std::string launch_trigger_type_string;
        nh.param<std::string>("launch_trigger_type", launch_trigger_type_string, "Thrust");

        if (launch_trigger_type_string == "Thrust") launch_trigger_type = LaunchTriggerType::THRUST;
        else if (launch_trigger_type_string == "Command") launch_trigger_type = LaunchTriggerType::COMMAND;
        else throw std::runtime_error("Invalid launch trigger type.");

        // Initialize external forces
        aero_control << 0, 0,
                        0, 0,
                        0, 0;

        //Get initial orientation and convert in Radians
        float roll = 0, zenith = 0, azimuth = 0.0;
        nh.getParam("/environment/rocket_roll", roll);
        nh.getParam("/environment/rail_zenith", zenith);
        nh.getParam("/environment/rail_azimuth", azimuth);

        roll *= M_PI / 180;
        zenith *= M_PI / 180;
        azimuth *= M_PI / 180;

        typedef EulerSystem<EULER_Z, EULER_Y, EULER_Z> Rail_system;
        typedef EulerAngles<double, Rail_system> angle_type;

        angle_type init_angle(azimuth, zenith, roll);

        Quaterniond q(init_angle);

        // Init state X
        X << 0, 0, 0,   0, 0, 0,   0.0, 0.0, 0.0, 1.0,   0.0, 0.0, 0.0,   rocket.propellant_mass;
        X.segment(6, 4) = q.coeffs();

        // Init sensors
        rocket.sensor_acc = (q.inverse())._transformVector(
                Eigen::Vector3d::UnitZ() * 3.986e14 / pow(6371e3 + rocket.h0, 2));
        rocket.sensor_gyro << 0.0, 0.0, 0.0;
        rocket.sensor_baro = 0;
	    rocket.sensor_mag << 1.0, 0.0, 0.0;
        xout = X;
    }

    void initTopics(ros::NodeHandle &nh) {
        // Subscribe to commands
        command_sub = nh.subscribe("commands", 10, &IntegratorNode::processCommand, this);

        // Subscribe to aero message
        rocket_aero_sub = nh.subscribe("rocket_aero", 100, &IntegratorNode::rocketAeroCallback, this);
        // Subscribe to perturbations message
        rocket_perturbation_sub = nh.subscribe("disturbance_pub", 1,
                                               &IntegratorNode::rocketPerturbationCallback, this);

        // Create state publisher
        rocket_state_pub = nh.advertise<rocket_utils::State>("rocket_state", 10);
        // Create fake sensors publisher
        rocket_sensor_pub = nh.advertise<rocket_utils::Sensor>("simu_sensor_pub", 10);
        // Create timer publisher and associated thread (100Hz)
        fsm_pub = nh.advertise<rocket_utils::FSM>("fsm_pub", 10);

        rocket_forces_pub = nh.advertise<rocket_utils::Control>("simu_actuator", 10);
    }

    void step() {
        // State machine ------------------------------------------
        if (current_fsm.state_machine.compare("Idle") == 0) {
            if (launch_trigger_type == LaunchTriggerType::THRUST){
                // Save accelerometer value as it is erased by dynamics_rail -> NOT VERY CLEAN
                Eigen::Vector3d static_acc = rocket.sensor_acc;
                
                // Compute force without integrating to detect liftoff
                rocket.updateActuators(X);
                Rocket::state xdot;
                rocket.dynamics_rail(X, xdot, aero_control, 0);
                double z_acc = xdot(5);
                if (z_acc > 0){
                    initLaunch();
                }
                rocket.sensor_acc = static_acc;
            }
        }
        else {
            rocket.updateActuators(X);
            if (current_fsm.state_machine.compare("Rail") == 0) {
                auto dynamics_rail = [this](const Rocket::state &x, Rocket::state &xdot, const double &t) -> void {
                    rocket.dynamics_rail(x, xdot, aero_control, t);
                };
                stepper.do_step(dynamics_rail, X, 0, xout, 0 + integration_period);

                // End of rail -> Launch state
                if (X(2) > rail_length) {
                    current_fsm.state_machine = "Launch";
                }
            } else if (current_fsm.state_machine.compare("Launch") == 0) {
                auto dynamics_flight = [this](const Rocket::state &x, Rocket::state &xdot, const double &t) -> void {
                    rocket.dynamics_flight(x, xdot, aero_control, perturbation_control, t);};

                stepper.do_step(dynamics_flight, X, 0, xout, 0 + integration_period);

                // End of burn -> no more thrust
                if (X(13) < 0) {
                    current_fsm.state_machine = "Coast";
                }
            } else if (current_fsm.state_machine.compare("Coast") == 0) {

                auto dynamics_flight = [this](const Rocket::state &x, Rocket::state &xdot, const double &t) -> void {
                    rocket.dynamics_flight(x, xdot, aero_control, perturbation_control, t);};

                stepper.do_step(dynamics_flight, X, 0, xout, 0 + integration_period);
            }

            X = xout;

            rocket.update_CM(X(13));
        }

        // Parse state and publish it on the /fast_rocket_state topic
        rocket_utils::State current_state;

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

        current_state.header.stamp = ros::Time::now();

        rocket_state_pub.publish(current_state);

        // Publish time + state machine
        fsm_pub.publish(current_fsm);

        //std::cout << "Fast integration time: " << 1000*(ros::Time::now().toSec()-time_now) << "ms \n";
    }

    // Callback function to store last received aero force and torque
    void rocketAeroCallback(const rocket_utils::Control::ConstPtr &rocket_aero) {
        aero_control << rocket_aero->force.x, rocket_aero->torque.x,
                rocket_aero->force.y, rocket_aero->torque.y,
                rocket_aero->force.z, rocket_aero->torque.z;
    }

    // Callback function to store last received aero force and torque
    void rocketPerturbationCallback(const rocket_utils::Control::ConstPtr &perturbation) {
        perturbation_control << perturbation->force.x, perturbation->torque.x,
                perturbation->force.y, perturbation->torque.y,
                perturbation->force.z, perturbation->torque.z;
    }

    void processCommand(const std_msgs::String &command) {
        if (command.data.compare("Coast") == 0) {
            current_fsm.state_machine = "Coast";
        } else if(launch_trigger_type == LaunchTriggerType::COMMAND) {
            //received launch command
            initLaunch();
        }
    }

    // Start the integration
    void initLaunch(){
        time_zero = ros::Time::now().toSec();
        if (rail_length == 0) current_fsm.state_machine = "Launch";
        else current_fsm.state_machine = "Rail";
    }

    void send_fake_sensor() {
        // construct a trivial random generator engine from a time-based seed:
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);

        std::normal_distribution<double> acc_noise(rocket.acc_bias, rocket.acc_noise);
        std::normal_distribution<double> gyro_noise(rocket.gyro_bias, rocket.gyro_noise);
        std::normal_distribution<double> baro_noise(rocket.baro_bias, rocket.baro_noise);

        std::normal_distribution<double> mag_noise(rocket.mag_bias, rocket.mag_noise);

        rocket_utils::Sensor sensor_msg;

        sensor_msg.header.stamp = ros::Time::now();

        sensor_msg.IMU_acc.x = rocket.sensor_acc(0) + acc_noise(generator);
        sensor_msg.IMU_acc.y = rocket.sensor_acc(1) + acc_noise(generator);
        sensor_msg.IMU_acc.z = rocket.sensor_acc(2) + acc_noise(generator);

        sensor_msg.IMU_gyro.x = rocket.sensor_gyro(0) + gyro_noise(generator);
        sensor_msg.IMU_gyro.y = rocket.sensor_gyro(1) + gyro_noise(generator);
        sensor_msg.IMU_gyro.z = rocket.sensor_gyro(2) + gyro_noise(generator);


        sensor_msg.IMU_mag.x = rocket.sensor_mag(0) + mag_noise(generator);
        sensor_msg.IMU_mag.y = rocket.sensor_mag(1) + mag_noise(generator);
        sensor_msg.IMU_mag.z = rocket.sensor_mag(2) + mag_noise(generator);

        sensor_msg.baro_height = rocket.sensor_baro + baro_noise(generator);

        rocket_sensor_pub.publish(sensor_msg);
    }

    void sendActuatorForce(){

        rocket_utils::Control actuatorMsg;

        actuatorMsg.force.x = rocket.rocket_control(0, 0);
        actuatorMsg.force.y = rocket.rocket_control(1, 0);
        actuatorMsg.force.z = rocket.rocket_control(2, 0);

        actuatorMsg.torque.x = rocket.rocket_control(0, 1);
        actuatorMsg.torque.y = rocket.rocket_control(1, 1);
        actuatorMsg.torque.z = rocket.rocket_control(2, 1);

        actuatorMsg.header.stamp = ros::Time::now();

        rocket_forces_pub.publish(actuatorMsg);
    }
};


int main(int argc, char **argv) {
    /* ---------- ROS intitialization ---------- */
    // Init ROS fast integrator node
    ros::init(argc, argv, "integrator");
    ros::NodeHandle nh;

    IntegratorNode integrator_node(nh);

    std::vector<double> timestep;

    // Thread to integrate state. Duration defines interval time in seconds
    ros::Timer integrator_thread = nh.createTimer(
            ros::Duration(integrator_node.integration_period), [&](const ros::TimerEvent &) {
                // double t0 = ros::Time::now().toSec();
                integrator_node.step();
                // timestep.push_back(ros::Time::now().toSec() - t0);
                //
                // if(timestep.size() > 1000) {
                //     std::cout << std::accumulate(timestep.begin(), timestep.end(),
                //                 decltype(timestep)::value_type(0)) << std::endl;
                //     timestep.clear();
                // }
            });

    double sensor_period;
    nh.getParam("/perturbation/sensor_period", sensor_period);
    ros::Timer sensor_thread = nh.createTimer(ros::Duration(sensor_period), [&](const ros::TimerEvent &) {
        integrator_node.send_fake_sensor();
    });


    ros::Timer actuator_thread = nh.createTimer(ros::Duration(20e-3), [&](const ros::TimerEvent &) {
        integrator_node.sendActuatorForce();
    });

    // Automatic callback of service and publisher from here
    ros::spin();
}
