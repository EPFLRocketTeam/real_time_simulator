#include "ros/ros.h"

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Sensor.h"

#include "real_time_simulator/Control.h"

#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"

#include <time.h>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <chrono>
#include <random>

using namespace Eigen;

class Rocket {
public:
    float dry_mass;
    float propellant_mass;
    float Isp;
    float minTorque;
    float maxTorque;

    std::vector<float> maxThrust{0, 0, 0};
    std::vector<float> minThrust{0, 0, 0};

    float dry_CM;
    float propellant_CM;
    float total_CM; // Current Cm of rocket, in real time

    float total_length;

    float initial_speed;

    float h0;

    std::vector<float> target_apogee = {0, 0, 0};
    std::vector<float> Cd = {0, 0, 0};
    std::vector<float> surface = {0, 0, 0};
    std::vector<float> drag_coeff = {0, 0, 0};

    std::vector<float> dry_Inertia{0, 0, 0};
    std::vector<float> total_Inertia{0, 0, 0};

    std::vector<float> J_inv{0, 0, 0};

    // Sensor data
    Vector3d sensor_acc;
    Vector3d sensor_gyro;
    float sensor_baro;

    float acc_noise, acc_bias;
    float gyro_noise, gyro_bias;
    float baro_noise, baro_bias;

    using state = Matrix<double, 14, 1>;
    using control = Matrix<double, 3, 2>;

    void updateCM(float current_prop_mass) {
        total_CM = total_length - (dry_CM * dry_mass + propellant_CM * current_prop_mass) /
                                  (dry_mass + current_prop_mass); // From aft of rocket

        float new_inertia = dry_Inertia[0] + pow(total_CM - (total_length - propellant_CM), 2) * current_prop_mass;

        total_Inertia[0] = new_inertia;
        total_Inertia[1] = new_inertia;

        J_inv[0] = total_CM / total_Inertia[0];
        J_inv[1] = total_CM / total_Inertia[1];
        J_inv[2] = 1 / total_Inertia[2];
    }


    void init(ros::NodeHandle n) {
        n.getParam("/rocket/minTorque", minTorque);
        n.getParam("/rocket/maxTorque", maxTorque);
        n.getParam("/rocket/maxThrust", maxThrust);
        n.getParam("/rocket/minThrust", minThrust);
        n.getParam("/rocket/Isp", Isp);

        n.getParam("/rocket/dry_mass", dry_mass);
        n.getParam("/rocket/propellant_mass", propellant_mass);

        n.getParam("/rocket/Cd", Cd);
        n.getParam("/rocket/dry_I", dry_Inertia);

        n.getParam("/rocket/dry_CM", dry_CM);
        n.getParam("/rocket/propellant_CM", propellant_CM);

        n.getParam("/environment/apogee", target_apogee);

        n.getParam("/rocket/initial_speed", initial_speed);

        std::vector<float> diameter = {0, 0, 0};
        std::vector<float> length = {0, 0, 0};

        int nStage;

        n.getParam("/rocket/diameters", diameter);
        n.getParam("/rocket/stage_z", length);
        n.getParam("/rocket/stages", nStage);

        total_length = length[nStage - 1];

        surface[0] = diameter[1] * total_length;
        surface[1] = surface[0];
        surface[2] = diameter[1] * diameter[1] / 4 * 3.14159;

        float rho_air = 1.225;
        drag_coeff[0] = 0.5 * rho_air * surface[0] * Cd[0];
        drag_coeff[1] = 0.5 * rho_air * surface[1] * Cd[1];
        drag_coeff[2] = 0.5 * rho_air * surface[2] * Cd[2];

        total_Inertia[2] = dry_Inertia[2];

        updateCM(propellant_mass);

        n.getParam("/perturbation/acc_noise", acc_noise);
        n.getParam("/perturbation/acc_bias", acc_bias);

        n.getParam("/perturbation/gyro_noise", gyro_noise);
        n.getParam("/perturbation/gyro_bias", gyro_bias);

        n.getParam("/perturbation/baro_noise", baro_noise);
        n.getParam("/perturbation/baro_bias", baro_bias);

        n.getParam("/environment/ground_altitude", h0);

    }

    void flightDynamics(const state &x,
                        state &xdot,
                        control &rocket_control,
                        control &aero_control,
                        control &perturbation_control,
                        const double &t) {
        // -------------- Simulation variables -----------------------------
        double g0 = 3.986e14 / pow(6371e3 + h0 + x(2), 2);  // Earth gravity in [m/s^2]

        double mass = dry_mass + x(13);                  // Instantaneous mass of the rocket in [kg]

        // Orientation of the rocket with quaternion
        Quaternion<double> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();
        //std::cout << (180/3.14)*std::acos(x(9)*x(9) - x(6)*x(6) - x(7)*x(7) + x(8)*x(8)) << "\n";


        // Force in inertial frame: gravity
        Matrix<double, 3, 1> gravity;
        gravity << 0, 0, g0 * mass;

        // Total force in inertial frame [N]
        Matrix<double, 3, 1> total_force;
        total_force = rot_matrix * rocket_control.col(0) - gravity + aero_control.col(0) + perturbation_control.col(0);
        //std::cout << total_force.transpose() << "\n";


        // Angular velocity omega in quaternion format to compute quaternion derivative
        Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));
        //std::cout << x.segment(10,3).transpose()*57.29 << "\n\n";

        // Tortal torque in body frame
        Matrix<double, 3, 1> I_inv;
        I_inv << 1 / total_Inertia[0], 1 / total_Inertia[1], 1 / total_Inertia[2];

        Matrix<double, 3, 1> total_torque;
        total_torque =
                rocket_control.col(1) + rot_matrix.transpose() * (aero_control.col(1) + perturbation_control.col(1));

        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed variation is Force/mass
        xdot.segment(3, 3) = total_force / mass;

        // Quaternion variation is 0.5*w◦q
        xdot.segment(6, 4) = 0.5 * (omega_quat * attitude).coeffs();

        // Angular speed variation is Torque/Inertia
        xdot.segment(10, 3) = rot_matrix * (total_torque.cwiseProduct(I_inv));

        // Mass variation is proportional to total thrust
        if (Isp != -1) {
            xdot(13) = -rocket_control.col(0).norm() / (Isp * g0);
        } else {
            xdot(13) = 0;
        }


        // Fake sensor data update -----------------
        sensor_acc = rot_matrix.transpose() * (total_force + gravity) / mass;

        sensor_gyro = rot_matrix.transpose() * x.segment(10, 3);

        sensor_baro = x(2);
    }


    void railDynamics(const state &x,
                      state &xdot,
                      control &rocket_control,
                      control &aero_control,
                      const double &t) {
        // -------------- Simulation variables -----------------------------
        double g0 = 3.986e14 / pow(6371e3 + x(2), 2);  // Earth gravity in [m/s^2]

        double mass = dry_mass + x(13);     // Instantaneous mass of the rocket in [kg]

        // Orientation of the rocket with quaternion
        Quaternion<double> attitude(x(9), x(6), x(7), x(8));
        attitude.normalize();
        Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Force in inertial frame: gravity
        Matrix<double, 3, 1> gravity;
        gravity << 0, 0, g0 * mass;

        // Total force in initial body frame [N] (rail frame)
        Matrix<double, 3, 1> total_force;
        total_force = rocket_control.col(0) - rot_matrix.transpose() * (gravity + aero_control.col(0));

        Matrix<double, 3, 1> body_acceleration;

        total_force.head(2) << 0.0, 0.0; // Zero force on axes perpendicular to rail to force rocket to stay on rail
        //std::cout << total_force << "\n";


        // Angular velocity omega in quaternion format to compute quaternion derivative
        Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));

        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3, 3);

        // Speed variation is Force/mass
        xdot.segment(3, 3) = rot_matrix * total_force / mass;

        // Quaternion variation is zero to keep rail orientation
        xdot.segment(6, 4) << 0.5 * (omega_quat * attitude).coeffs();

        // Angular speed variation is zero to keep rail orientation
        xdot.segment(10, 3) << 0.0, 0.0, 0.0;

        // Mass variation is proportional to total thrust
        xdot.tail(1) << -rocket_control.col(0).norm() / (Isp * g0);

        // Fake sensor data update -----------------
        sensor_acc = (total_force + rot_matrix.transpose() * gravity) / mass;

        sensor_gyro << 0.0, 0.0, 0.0;

        sensor_baro = x(2);
    }
};
