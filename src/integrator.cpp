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

class Rocket
{
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

    std::vector<float> target_apogee = {0, 0, 0};
    std::vector<float> Cd = {0, 0, 0};
    std::vector<float> surface = {0, 0, 0};
    std::vector<float> drag_coeff = {0, 0, 0};

    std::vector<float> dry_Inertia{0, 0, 0};
    std::vector<float> total_Inertia{0, 0, 0};

    std::vector<float> J_inv{0, 0, 0};

    // Sensor data
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float baro;

    float acc_noise, acc_bias;
    float gyro_noise, gyro_bias;
    float baro_noise, baro_bias;


    void update_CM( float current_prop_mass)
    {
      total_CM = total_length - (dry_CM*dry_mass + propellant_CM*current_prop_mass)/(dry_mass+current_prop_mass) ; // From aft of rocket

      float new_inertia = dry_Inertia[0] + pow(total_CM-(total_length-propellant_CM), 2)*current_prop_mass;

      total_Inertia[0] = new_inertia;
      total_Inertia[1] = new_inertia;

      J_inv[0] = total_CM/total_Inertia[0];
      J_inv[1] = total_CM/total_Inertia[1];
      J_inv[2] = 1/total_Inertia[2];
    }


    void init(ros::NodeHandle n)
    {
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

      total_length = length[nStage-1];

      surface[0] = diameter[1]*total_length;
      surface[1] = surface[0];
      surface[2] = diameter[1]*diameter[1]/4 * 3.14159;

      float rho_air = 1.225;
      drag_coeff[0] = 0.5*rho_air*surface[0]*Cd[0];
      drag_coeff[1] = 0.5*rho_air*surface[1]*Cd[1];
      drag_coeff[2] = 0.5*rho_air*surface[2]*Cd[2];

      total_Inertia[2] = dry_Inertia[2];

      update_CM(propellant_mass);

      n.getParam("/perturbation/acc_noise", acc_noise);
      n.getParam("/perturbation/acc_bias", acc_bias);

      n.getParam("/perturbation/gyro_noise", gyro_noise);
      n.getParam("/perturbation/gyro_bias", gyro_bias);

      n.getParam("/perturbation/baro_noise", baro_noise);
      n.getParam("/perturbation/baro_bias", baro_bias);
    }
};

Rocket rocket;

// Global variable with last updated rocket force & torque
Eigen::Matrix<double, 3,2> rocket_control;

//Global variable with last updated aerodynamic force & torque
Eigen::Matrix<double, 3,2> aero_control;

//Global variable with last updated perturbations force & torque
Eigen::Matrix<double, 3,2> perturbation_control;

// Global variable with last requested fsm
real_time_simulator::FSM current_fsm;
double time_zero;

// Callback function to store last received control
void rocket_controlCallback(const real_time_simulator::Control::ConstPtr& control_law)
{
  rocket_control << control_law->force.x,  control_law->torque.x,
                    control_law->force.y,  control_law->torque.y,
                    control_law->force.z,  control_law->torque.z;
}

// Callback function to store last received aero force and torque
void rocket_aeroCallback(const real_time_simulator::Control::ConstPtr& rocket_aero)
{
  aero_control <<   rocket_aero->force.x,  rocket_aero->torque.x,
                    rocket_aero->force.y,  rocket_aero->torque.y,
                    rocket_aero->force.z,  rocket_aero->torque.z;
}

// Callback function to store last received aero force and torque
void rocket_perturbationCallback(const real_time_simulator::Control::ConstPtr& perturbation)
{
  perturbation_control <<   perturbation->force.x,  perturbation->torque.x,
                            perturbation->force.y,  perturbation->torque.y,
                            perturbation->force.z,  perturbation->torque.z;
}

void send_fake_sensor(ros::Publisher rocket_sensor_pub)
{
  // construct a trivial random generator engine from a time-based seed:
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);

  std::normal_distribution<double> acc_noise (rocket.acc_bias, rocket.acc_noise);
  std::normal_distribution<double> gyro_noise (rocket.gyro_bias, rocket.gyro_noise);
  std::normal_distribution<double> baro_noise (rocket.baro_bias, rocket.baro_noise);

  real_time_simulator::Sensor sensor_msg;

  sensor_msg.IMU_acc.x = rocket.accX + acc_noise(generator);
  sensor_msg.IMU_acc.y = rocket.accY + acc_noise(generator);
  sensor_msg.IMU_acc.z = rocket.accZ + acc_noise(generator);

  sensor_msg.IMU_gyro.x = rocket.gyroX + gyro_noise(generator);
  sensor_msg.IMU_gyro.y = rocket.gyroY + gyro_noise(generator);
  sensor_msg.IMU_gyro.z = rocket.gyroZ + gyro_noise(generator);

  sensor_msg.baro_height = rocket.baro + baro_noise(generator);

  rocket_sensor_pub.publish(sensor_msg);
}


using namespace std;
using namespace boost::numeric::odeint;



template<typename scalar_t>
using state_t = Eigen::Matrix<scalar_t, 14, 1>;

using state = state_t<double>;

template<typename scalar_t>
using control_t = Eigen::Matrix<scalar_t, 4, 1>;

void dynamics_flight(const state& x, state& xdot, const double &t)
{
  // -------------- Simulation variables -----------------------------
  double g0 = 3.986e14/pow(6371e3+x(2), 2);  // Earth gravity in [m/s^2]

  double mass = rocket.dry_mass + x(13);                  // Instantaneous mass of the rocket in [kg]

  // Orientation of the rocket with quaternion
  Eigen::Quaternion<double> attitude( x(9), x(6), x(7), x(8)); attitude.normalize();
  Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();


  // Force in inertial frame: gravity
  Eigen::Matrix<double, 3, 1> gravity; gravity << 0, 0, g0*mass;

  // Total force in inertial frame [N]
  Eigen::Matrix<double, 3, 1> total_force;  total_force = rot_matrix*rocket_control.col(0) - gravity + aero_control.col(0) + perturbation_control.col(0);
  //std::cout << total_force.transpose() << "\n";


  // Angular velocity omega in quaternion format to compute quaternion derivative
  Eigen::Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));
  //std::cout << x.segment(10,3).transpose()*57.29 << "\n\n";

  // Tortal torque in body frame
  Eigen::Matrix<double, 3, 1> I_inv; I_inv << 1/rocket.total_Inertia[0], 1/rocket.total_Inertia[1], 1/rocket.total_Inertia[2];

  Eigen::Matrix<double, 3, 1> total_torque;
  total_torque = rocket_control.col(1) + rot_matrix.transpose()*(aero_control.col(1) + perturbation_control.col(1));

  // -------------- Differential equation ---------------------

  // Position variation is speed
  xdot.head(3) = x.segment(3,3);

  // Speed variation is Force/mass
  xdot.segment(3,3) = total_force/mass;

  // Quaternion variation is 0.5*w◦q
  xdot.segment(6, 4) =  0.5*(omega_quat*attitude).coeffs();

  // Angular speed variation is Torque/Inertia
  xdot.segment(10, 3) = rot_matrix*(total_torque.cwiseProduct(I_inv));

  // Mass variation is proportional to total thrust
  if(rocket.Isp != -1){
      xdot(13) = -rocket_control.col(0).norm()/(rocket.Isp*g0);
  }
  else{
      xdot(13) = 0;
  }


  // Fake sensor data update -----------------
  Eigen::Matrix<double, 3, 1> body_acceleration;
  body_acceleration = rot_matrix.transpose()*(total_force+gravity)/mass;
  rocket.accX = body_acceleration(0);
  rocket.accY = body_acceleration(1);
  rocket.accZ = body_acceleration(2);

  Eigen::Matrix<double, 3, 1> body_gyroscope;
  body_gyroscope = rot_matrix.transpose()*x.segment(10,3);

  rocket.gyroX = body_gyroscope(0);
  rocket.gyroY = body_gyroscope(1);
  rocket.gyroZ = body_gyroscope(2);

  rocket.baro = x(2);
  }


void dynamics_rail(const state& x, state& xdot, const double &t)
{
  // -------------- Simulation variables -----------------------------
  double g0 = 3.986e14/pow(6371e3+x(2), 2);  // Earth gravity in [m/s^2]

  double mass = rocket.dry_mass + x(13);     // Instantaneous mass of the rocket in [kg]

  // Orientation of the rocket with quaternion
  Eigen::Quaternion<double> attitude( x(9), x(6), x(7), x(8)); attitude.normalize();
  Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();

  // Force in inertial frame: gravity
  Eigen::Matrix<double, 3, 1> gravity; gravity << 0, 0, g0*mass;

  // Total force in initial body frame [N] (rail frame)
  Eigen::Matrix<double, 3, 1> total_force;  total_force = rocket_control.col(0) - rot_matrix.transpose()*(gravity + aero_control.col(0));

  Eigen::Matrix<double, 3, 1> body_acceleration;

  total_force.head(2) << 0.0, 0.0; // Zero force on axes perpendicular to rail to force rocket to stay on rail
  //std::cout << total_force << "\n";


  // Angular velocity omega in quaternion format to compute quaternion derivative
  Eigen::Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));

  // -------------- Differential equation ---------------------

  // Position variation is speed
  xdot.head(3) = x.segment(3,3);

  // Speed variation is Force/mass
  xdot.segment(3,3) = rot_matrix*total_force/mass;

  // Quaternion variation is zero to keep rail orientation
  xdot.segment(6, 4) << 0.5*(omega_quat*attitude).coeffs();

  // Angular speed variation is zero to keep rail orientation
  xdot.segment(10, 3) << 0.0, 0.0, 0.0;

  // Mass variation is proportional to total thrust
  xdot(13) = -rocket_control.col(0).norm()/(rocket.Isp*g0);

  // Fake sensor data update -----------------
  body_acceleration = (total_force + rot_matrix.transpose()*gravity)/mass; // Here gravity is measured by the accelerometer because of the rail's reaction force
  rocket.accX = body_acceleration(0);
  rocket.accY = body_acceleration(1);
  rocket.accZ = body_acceleration(2);

  rocket.gyroX = 0.0;
  rocket.gyroY = 0.0;
  rocket.gyroZ = 0.0;

  rocket.baro = x(2);
}

float rail_length = 0;

void processCommand(const std_msgs::String &command){
    if(command.data.compare("Coast") == 0){
        current_fsm.state_machine = "Coast";
    } else{
        //received launch command
        time_zero = ros::Time::now().toSec();
        if (rail_length==0) current_fsm.state_machine = "Launch";
        else current_fsm.state_machine = "Rail";
    }
}


typedef runge_kutta_dopri5< double > stepper_type;
using stepper_type2 = runge_kutta_dopri5<state>;


int main(int argc, char **argv)
{
  /* ---------- ROS intitialization ---------- */

  // Init ROS fast integrator node
  ros::init(argc, argv, "integrator");
  ros::NodeHandle n;

  // Subscribe to commands
  ros::Subscriber command_sub = n.subscribe("commands", 10, processCommand);
    
  // Subscribe to control message from control node
  ros::Subscriber rocket_control_sub = n.subscribe("control_pub", 100, rocket_controlCallback);

  // Subscribe to aero message
  ros::Subscriber rocket_aero_sub = n.subscribe("rocket_aero", 100, rocket_aeroCallback);

  // Subscribe to perturbations message
  ros::Subscriber rocket_perturbation_sub = n.subscribe("disturbance_pub", 100, rocket_perturbationCallback);

	// Create state publisher
	ros::Publisher rocket_state_pub = n.advertise<real_time_simulator::State>("rocket_state", 10);

  // Create fake sensors publisher
	ros::Publisher rocket_sensor_pub = n.advertise<real_time_simulator::Sensor>("sensor_pub", 10);

  // Create timer publisher and associated thread (100Hz)
	ros::Publisher fsm_pub = n.advertise<real_time_simulator::FSM>("fsm_pub", 10);

  /* ---------- Variable initialization  ---------- */

  // Initialize rocket class with useful parameters
  rocket.init(n);

  // Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";
	
	n.getParam("/environment/rail_length", rail_length);

  // Initialize external forces
  aero_control << 0, 0,
                  0, 0,
                  0, 0;

  rocket_control << 0, 0,
                    0, 0,
                    rocket.maxThrust[2], 0;

  using namespace Eigen;

  //Get initial orientation and convert in Radians
  float roll = 0, zenith = 0, azimuth = 0.0;
  n.getParam("/environment/rocket_roll", roll);
  n.getParam("/environment/rail_zenith", zenith);
  n.getParam("/environment/rail_azimuth", azimuth);

  roll *= 3.14159/180; zenith *= 3.14159/180; azimuth *= 3.14159/180;

  typedef EulerSystem<-EULER_Z, EULER_Y, EULER_Z> Rail_system;
  typedef EulerAngles<double, Rail_system> angle_type;

  angle_type init_angle(azimuth, zenith, roll);

  Quaterniond q(init_angle);

  // Init state X
  state X;
  X << 0, 0, 0,   0, 0, 0,     0.0, 0.0 , 0.0 , 1.0 ,     0.0, 0.0, 0.0,    rocket.propellant_mass;
  X.segment(6,4) = q.coeffs();

  state xout = X;

  // Init solver
  float period_integration = 10e-3;
  stepper_type2 stepper;

  // Thread to integrate state. Duration defines interval time in seconds
  ros::Timer integrator_thread = n.createTimer(ros::Duration(period_integration), [&](const ros::TimerEvent&)
	{
    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{

		}
    else
    {
      // Update current time
	  	current_fsm.time_now = ros::Time::now().toSec() - time_zero;

      if (current_fsm.state_machine.compare("Rail") == 0)
      {
        stepper.do_step(dynamics_rail, X, 0, xout, 0 + period_integration);
        
        // End of rail -> Launch state
        if(X(2)>rail_length)
        {
					current_fsm.state_machine = "Launch";
				}
      }

      else if (current_fsm.state_machine.compare("Launch") == 0)
      {
        stepper.do_step(dynamics_flight, X, 0, xout, 0 + period_integration);
        
        // End of burn -> no more thrust
				if(X(13) < 0)
				{
					current_fsm.state_machine = "Coast";
				}
      }

      else if (current_fsm.state_machine.compare("Coast") == 0)
      {
        rocket_control << 0, 0,
                          0, 0,
                          0, 0;
        stepper.do_step(dynamics_flight, X, 0, xout, 0 + period_integration);
      }


      X = xout;

      rocket.update_CM(X(13));
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
  });

  double sensor_period; n.getParam("/perturbation/sensor_period", sensor_period);
  ros::Timer sensor_thread = n.createTimer(ros::Duration(sensor_period), [&](const ros::TimerEvent&)
	{
    send_fake_sensor(rocket_sensor_pub);
  });

  // Automatic callback of service and publisher from here
	ros::spin();

}
