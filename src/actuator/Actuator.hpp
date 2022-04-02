#include "ros/ros.h"

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

#include "real_time_simulator/Control.h"

class Actuator{

    public:

        

        // Subscriber to actuator command and publisher to actuator real-time state
        ros::Publisher actuatorPublisher;
        ros::Subscriber actuatorSubscriber;

        void computeStateActuator();
        real_time_simulator::Control getActuatorWrench();




};