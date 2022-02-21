#include <math.h>
#include "ros/ros.h"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "CommandMotorSpeed.pb.h"
#include "quadcopter/SendActuator.h"
using namespace std;

class ActuatorCallback{
    public:
    ActuatorCallback() {
        gazebo::transport::init();
        gazebo::transport::run();
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init("default");
        motorPub = node->Advertise<mav_msgs::msgs::CommandMotorSpeed>(
                                                            "/gazebo/default/iris/gazebo/command/motor_speed");

    }
    ~ActuatorCallback() {
        gazebo::transport::fini();
    }
    void add_motors(){
        speed_req.add_motor_speed(0.0);
        speed_req.add_motor_speed(0.0);
        speed_req.add_motor_speed(0.0);
        speed_req.add_motor_speed(0.0);
    }
    void actuator_callback(const quadcopter::SendActuator::ConstPtr& msg){
        //actuatordata_ = msg;
        speed_req.set_motor_speed(0,msg->motor1);
        speed_req.set_motor_speed(1,msg->motor2);
        speed_req.set_motor_speed(2,msg->motor3);
        speed_req.set_motor_speed(3,msg->motor4);
        motorPub->WaitForConnection();
        motorPub->Publish(speed_req);
        
    }
    private:
    mav_msgs::msgs::CommandMotorSpeed speed_req;
    gazebo::transport::PublisherPtr motorPub;
    //quadcopter::SendActuator& actuatordata_;
};

int main(int argc, char * argv[])
{
    /*gazebo::transport::init();
    gazebo::transport::run();
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init("default");
    gazebo::transport::PublisherPtr motorPub = node->Advertise<mav_msgs::msgs::CommandMotorSpeed>(
                                                            "/gazebo/default/iris/gazebo/command/motor_speed");*/
    //motorPub->WaitForConnection();

    ActuatorCallback actuatorcallback;
    actuatorcallback.add_motors();
    ros::init(argc,argv,"motor_publisher");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/desired/motor_speeds",10, &ActuatorCallback::actuator_callback,&actuatorcallback);

    ros::spin();
    
     
    return 0;
}