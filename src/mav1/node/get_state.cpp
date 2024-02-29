#include "ros/ros.h"
#include <mavros_msgs/State.h>


mavros_msgs::State state;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{

    state = *msg;

}


int main(int argv,char** argc)
{
    ros::init(argv,argc,"get_state");
    ros::NodeHandle nh;


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);

    
    ros::Rate rate(10.0);

    while(ros::ok())
    {
        ROS_INFO("current_state_is:%d",state.mode);


        ros::spinOnce();
        rate.sleep();

    }



    return 0;
}