#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


geometry_msgs::PoseStamped pos;

void host(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pos = *msg;
}

int main(int argv,char** argc)
{
    ros::init(argv,argc,"optitrack");
    ros::NodeHandle nh;

    //subscribe :/vrpn_client_mode .... and publish to mavros/vison... then uav can receive optitrack
    ros::Publisher position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose", 10, host);
    ros::Rate rate(100);
    while(ros::ok())
    {
        //ROS_INFO("MAV1: %.3f, %.3f , %.3f",pos.pose.position.x,pos.pose.position.y,pos.pose.position.z);
        position_pub.publish(pos);
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}