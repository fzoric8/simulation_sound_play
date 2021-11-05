
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


int main (int argc, char** argv){
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");


    int counter=0;
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/path", 100);

    std::vector<geometry_msgs::PoseStamped> poses;
    int time = 0;
    for (int i=0; i < 10; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp=ros::Time(0)+ros::Duration(time,0);  
        pose.pose.position.x = 0;
        pose.pose.position.y = 0 + (double) i * 4/10;
        pose.pose.position.z = 1;
        poses.push_back(pose);
        time=time+1;
    }
    for (int i=0;i<10;i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp=ros::Time(0)+ros::Duration(time,0);
        pose.pose.position.x = 0 + (double) i * 5/10; 
        pose.pose.position.y = 4;
        pose.pose.position.z = 1;
        poses.push_back(pose);
        time = time + 1;
    }
    for (int i = 0; i < 10; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(0) + ros::Duration(time, 0);
        pose.pose.position.x = 5;
        pose.pose.position.y = 4 - (double) i * 6/10 ;
        pose.pose.position.z = 1; 
        poses.push_back(pose);
        time=time+1;
    }

    for (int i = 0; i < 10; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(0) + ros::Duration(time, 0);
        pose.pose.position.x = 5 + (double) i * 3/10;
        pose.pose.position.y = -2 ;
        pose.pose.position.z = 1; 
        poses.push_back(pose);
        time=time+1;
    }
    ros::Rate loop_rate(1);
    bool first=true;
    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
        
	if (first)

    {

		std::cout<<"publish"<<std::endl;
		first = false;
        tf::StampedTransform transform;
        ros::Time t = ros::Time(0);

        nav_msgs::Path data;
	    data.poses = poses;
	    data.header.stamp = ros::Time::now();
	    path_publisher.publish(data);
	}
        loop_rate.sleep();
        ros::spinOnce();
	
    }
}

