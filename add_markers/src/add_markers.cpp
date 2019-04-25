#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

float PICK_UP_X = 7.70;
float PICK_UP_Y = 0.37;

float DROP_OFF_X = 0.00;
float DROP_OFF_Y = 1.30;

// state 0 = going to pickup
// state 1 = going to dropoff
// state 2 = droped
int state = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

  float pos_x = msg->pose.pose.position.x;
  float pos_y = msg->pose.pose.position.y;

  float dist = 0.0;

  switch(state){
    case 0:
             dist = sqrt(pow((PICK_UP_X - pos_x),2) + pow((PICK_UP_Y - pos_y),2));
             ROS_INFO("%f",dist);
             if(dist <= 0.5){
               ROS_INFO("In the pick up");
               state = 1;
             }
             break;
   
    case 1:
             dist = sqrt(pow((DROP_OFF_X - pos_x),2) + pow((DROP_OFF_Y - pos_y),2));
             ROS_INFO("%f",dist);
             if(dist <= 0.5){
               ROS_INFO("In the dropoff");
               state = 2;
             }
             break;

  }

}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber odom_sub = n.subscribe("/odom",1000, odomCallback);

  // Set our initial shape type to be a cube
  uint32_t cube = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = cube;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = PICK_UP_X;
  marker.pose.position.y = PICK_UP_Y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker 
  marker.scale.x = 0.10;
  marker.scale.y = 0.10;
  marker.scale.z = 0.10;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (ros::ok())
  {

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    switch(state){
      case 0:
              ROS_INFO("going to pick up");
              break;
      case 1: marker.action = visualization_msgs::Marker::DELETE;
              ros::Duration(5.0).sleep();
              ROS_INFO("going to dropoff");
              break;
      case 2: 
              marker.pose.position.x = DROP_OFF_X;
              marker.pose.position.y = DROP_OFF_Y;
              marker.action = visualization_msgs::Marker::ADD;
              ROS_INFO("droped");
              ros::Duration(5.0).sleep();
              break;
    }
    marker_pub.publish(marker);
    ros::spinOnce();
  }
}
