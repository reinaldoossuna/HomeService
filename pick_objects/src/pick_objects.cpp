#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct pose{
  float x;
  float y;
  float z;
  float orient_x;
  float orient_y;
  float orient_z;
  float orient_w;
};

move_base_msgs::MoveBaseGoal create_goal(struct goal_pose);

pose PICK_UP_POSE = { .00, 1.30, 0.0, 0.0, 0.0, 0.0, 1.50};
pose DROP_OFF_POSE = { 7.70, 0.37, 0.0, 0.0, 0.0, 0.0, 1.50};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_objects");

    ROS_INFO("NAVIGATION GOALs");
    // Spin a thread
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server");
    }
    ROS_INFO("Connected to move_base server");

    move_base_msgs::MoveBaseGoal pickup = create_goal(PICK_UP_POSE);
    ROS_INFO("Sending goal");
    ac.sendGoal(pickup);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Excellent! Your robot has reached the pick up pose.");
    else
        ROS_INFO("The robot failed to reach the goal position");

    ros::Duration(5.0).sleep();

    move_base_msgs::MoveBaseGoal dropoff = create_goal(DROP_OFF_POSE);
    ROS_INFO("Robot is traveling to dropoff");

    ac.sendGoal(dropoff);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Excellent! Your robot has reached the dropoff.");
    else
        ROS_INFO("The robot failed to reach the goal position");
    
    return 0;
}

move_base_msgs::MoveBaseGoal create_goal(struct pose g)
{

    move_base_msgs::MoveBaseGoal goal;

    // Send goal pose
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = g.x;
    goal.target_pose.pose.position.y = g.y;

    goal.target_pose.pose.orientation.x = g.orient_x;
    goal.target_pose.pose.orientation.y = g.orient_y;
    goal.target_pose.pose.orientation.z = g.orient_z;
    goal.target_pose.pose.orientation.w = g.orient_w;

    return goal;

}
