#!/usr/bin/env python
import rospy
#include "ros/ros.h"
from std_msgs.msg import *
from time import sleep
from sensor_msgs.msg import JointState, Joy
from copy import deepcopy
from visualization_msgs.msg import Marker, MarkerArray

import actionlib
from hebiros.srv import AddGroupFromNamesSrv
from hebiros.msg import WaypointMsg, TrajectoryAction, TrajectoryGoal
from numpy import pi, nan
from actionlib import simple_action_client
import actionlib
global feedback
feedback = None
global group_name
group_name = "MyArm"


def callback_turn_on():
    print("started action")


def callback_active(completion):
    print(completion)


def callbackFeedback(jointStateMsg):
    global feedback
    feedback = jointStateMsg


def trajectoryDone(state, result):
    print("Final state: %s", str(state))



def main():
    print("use gazebo tag")
    rospy.init_node('trajectoryPub', anonymous=True)
    print("wait for name")
    rospy.wait_for_service("/hebiros/add_group_from_names")
    print("name up and running")
    add_group = rospy.ServiceProxy("/hebiros/add_group_from_names", AddGroupFromNamesSrv)
    addGZgroup = rospy.ServiceProxy("/hebiros_gazebo_plugin/add_group", AddGroupFromNamesSrv)

    names = ("base", "shoulder", "elbow", "wrist")
    families = ("HEBI",)

    resp1 = None
    while not resp1:
        resp1 = add_group(group_name=group_name, names=names, families=families)
    feedback_subscriber = rospy.Subscriber("/hebiros/" + group_name + "/feedback/joint_state", JointState,
                                           callback=callbackFeedback)
    actionTag = "/hebiros/"+group_name+"/trajectory"
    actiontype = TrajectoryAction
    client = actionlib.SimpleActionClient(actionTag, actiontype)
    print("waiting on:", actionTag, "of type:", actiontype)
    client.wait_for_server(timeout=rospy.Duration(0.5))
    print("client ready")
    goal = TrajectoryGoal()
    waypoint = WaypointMsg()
    times = (0, 3, 6, 9, 12)
    positions = ((feedback.position[0], 0, -pi/2, 0, 0),
                 (feedback.position[1], 0, 0, pi, pi/2),
                 (feedback.position[2], 0, pi/2, -pi/2, 0),
                 (feedback.position[3], 0, 0, 0, 0))

    velocities = tuple([tuple([0]+[1] * (len(times)-2)+[0])]*len(names))
    """((0, 0, 0, 0, 0),
                  (0, 0, 0, 0, 0),
                  (0, 0, 0, 0, 0),
                  (0, 0, 0, 0, 0))"""
    accelerations = tuple([tuple([0]+[nan] * (len(times)-2)+[0])]*len(names))
    """((0, nan, nan, nan, 0),
                     (0, nan, nan, nan, 0),
                     (0, nan, nan, nan, 0),
                     (0, nan, nan, nan, 0))"""
    waypoint.names = ["HEBI/"+name for name in names]
    for i in range(len(times)):

        waypoint.positions = tuple([positions[j][i]for j in range(len(names))])
        waypoint.velocities = tuple([velocities[j][i]for j in range(len(names))])
        waypoint.accelerations = tuple([accelerations[j][i]for j in range(len(names))])
        goal.times.append(times[i])
        goal.waypoints.append(deepcopy(waypoint))
    print("start sendgoal")
    client.send_goal(goal,
                active_cb=callback_turn_on,
                feedback_cb=callback_active,
                done_cb=None)
    client.wait_for_result()
    print(client.get_result())


if __name__ == "__main__":
    main()
"""
#include "hebiros/TrajectoryAction.h"

using namespace hebiros;


//Global variable and callback function used to store feedback data
sensor_msgs::JointState feedback;

void feedback_callback(sensor_msgs::JointState data) {
  feedback = data;
}

//Callback which is called once when the action goal completes
void trajectory_done(const actionlib::SimpleClientGoalState& state,
  const TrajectoryResultConstPtr& result) {
  std::cout << "Final state: " << state.toString() << std::endl;

  for (int i = 0; i < result->final_state.name.size(); i++) {
    std::cout << result->final_state.name[i] << ": " << std::endl;
    std::cout << "  Position: " << result->final_state.position[i] << std::endl;
    std::cout << "  Velocity: " << result->final_state.velocity[i] << std::endl;
    std::cout << "  Effort: " << result->final_state.effort[i] << std::endl;
  }
  
  ros::shutdown();
}

//Callback which is called once when the action goal becomes active
void trajectory_active()
{
  std:: cout << "Goal just went active" << std::endl;
}

//Callback which is called every time feedback is received for the action goal
void trajectory_feedback(const TrajectoryFeedbackConstPtr& feedback)
{
  std::cout << "Trajectory percent completion: " << feedback->percent_complete << std::endl;
}


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_04_trajectory_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "my_group";
  int num_joints = 3;
  int num_waypoints = 5;

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  //Create a subscriber to receive feedback from a group
  //Register feedback_callback as a callback which runs when feedback is received
  ros::Subscriber feedback_subscriber = n.subscribe(
    "/hebiros/"+group_name+"/feedback/joint_state", 100, feedback_callback);

  AddGroupFromNamesSrv add_group_srv;

  //Construct a group using 3 known modules
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"base", "shoulder", "elbow"};
  add_group_srv.request.families = {"HEBI"};
  //Call the add_group_from_urdf service to create a group until it succeeds
  //Specific topics and services will now be available under this group's namespace
  while(!add_group_client.call(add_group_srv)) {}

  feedback.position.reserve(3);

  //Create an action client for executing a trajectory
  actionlib::SimpleActionClient<TrajectoryAction> client("/hebiros/"+group_name+"/trajectory", true);
  //Wait for the action server corresponding to the action client
  client.waitForServer();

  //Construct a trajectory to be sent as an action goal
  TrajectoryGoal goal;
  goal.times.resize(num_waypoints);
  goal.waypoints.resize(num_waypoints);

  WaypointMsg waypoint;
  waypoint.names.resize(num_joints);
  waypoint.positions.resize(num_joints);
  waypoint.velocities.resize(num_joints);
  waypoint.accelerations.resize(num_joints);

  double nan = std::numeric_limits<float>::quiet_NaN();
  //Set the times to reach each waypoint in seconds
  std::vector<double> times = {0, 5, 10, 15, 20};
  std::vector<std::string> names = {"HEBI/base", "HEBI/shoulder", "HEBI/elbow"};
  //Set positions, velocities, and accelerations for each waypoint and each joint
  //The following vectors have one joint per row and one waypoint per column
  std::vector<std::vector<double>> positions = {{feedback.position[0], 0, M_PI_2, 0,      0},
                                                {feedback.position[1], 0, M_PI_2, M_PI_2, 0},
                                                {feedback.position[2], 0, 0,      M_PI_2, 0}};
  std::vector<std::vector<double>> velocities = {{0, nan, nan, nan, 0},
                                                 {0, nan, nan, nan, 0},
                                                 {0, nan, nan, nan, 0}};
  std::vector<std::vector<double>> accelerations = {{0, nan, nan, nan, 0},
                                                    {0, nan, nan, nan, 0},
                                                    {0, nan, nan, nan, 0}};

  //Construct the goal using the TrajectoryGoal format
  for (int i = 0; i < num_waypoints; i++) {
    for (int j = 0; j < num_joints; j++) {
      waypoint.names[j] = names[j];
      waypoint.positions[j] = positions[j][i];
      waypoint.velocities[j] = velocities[j][i];
      waypoint.accelerations[j] = accelerations[j][i];
    }
    goal.times[i] = times[i];
    goal.waypoints[i] = waypoint;
  }

  //Send the goal, executing the trajectory
  client.sendGoal(goal, &trajectory_done, &trajectory_active, &trajectory_feedback);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
"""