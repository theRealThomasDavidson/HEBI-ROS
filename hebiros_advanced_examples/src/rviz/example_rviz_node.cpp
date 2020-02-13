#include "ros/ros.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/AddGroupFromNamesSrv.h"

using namespace hebiros;


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_rviz_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "x_demo";

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");


  AddGroupFromNamesSrv add_group_srv;

  //Construct a group using 3 known modules
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"base", "shoulder", "elbow"};
  add_group_srv.request.families = {"HEBI"};
  //Call the add_group_from_urdf service to create a group until it succeeds
  //Specific topics and services will now be available under this group's namespace
  while(!add_group_client.call(add_group_srv)) {}


  return 0;
}









