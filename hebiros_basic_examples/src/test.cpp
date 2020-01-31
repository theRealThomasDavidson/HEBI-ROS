#include "ros/ros.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"

using namespace hebiros;


int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);

    std::string group_name = "arm";
    ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

    add_group_srv.request.group_name = group_name;
    add_group_srv.request.names = {"base", "shoulder", "elbow"};
    add_group_srv.request.families = {"HEBI"};


    while(!add_group_client.call(add_group_srv)) {}

    return 0;
}