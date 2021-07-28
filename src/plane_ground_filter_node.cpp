//
// Created by zlc on 2021/3/8.
//

#include "../include/plane_ground_filter_core.h"

int main(int argc, char* *argv)
{
    ros::init(argc, argv, "plane_ground_filter");

    ros::NodeHandle nh("~");
    // ros::NodeHandle pn1("~");       //pn1 命名空间为/node_namespace/node_name
    // ros::NodeHandle pn2("~sub");    //pn2 命名空间为/node_namespace/node_name/sub

    PlaneGroundFilter core(nh);


    return 0;
}

