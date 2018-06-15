
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test");

    ros::NodeHandle nh("k");
    ros::NodeHandle private_nh("~");
    //ros::NodeHandle nh("test");
   
    private_nh.setParam("num",1);
    nh.setParam("num",2);

    while (ros::ok()) {
    ros::spinOnce();
    }

    return 0;

}
