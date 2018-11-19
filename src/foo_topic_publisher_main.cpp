#include <birl_foonet/foo_topic_publisher.h>

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "example_lib_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    FooNetClass fooNetClass(&nh);  

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 