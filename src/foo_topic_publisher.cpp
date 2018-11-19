#include <birl_foonet/foo_topic_publisher.h>

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
FooNetClass::FooNetClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of FooNetClass");
    initializeArMarker_Subscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializeBaxterEndpoint_Subscribers();
    initializeObjectTransform_Publishers();
    
    //initialize variables here, as needed
    // msg_marker_shared = None;
    // msg_endpoint_shared = None;
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &FooNetClass::subscriberCallback is a pointer to a member function of FooNetClass
// "this" keyword is required, to refer to the current instance of FooNetClass
void FooNetClass::initializeArMarker_Subscribers()
{
    ROS_INFO("Initializing ArMarker Subscribers");
    arMarker_subscriber_ = nh_.subscribe("ar_pose_marker", 10, &FooNetClass::arMarkerCallback,this);  
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void FooNetClass::initializeBaxterEndpoint_Subscribers()
{
    ROS_INFO("Initializing BaxterEndpoint Services");
    baxterEndpoint_subscriber_ = nh_.advertiseService("/robot/limb/right/endpoint_state",
                                                   &FooNetClass::baxterEndpointCallback,
                                                   this);  

}

//member helper function to set up publishers;
void FooNetClass::initializeObjectTransform_Publishers()
{
    ROS_INFO("Initializing ObjectTransform Publishers");
    objectTransform_pub_ = nh_.advertise<birl_foonet::foonet>("foonet/object_transfrom", 10, true); 
   
}


void FooNetClass::arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers& arMarkerMsg) {

    msg_marker_shared = arMarkerMsg; // copy the received data into member variable, so ALL member funcs of FooNetClass can access it
    // ROS_INFO("myCallback activated: received value %f",msg_marker_shared);
    // birl_foonet::foonet output_msg;
    // objectTransform_pub_.publish(output_msg); //output the square of the received value; 
}

void FooNetClass::baxterEndpointCallback(const baxter_core_msgs::EndpointState& baxterEndpointMsg)) {

    msg_endpoint_shared = baxterEndpointMsg; // copy the received data into member variable, so ALL member funcs of FooNetClass can access it

}
