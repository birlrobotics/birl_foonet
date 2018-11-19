#ifndef FOO_TOPIC_PUBLISHER_H_
#define FOO_TOPIC_PUBLISHER_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <ar_track_alvar_msgs/AlvarMarkers.h> 
#include <ar_track_alvar_msgs/AlvarMarker.h> 
#include <baxter_core_msgs/EndpointState.h>
#include <birl_foonet/foonet.h>
// define a class, including a constructor, member variables and member functions
class FooNetClass
{
public:
    FooNetClass(ros::NodeHandle* nodehandle); 
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber arMarker_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber baxterEndpoint_subscriber_;
    ros::Publisher  objectTransform_pub_;
    
    AlvarMarkers::AlvarMarker msg_marker_shared;
    baxter_core_msgs::EndpointState msg_endpoint_shared;
    
    // member methods as well:
    void initializeArMarker_Subscribers(); 
    void initializeBaxterEndpoint_Subscribers(); 
    void initializeObjectTransform_Publishers();

    
    void arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers& arMarkerMsg); 
    void baxterEndpointCallback(const baxter_core_msgs::EndpointState& baxterEndpointMsg); 

}; // note: a class definition requires a semicolon at the end of the definition

#endif // this closes the header-include trick...ALWAYS need one of these to match #ifndef