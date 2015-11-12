
#include <cstdio>

#include <ros/ros.h>

#include "std_srvs/Empty.h"

int main( int argc, char **argv )
{
  ros::init(argc, argv, "start_camera");

  // Allow the action server to receive and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // Block until the camera capture service is ready
  ROS_INFO("Waiting for camera capture service");
  bool ready = ros::service::waitForService( "/camera/start_capture", -1 );
  if ( ready )
  {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    bool started = ros::service::call<std_srvs::Empty::Request,std_srvs::Empty::Response>( "/camera/start_capture", req, res );

    if ( !started )
    {
      ROS_ERROR("Could not start camera capture.");
    } else {
      ROS_INFO("Camera capture started.");
    }
  }

  return 0;
}

