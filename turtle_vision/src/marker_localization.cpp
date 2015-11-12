
#include <cstdio>

#include "std_srvs/Empty.h"

#include "marker_localization.h"

namespace marker_localization
{

  MarkerLocalization::MarkerLocalization()
  {
      // Load map to marker transform
      double yaw, pitch, roll;
      ros::param::param<double>("map_marker_0_yaw",yaw,-M_PI/2);
      ros::param::param<double>("map_marker_0_pitch",pitch,0);
      ros::param::param<double>("map_marker_0_roll",roll,0);

      double x,y,z;
      ros::param::param<double>("map_marker_0_x",x,0);
      ros::param::param<double>("map_marker_0_y",y,0);
      ros::param::param<double>("map_marker_0_z",z,0.0335);

      ROS_INFO_STREAM("yaw: " << yaw << ", pitch: " << pitch << ", roll: " << roll );

      tf::Quaternion R_map_marker_0;
      R_map_marker_0.setEuler(yaw,pitch,roll);
      T_map_marker_0_.setRotation( R_map_marker_0 );

      T_map_marker_0_.setOrigin( tf::Vector3(x,y,z) );

      // Initialize map to odom transform
      T_map_odom_.setRotation( tf::Quaternion(0,0,0,1) );
      T_map_odom_.setOrigin( tf::Vector3(0,0,0) );

      // Start the update loop
      ros::Duration update_freq = ros::Duration(1.0/5.0);
      non_realtime_loop_ = nh_.createTimer(5.0, &MarkerLocalization::update, this);

      ROS_INFO_NAMED("marker_localization", "Loaded marker_localization.");
      
  }

  MarkerLocalization::~MarkerLocalization()
  {
  }

  void MarkerLocalization::update(const ros::TimerEvent& e)
  {
    // Look for marker 0 tf
    tf::StampedTransform T_odom_marker_0;
    try {
      tf_listener_.lookupTransform( "/odom", "/ar_marker_0", ros::Time(0), T_odom_marker_0 );
 
      // Calculate transform from map to odom
      T_map_odom_ = T_map_marker_0_ * T_odom_marker_0.inverse();

      // Correct transform so that robot still sits on the ground
      // (eliminate any rotation about X and Y axes)
    } catch ( tf::TransformException &ex ) {
      //ROS_INFO_NAMED("marker_localization", "Could not find transform from odom to ar_marker_0");
    }

    // Always send the current map to odom tf
    tf_broadcaster_.sendTransform(tf::StampedTransform(T_map_odom_,ros::Time::now(),"map","odom"));

    // DEBUG: send map to marker tf
    //tf_broadcaster_.sendTransform(tf::StampedTransform(T_map_marker_0_,ros::Time::now(),"map","ar_marker_0"));
  }

}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "marker_localization");

  // Allow the action server to receive and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  marker_localization::MarkerLocalization ml;

  ros::spin();

  ROS_INFO_NAMED("marker_localization","Shutting down.");

  return 0;
}

