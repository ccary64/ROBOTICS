#include "turtle_hardware_interface.h"
#include "turtle_hardware_interface/Encoders.h"

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>

namespace turtle_hardware_interface
{

  TurtleHardwareInterface::TurtleHardwareInterface( const char *tty_path )
  {
      /// open serial port
      serial_port_.setPort( tty_path );
      serial_port_.setBaudrate( 9600 );
      serial::Timeout timeout( serial::Timeout::simpleTimeout(1000) );
      serial_port_.setTimeout( timeout );
      try
      {
          serial_port_.open();
      }
      catch ( std::exception &e )
      {
          ROS_ERROR_STREAM( "could not open " << tty_path << " for read/write" );
      }
      
      // read parameters
      ros::param::param<double>("~clicksPerRotation",clicks_per_rotation_,20);
      ros::param::param<double>("~samplesPerSec",samples_per_sec_,5);

      rads_per_click_ = (2.*M_PI)/clicks_per_rotation_;

      ROS_INFO_STREAM_NAMED("hardware_interface","Clicks per rotation: " << clicks_per_rotation_);
      ROS_INFO_STREAM_NAMED("hardware_interface","Samples per sec: " << samples_per_sec_);

      // initialize state variables
      pos_[0] = pos_[1] = 0;
      vel_[0] = vel_[1] = 0;
      eff_[0] = eff_[1] = 0;
      cmd_[0] = cmd_[1] = 0;

      /// connect and register the joint state interface
      hardware_interface::JointStateHandle state_handle_a("left_motor", &pos_[0], &vel_[0], &eff_[0]);
      jnt_state_interface_.registerHandle(state_handle_a);

      hardware_interface::JointStateHandle state_handle_b("right_motor", &pos_[1], &vel_[1], &eff_[1]);
      jnt_state_interface_.registerHandle(state_handle_b);

      registerInterface(&jnt_state_interface_);

      /// connect and register the joint velocity interface
      hardware_interface::JointHandle vel_handle_a(jnt_state_interface_.getHandle("left_motor"), &cmd_[0]);
      jnt_vel_interface_.registerHandle(vel_handle_a);

      hardware_interface::JointHandle vel_handle_b(jnt_state_interface_.getHandle("right_motor"), &cmd_[1]);
      jnt_vel_interface_.registerHandle(vel_handle_b);

      registerInterface(&jnt_vel_interface_);

      // Create the controller manager
      ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
      controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

      // Create the custom publishers
      encoders_publisher_ = nh_.advertise<turtle_hardware_interface::Encoders>("encoders",1000);
      encoders_cmd_publisher_ = nh_.advertise<turtle_hardware_interface::Encoders>("encoders_cmd",1000);

      // Start the update loop
      ros::Duration update_freq = ros::Duration(1.0/samples_per_sec_);
      non_realtime_loop_ = nh_.createTimer(samples_per_sec_, &TurtleHardwareInterface::update, this);

      ROS_INFO_NAMED("hardware_interface", "Loaded turtle_hardware_interface.");
  }

  TurtleHardwareInterface::~TurtleHardwareInterface()
  {
      /// close serial port
      serial_port_.close();
  }

  void TurtleHardwareInterface::read()
  {
      // if hardware is not attached, do nothing
      if ( !serial_port_.isOpen() )
      {
        return;
      }

      // read 4 bytes
      unsigned char data[4];
      int totalread = 0;
      while ( totalread < 4 )
      {
          try { 
            const int nread = serial_port_.read( data+totalread, 4-totalread );
            totalread += nread;
          } catch ( std::exception &e ) {
            ROS_ERROR_STREAM( "read() failed: " << e.what() );
            return;
          }
      }

      int encoders[2];
      encoders[0] = data[1];
      if ( data[0] ) encoders[0] = -encoders[0];
      encoders[1] = data[3];
      if ( data[2] ) encoders[1] = -encoders[1];

      Encoders encoders_msg;
      encoders_msg.header.stamp = ros::Time::now();
      encoders_msg.left = encoders[0];
      encoders_msg.right = encoders[1];
      encoders_publisher_.publish(encoders_msg);
      //ROS_INFO_STREAM( "read " << nread << " bytes from serial port" );
      //ROS_INFO_STREAM( "encoders: " << (int)encoders[0] << ", " << (int)encoders[1] );
      
      /// update pos (rad) according to encoder counts
      const double left_increment = encoders[0]*rads_per_click_;
      const double right_increment = encoders[1]*rads_per_click_;
      
      pos_[0] += left_increment;
      pos_[1] += right_increment;
  }

  void TurtleHardwareInterface::write()
  {
      // if hardware is not attached, simulate the command
      if ( !serial_port_.isOpen() )
      {
          const double left_increment = cmd_[0]/samples_per_sec_;
          const double right_increment = cmd_[1]/samples_per_sec_;
          pos_[0] += left_increment;
          pos_[1] += right_increment;
          return;
      }

      // cmd contains angular wheel velocities

      /// convert angular velocities to clicks per sample
      const double left_setpoint = cmd_[0]/rads_per_click_/samples_per_sec_;
      const double right_setpoint = cmd_[1]/rads_per_click_/samples_per_sec_;

      //ROS_INFO_STREAM( "incoming command: " << cmd_[0] << ", " << cmd_[1] );

      // convert to integers
      unsigned char data[4];
      data[0] = (left_setpoint<0)?1:0;
      data[1] = fabs(left_setpoint);
      data[2] = (right_setpoint<0)?1:0;
      data[3] = fabs(right_setpoint);

      Encoders encoders_cmd_msg;
      encoders_cmd_msg.header.stamp = ros::Time::now();
      encoders_cmd_msg.left = left_setpoint;
      encoders_cmd_msg.right = right_setpoint;
      encoders_cmd_publisher_.publish(encoders_cmd_msg);

      //ROS_INFO_STREAM( "outgoing command: " << (int)left_setpoint << ", " << (int)right_setpoint );

      // write to serial port
      serial_port_.flush();
      int totalwritten = 0;
      while ( totalwritten < 4 )
      {
          int nwritten = serial_port_.write( data+totalwritten, 4-totalwritten );
          totalwritten += nwritten;
      }
  }

  void TurtleHardwareInterface::update(const ros::TimerEvent& e)
  {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);

    // Input
    read();

    // Control
    controller_manager_->update(ros::Time::now(), elapsed_time_);

    // Output
    write();
  }

}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "turtle_hardware_interface");

  // Allow the action server to receive and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  std::string serial_name( "/dev/ttyUSB0" );

  for ( int i = 0; i < argc; i++ )
  {
    if ( std::string(argv[i]).compare("--serial") == 0 )
    {
      if ( argc > i+1 )
      {
        serial_name = argv[i+1];
      }
    }
  }


  turtle_hardware_interface::TurtleHardwareInterface robot(serial_name.c_str());

  ros::spin();

  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

  return 0;
}

