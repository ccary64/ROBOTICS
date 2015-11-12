#include "turtle_hardware_interface.h"

#define SAMPLES_PER_SEC 5

namespace turtle_hardware_interface
{
    
    TurtleHardwareInterface::TurtleHardwareInterface( const char *tty_path )
    {
        // open serial port
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
        
        // initialize state variables
        pos_[0] = pos_[1] = 0;
        vel_[0] = vel_[1] = 0;
        eff_[0] = eff_[1] = 0;
        cmd_[0] = cmd_[1] = 0;
        
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a("left_motor", &pos_[0], &vel_[0], &eff_[0]);
        jnt_state_interface_.registerHandle(state_handle_a);
        
        hardware_interface::JointStateHandle state_handle_b("right_motor", &pos_[1], &vel_[1], &eff_[1]);
        jnt_state_interface_.registerHandle(state_handle_b);
        
        registerInterface(&jnt_state_interface_);
        
        // connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle_a(jnt_state_interface_.getHandle("left_motor"), &cmd_[0]);
        jnt_vel_interface_.registerHandle(vel_handle_a);
        
        hardware_interface::JointHandle vel_handle_b(jnt_state_interface_.getHandle("right_motor"), &cmd_[1]);
        jnt_vel_interface_.registerHandle(vel_handle_b);
        
        registerInterface(&jnt_vel_interface_);
        
        // create the controller manager
        ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        
        // start the update loop
        ros::Duration update_freq = ros::Duration(1.0/SAMPLES_PER_SEC);
        update_loop_ = nh_.createTimer(samples_per_sec_, &TurtleHardwareInterface::update, this);
    }
    
    TurtleHardwareInterface::~TurtleHardwareInterface()
    {
        // close serial port
        serial_port_.close();
    }
    
    void TurtleHardwareInterface::read()
    {
        // if serial port is not open, do nothing
        
        // read 4 bytes: left motor direction, left encoder count, right motor direction, right encoder count
        
        // update pos_[] field (in radians) according to encoder counts
    }
    
    void TurtleHardwareInterface::write()
    {
        // if serial port is not open, directly update the pos_[] field
        
        // convert angular velocities in cmd_[] (in radians per second) to clicks per sample
        
        // write 4 bytes: left motor direction, left encoder count, right motor direction, right encoder count
    }
    
    void TurtleHardwareInterface::update(const ros::TimerEvent& e)
    {
        // calculate time elapsed since last update
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        
        // read from motor controller
        read();
        
        // update controller manager
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        
        // write out to motor controller
        write();
    }
    
}

int main( int argc, char **argv )
{
    // initialize node
    ros::init(argc, argv, "turtle_hardware_interface");
    
    // allow the action server to receive and send ros messages
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // create the ROS node handle
    ros::NodeHandle nh;
    
    // get tty path from arguments
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
    
    // instantiate hardware interface
    turtle_hardware_interface::TurtleHardwareInterface robot(serial_name.c_str());
    
    // spin until ROS tells us to stop
    ros::spin();
    
    return 0;
}

