#include <ros/ros.h>

#include <controller_manager/controller_manager.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <serial/serial.h>

namespace turtle_hardware_interface
{

  class TurtleHardwareInterface : public hardware_interface::RobotHW
  {
  public:
    TurtleHardwareInterface(const char *tty_path);
    ~TurtleHardwareInterface();
    void read();
    void write();
    void update(const ros::TimerEvent& e);
  private:
    // ros machinery
    ros::NodeHandle nh_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    ros::Timer update_loop_;
    ros::Duration elapsed_time_;
  
    // serial port
    serial::Serial serial_port_;
  
    // joint state interfaces
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];
  };
  
}

