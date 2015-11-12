#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace cylinder_detection
{

  class CylinderDetection
  {
  public:
    CylinderDetection();
    ~CylinderDetection();
    
    void cameraInfoReceived( const sensor_msgs::CameraInfo &camera_info_msg );
    void imageReceived( const sensor_msgs::Image &image_msg );
  private:
    // ros machinery
    ros::NodeHandle node_handle_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
    
    // object parameters
    double object_width_;

    // camera calibration parameters
    bool camera_info_received_;
    double fx_;
    double cx_;

    // image subscriber
    ros::Subscriber camera_info_subscriber_;
    ros::Subscriber image_subscriber_;
  };
  
}

