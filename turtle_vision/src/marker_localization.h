#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace marker_localization
{

  class MarkerLocalization
  {
  public:
    MarkerLocalization();
    ~MarkerLocalization();
    void update(const ros::TimerEvent& e);
  private:
    // ros machinery
    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    
    // Map to marker transforms
    tf::Transform T_map_marker_0_;

    // Map to odom transform
    tf::Transform T_map_odom_;

    // tf listener
    tf::TransformListener tf_listener_;

    // tf broadcaster
    tf::TransformBroadcaster tf_broadcaster_;
  };
  
}

