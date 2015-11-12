
#include "cylinder_detection.h"

#include <cstdio>

#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cylinder_detection
{

  CylinderDetection::CylinderDetection() : action_client_("move_base",false)
  {
    ros::param::param<double>("object_width",object_width_,0.022);

    cv::namedWindow("image");
    cv::namedWindow("mask");
    cv::namedWindow("contours");
    camera_info_received_ = false;
    camera_info_subscriber_ = node_handle_.subscribe( "/camera/camera_info", 1000, &CylinderDetection::cameraInfoReceived, this );
    image_subscriber_ = node_handle_.subscribe( "/camera/image_raw", 1000, &CylinderDetection::imageReceived, this );
  }

  CylinderDetection::~CylinderDetection()
  {
  }

  void CylinderDetection::cameraInfoReceived( const sensor_msgs::CameraInfo &camera_info_msg )
  {
    camera_info_received_ = true;
    fx_ = camera_info_msg.K[0];
    cx_ = camera_info_msg.K[2];
    ROS_INFO_STREAM("Camera focal length: " << fx_ );
    camera_info_subscriber_.shutdown();
  }
  
  void CylinderDetection::imageReceived( const sensor_msgs::Image &image_msg )
  {
    // makes GUI work
    cv::waitKey(30);

    // copy image to opencv format
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy( image_msg );
    
    // flip image
    cv::Mat image = cv_image_ptr->image;
    cv::Mat flipped( image.size(), CV_8UC3 );
    cv::flip( image, flipped, -1 );

    cv::imshow("image",flipped);

    // find red pixels
    cv::Mat redmask( image.size(), CV_8UC1 );
    cv::inRange(flipped,cv::Scalar(0,0,128),cv::Scalar(64,64,255),redmask);

    cv::imshow("mask",redmask);

    // find blobs
    cv::Mat contourim( image.size(), CV_8UC1 );
    contourim = cv::Scalar(0);
    
    std::vector< std::vector< cv::Point2i > > contours;
    cv::findContours(redmask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    
    cv::drawContours(contourim,contours,-1,cv::Scalar(128),-1);

    // find the biggest blob
    double largestArea = 0;
    int largestContourIndex = -1;
    for ( int i = 0; i < contours.size(); i++ )
    {
      double area = cv::contourArea(contours[i]);
      if ( area > largestArea )
      {
        largestArea = area;
        largestContourIndex = i;
      }
    }

    cv::drawContours(contourim,contours,largestContourIndex,cv::Scalar(255),-1);
    cv::imshow("contours",contourim);

    if ( largestArea < 25. ) return;
    
    // find the width of the blob and the center x coordinate
    int minx = INFINITY;
    int maxx = 0;
    for ( int i = 0; i < contours[largestContourIndex].size(); i++ )
    {
      int x = contours[largestContourIndex][i].x;
      if ( x < minx ) minx = x;
      if ( x > maxx ) maxx = x;
    }
    int contourwidth = maxx - minx;
    float meanx = (maxx + minx)/2.f;
    ROS_INFO_STREAM( "contour width: " << contourwidth );
    ROS_INFO_STREAM( "mean contour x: " << meanx );

    if ( !camera_info_received_ ) return;

    // calculate the heading of the blob
    float angle = atan2((meanx - cx_),fx_);
    ROS_INFO_STREAM("angle of cylinder: " << angle*180/M_PI );

    // calculate the distance to the blob
    // xmax = fx * Xmax / Z
    // xmin = fx * Xmin / Z
    // xmax - xmin = ( fx / Z ) * ( Xmax - Xmin )
    // contourwidth = ( fx / Z ) * objectwidth
    // Z = fx * objectwidth / contourwidth
    float distance = fx_ * object_width_ / contourwidth;
    ROS_INFO_STREAM("distance to cylinder: " << distance );

    // create rotation action
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0;
    tf::Quaternion goal_orientation;
    goal_orientation.setEuler(0,0,angle);
    goal.target_pose.pose.orientation = goal_orientation;

    action_client_.sendGoal(goal);

    action_client_.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Action succeeded.");
    }
    else
    {
      ROS_ERROR("Action failed.");
    }
  }

}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "cylinder_detection");

  // Allow the action server to receive and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  cylinder_detection::CylinderDetection cd;

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

  ros::spin();

  ROS_INFO("Shutting down.");

  return 0;
}

