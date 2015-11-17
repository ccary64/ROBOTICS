#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

ros::Subscriber camera_info_subscriber_;

cv_bridge::CvImageConstPtr cv_ptr;
image_transport::Publisher pub;

double fx_;
double cx_;
double object_width_;
bool camera_info_received_;


int LowerH = 0;
int LowerS = 10;
int LowerV = 50;
int UpperH = 10;
int UpperS = 255;
int UpperV = 255;

void cameraInfoReceived( const sensor_msgs::CameraInfo &camera_info_msg )
{
    camera_info_received_ = true;
    fx_ = camera_info_msg.K[0];
    cx_ = camera_info_msg.K[2];
    ROS_INFO_STREAM("Camera focal length: " << fx_ );
    camera_info_subscriber_.shutdown();
  }



void getImage(const sensor_msgs::Image::ConstPtr& original_image)
{
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    

    cv::Mat img_mask,img_hsv;
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV); 
    cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask); 
//    cv::imshow("redmask", img_mask);
//    cv::waitKey(3);

    cv::Mat contourim( cv_ptr->image.size(), CV_8UC1 );
    contourim = cv::Scalar(0);
    
    std::vector< std::vector< cv::Point2i > > contours;
    cv::findContours(img_mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    
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
    
//    cv::imshow("contours",contourim);
//    cv::waitKey(3);

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

  //  float distance = fx_ * object_width_ / contourwidth;
 float distance = 200 / contourwidth;
    ROS_INFO_STREAM("distance to cylinder: " << distance );

}
int main( int argc, char **argv )
{
  ros::init(argc, argv, "cylinder_detection");
  ros::NodeHandle nh;
  cv::namedWindow("redmask");
  cv::namedWindow("contours");
  camera_info_received_ = false;
  camera_info_subscriber_ = nh.subscribe( "/camera/camera_info", 1, cameraInfoReceived );

  image_transport::ImageTransport it(nh);
 
  
  pub  = it.advertise("/camera/seeing_red", 1);
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, getImage);
  ros::spin();
  return 0;
}

