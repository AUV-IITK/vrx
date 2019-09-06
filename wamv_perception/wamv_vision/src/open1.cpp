#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

    cv::Mat image_front;
    std::mutex vision_mutex;
    cv::Mat blu;
    cv::Mat blur1;
    cv::Mat mask;
    cv::Mat hsv;
    cv::Rect2d bbox1_;
    cv::Rect2d bbox1;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
		image_front = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "image1");
  cv::Mat temp_src;
  ros::NodeHandle nh;
//  cv::namedWindow("view",cv::WINDOW_NORMAL);
  ros::Rate loop_rate(50);
//  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/front_right_camera/image_raw", 1, imageCallback);
  image_transport::Publisher front_thresholded_pub = it.advertise("/camera/mask_image", 1);
  image_transport::Publisher front_contour_pub = it.advertise("/camera/image_with_contour", 1);
  sensor_msgs::ImagePtr front_image_thresholded_msg ;
  sensor_msgs::ImagePtr front_image_contour_msg ;
  while (ros::ok())
  {
  if(!image_front.empty())
//  cv::resize(image_front, temp_src,  cv::Point(960, 540));
  {
    vision_mutex.lock();
    temp_src = image_front.clone();
    vision_mutex.unlock();
    cv::bilateralFilter(temp_src,blu, 9, 75, 75);
    cv::GaussianBlur(blu,blur1, cv::Size(15, 15), 0);
    cv::cvtColor(blur1,hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0,100,80), cv::Scalar(5, 160, 93), mask);    //range for red (surmark_950410)
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    int ptr=0;
    double big_=0.0;
    cv::findContours (mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    if(contours.size()>0)
    {
    for(size_t i=0; i<contours.size(); i++)
            { cv::Rect2d bbox1_ = boundingRect(cv::Mat(contours[i]));
            if(bbox1_.width*bbox1_.height>big_) { big_ = bbox1_.width*bbox1_.height;  ptr=i; }
            }
    bbox1 = boundingRect(cv::Mat(contours[ptr]));
    rectangle(temp_src, bbox1, cv::Scalar(0, 255, 0), 2, 1);
    }
}
else {
  ROS_INFO("Image empty");
}

front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
front_image_contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
front_thresholded_pub.publish(front_image_thresholded_msg);
front_contour_pub.publish(front_image_contour_msg);
cv::destroyWindow("view");
loop_rate.sleep();
//ros::spin();
ros::spinOnce();

}
}
