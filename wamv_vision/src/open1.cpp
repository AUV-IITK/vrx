#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

    cv::Mat image_front;
    std::mutex vision_mutex;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
//    vision_mutex.lock();
		image_front = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
//    vision_mutex.unlock();
//    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//    cv::imshow("view", image_front);
//    cv::waitKey(30);

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
  image_transport::Publisher front_thresholded_pub = it.advertise("/camera/image11111111111", 1);
  sensor_msgs::ImagePtr front_image_thresholded_msg ;
  while (ros::ok())
  {
  if(!image_front.empty())
//  cv::resize(image_front, temp_src,  cv::Point(960, 540));
  {
    vision_mutex.lock();
    temp_src = image_front.clone();
    vision_mutex.unlock();
//    cv::imshow("view", temp_src);
//    cv::waitKey(50);
}
else {
  ROS_INFO("Image empty");
}
front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp_src).toImageMsg();
front_thresholded_pub.publish(front_image_thresholded_msg);
  cv::destroyWindow("view");
 	loop_rate.sleep();
  //ros::spin();
  ros::spinOnce();

//  cv::waitKey(30);
//  cv::destroyWindow("view");
}
}
