#include <iostream>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "robo_vision/RMVideoCapture.hpp"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "robo_vision_readcam");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise("usbcamera/image", 1);

    RMVideoCapture cap("/dev/ttyVideo0", 1);
    cap.info();
    cap.setVideoFormat(640, 480, 1);
    cap.getCurrentSetting();
    cap.setExposureTime(0, 64);  // settings->exposure_time);
    cap.startStream();
    ROS_INFO("Image Producer Start!");

    cv::Mat img;
    while (ros::ok()) {
        cap >> img;
        cv_bridge::CvImage img_msg;
        img_msg.header.stamp = ros::Time::now();
        img_msg.header.frame_id = "image";
        img_msg.image = img;
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
        pub_image.publish(img_msg.toImageMsg());
    }
}
