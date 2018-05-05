#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "robo_vision/RMVideoCapture.hpp"

using namespace std;
using namespace cv;
#define SHOW_INAGE 1
Mat img_recv;

void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    Mat img_temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    ROS_INFO("recv");
    if (img_temp.empty()) {
        ROS_ERROR("Could't not get image");
    } else {
        cv::imshow("view", img_temp);
        img_recv = img_temp.clone();
    }
}

void prosess(Mat &img) {
    int img_size = img.cols * img.rows;
    const uchar *ptr_begin = img.data;
    const uchar *ptr_src = img.data;
    const uchar *ptr_src_end = img.data + img_size * 3;
    vector<Point2f> pt;
    for (; ptr_src != ptr_src_end; ++ptr_src) {
        uchar b = *ptr_src;
        uchar g = *(++ptr_src);
        uchar r = *(++ptr_src);
        if ((r - b) > 90) {
            int pt_y = (ptr_src - ptr_begin) / 3 / img.cols;
            int pt_x = ((ptr_src - ptr_begin) / 3) % img.cols;
            pt.push_back(Point2f(pt_x, pt_y));
        }
    }
    if (pt.size() < 5) return;
    Mat points(pt);
    Mat labels;
    Mat centers;
    kmeans(points, 2, labels,
           TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3,
           KMEANS_PP_CENTERS, centers);
    vector<Point> v_labels = Mat_<Point>(labels);
    cout << centers << endl;
    cout << v_labels << endl;

    circle(img, Point2f(centers.at<float>(0, 0), centers.at<float>(0, 1)), 10,
           Scalar(255, 255, 255), 10);
    circle(img, Point2f(centers.at<float>(1, 0), centers.at<float>(1, 1)), 10,
           Scalar(255, 255, 255), 10);
    imshow("ff", img);
    waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robo_fishcam");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub =
        it.subscribe("usbcamera/image", 1, &image_callback);

    Mat src, src_csm;
    ros::Rate rate(100);
    ROS_INFO("Image FishEye Start!");
    while (ros::ok()) {
        if (img_recv.empty()) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        img_recv.copyTo(src);
        if (SHOW_INAGE) {
            src.copyTo(src_csm);
        }
        prosess(src);
    }
}
