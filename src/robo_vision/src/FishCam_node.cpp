#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "robo_vision/RMVideoCapture.hpp"
#include "robo_vision/FishCamInfo.h"

using namespace std;
using namespace cv;
#define SHOW_INAGE 1
Mat img_recv_left, img_recv_right;

void image_left_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img_temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    //ROS_INFO("recv");
    if (img_temp.empty())
    {
        ROS_ERROR("Could't not get image");
    }
    else
    {
        cv::imshow("view_left", img_temp);
        waitKey(1);
        img_recv_left = img_temp.clone();
    }
}

void image_right_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img_temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    ROS_INFO("recv");
    if (img_temp.empty())
    {
        ROS_ERROR("Could't not get image");
    }
    else
    {
        cv::imshow("view_right", img_temp);
        img_recv_right = img_temp.clone();
    }
}

void prosess(Mat &img, vector<Vec3f> &pt_center)
{
    int img_size = img.cols * img.rows;
    const uchar *ptr_begin = img.data;
    const uchar *ptr_src = img.data;
    const uchar *ptr_src_end = img.data + img_size * 3;
    Mat img_rb(img.rows, img.cols, CV_8UC1);
    uchar *ptr_img_rb = img_rb.data;
    vector<Point2f> pt;
    for (; ptr_src != ptr_src_end; ++ptr_src)
    {
        uchar b = *ptr_src;
        uchar g = *(++ptr_src);
        uchar r = *(++ptr_src);
        if ((r - b) > 120)
        {
            *ptr_img_rb = 255;
            int pt_y = (ptr_src - ptr_begin) / 3 / img.cols;
            int pt_x = ((ptr_src - ptr_begin) / 3) % img.cols;
            pt.push_back(Point2f(pt_x, pt_y));
        }
        else
        {
            *ptr_img_rb = 0;
        }
        ptr_img_rb = ptr_img_rb + 1;
    }

    if (pt.size() < 5)
        return;
    Mat points(pt);
    Mat labels;
    Mat centers;
    kmeans(points, 2, labels,
           TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3,
           KMEANS_PP_CENTERS, centers);
    cout << centers << "   " << countNonZero(labels) << endl;
    pt_center.push_back(Vec3f(centers.at<float>(0, 0), centers.at<float>(0, 1), labels.rows - countNonZero(labels)));
    pt_center.push_back(Vec3f(centers.at<float>(1, 0), centers.at<float>(1, 1), countNonZero(labels)));
    if (SHOW_INAGE)
    {
        circle(img, Point2f(centers.at<float>(0, 0), centers.at<float>(0, 1)), 10,
               Scalar(255, 255, 255), 10);
        circle(img, Point2f(centers.at<float>(1, 0), centers.at<float>(1, 1)), 10,
               Scalar(255, 255, 255), 10);
        imshow("ff", img);
        imshow("rb", img_rb);
        waitKey(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_fishcam");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub_fisheye_list=nh.advertise<robo_vision::FishCamInfo>("/base/fishcam_info",1);

    int cam_left_center, cam_left_radius, cam_left_up, cam_left_down;
    int cam_right_center, cam_right_radius, cam_right_up, cam_right_down;
    private_nh.getParam("cam_left_center", cam_left_center);
    private_nh.getParam("cam_left_radius", cam_left_radius);
    private_nh.getParam("cam_left_up", cam_left_up);
    private_nh.getParam("cam_left_down", cam_left_down);
    private_nh.getParam("cam_right_center", cam_right_center);
    private_nh.getParam("cam_right_radius", cam_right_radius);
    private_nh.getParam("cam_right_up", cam_right_up);
    private_nh.getParam("cam_right_down", cam_right_down);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub_left =
        it.subscribe("fishcamera/left/image", 1, &image_left_callback);
    image_transport::Subscriber image_sub_right =
        it.subscribe("fishcamera/right/image", 1, &image_right_callback);

    Mat src_left, src_right, src_csm_left, src_csm_right;
    ros::Rate rate(10);
    ROS_INFO("Image FishEye Start!");
    while (ros::ok())
    {
        if (img_recv_left.empty())
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        Rect roi_left(Point(cam_left_center - cam_left_radius, cam_left_up), Point(cam_left_center + cam_left_radius, cam_left_down));
        Rect roi_right(Point(cam_right_center - cam_right_radius, cam_right_up), Point(cam_right_center + cam_right_radius, cam_right_down));

        img_recv_left(roi_left).copyTo(src_left);
        //img_recv_right(roi_right).copyTo(src_right);
        if (SHOW_INAGE)
        {
            src_left.copyTo(src_csm_left);
            //src_right.copyTo(src_csm_right);
        }
        vector<Vec3f> pt_center_left;

        prosess(src_left, pt_center_left);

        robo_vision::FishCamInfo fishcam_msg;
        geometry_msgs::Vector3 vec3_msg;
        fishcam_msg.header.stamp=ros::Time::now();
        fishcam_msg.header.frame_id="base_link";

        imshow("ff", src_left);
        waitKey(1);
        //prosess(src_right);
        ros::spinOnce();
        rate.sleep();
    }
}
