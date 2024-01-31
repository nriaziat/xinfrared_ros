#include "xtherm.h"
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h> 

using namespace cv;

int main(int argc, char **argv)
{
    XTherm xtherm = XTherm();
    Mat frame;
    float temperatureData[384 * 292];
    int ret = 0;
    ros::init(argc, argv, "xtherm_node");
    ros::NodeHandle nh;
    ros::Publisher temp_pub = nh.advertise<std_msgs::Float32MultiArray>("temperature", 1);
    ros::Publisher im_pub = nh.advertise<sensor_msgs::Image>("image", 1);
    ros::Rate loop_rate(25);
    cv::namedWindow("frame", cv::WINDOW_NORMAL);
    while (ros::ok())
    {
        ret = xtherm.getFrame(frame);        
        if (ret == 1)
        {
            sensor_msgs::Image im_msg;
            imshow("frame", frame);
            waitKey(1);
            im_msg.header.stamp = ros::Time::now();
            im_msg.height = frame.rows;
            im_msg.width = frame.cols;
            im_msg.encoding = "mono8";
            im_msg.is_bigendian = false;
            im_msg.step = frame.cols;
            im_msg.data.clear();
            for (int i = 0; i < frame.rows; i++)
            {
                for (int j = 0; j < frame.cols; j++)
                {
                    im_msg.data.push_back(frame.at<uchar>(i, j));
                }
            }
            im_pub.publish(im_msg);
            cv::imshow("frame", frame);
        }
        loop_rate.sleep();
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }
    xtherm.close_device();
    cv::destroyAllWindows();
    return 0;
}