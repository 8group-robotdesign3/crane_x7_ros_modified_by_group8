#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <algorithm>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber sub_depth;
    ros::Publisher pub_coordinate;
    tf::TransformListener tflisten;
    tf::StampedTransform transform;

    int32_t point_count;
    bool called_flag;
    std::vector<std::pair<int32_t, int32_t>> x_z_point;

public:
    moveit::planning_interface::MoveGroupInterface arm;

    ImageConverter()
        : it_(nh_), x_z_point(), point_count(0), arm("arm")
    {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
        sub_depth = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &ImageConverter::depthImageCallback, this);
        pub_coordinate = nh_.advertise<std_msgs::Float64MultiArray>("location_of_bottle", 1);
        called_flag = false;
        ROS_INFO("aaa");
        arm.setMaxVelocityScalingFactor(0.1);
        arm.setPoseReferenceFrame("base_link");
        geometry_msgs::PoseStamped home;
        home.header.frame_id = "base_link";
        home.pose.position.x = 0.211;
        home.pose.position.y = 0.010;
        home.pose.position.z = 0.020;
        auto q = quaternion_fro
        home.pose.orientation.x = -0.702;
        home.pose.orientation.y = 0.0295 ;
        home.pose.orientation.z = 0.0223 ;
        home.pose.orientation.w = 0.7107 ;
        arm.setPoseTarget(home);
        if (!arm.move())
        {
            ROS_WARN("Could not move to home pose");
        }
        ROS_INFO("Moving to home pose");
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv::Mat hsv_image, color_mask;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(150, 100, 100, 0), cv::Scalar(175, 255, 255, 0), color_mask);

        x_z_point.clear();
        point_count = 0;
        for (int32_t y = 0; y < color_mask.rows; ++y)
        {
            for (int32_t x = 0; x < color_mask.cols; ++x)
            {
                if (color_mask.at<u_int8_t>(y, x) != 0)
                {
                    x_z_point.push_back({x, y});
                    ++point_count;
                }
            }
        }
    }

    void depthImageCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {

        int width = msg->width;
        int height = msg->height;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        //ROS_INFO("%d", cloud.is_dense);
        //+xが右、+yが下。imageの座標は
        double sum_x = 0, sum_z = 0, sum_y;
        int32_t point_sum = 0;
        for (auto &x_y : x_z_point)
        {
            if (pcl::isFinite(cloud.at(x_y.first, x_y.second)) != 0)
            {
                auto cl = cloud.at(x_y.first, x_y.second);
                sum_x += cl.x;
                sum_z += cl.z;
                sum_y += cl.y;
                ++point_sum;
                //ROS_INFO("%d,%d x = %lfmm, z = %lfmm", x_y.first, x_y.second, cl.x * 1000, cl.z * 1000);
            }
        }
        sum_x /= point_sum;
        sum_z /= point_sum;
        sum_y /= point_sum;
        if ((point_sum > 70000) && (!called_flag))
        {
            //m単位になってる
            //std_msgs::Float64MultiArray send_msg;
            //send_msg.data.resize(2);
            //send_msg.data[0] = sum_x;
            //send_msg.data[1] = sum_z;
            //pub_coordinate.publish(send_msg);
            ROS_INFO("x = %lfmm,y = %lfmm z = %lfmm", sum_x, sum_y, sum_z);
            called_flag = true;
            //---------------------------------------------------------
            try
            {
                tflisten.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "base_link";
            ROS_INFO("x %f y %f z %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            pose.pose.position.x = sum_z + transform.getOrigin().x();
            pose.pose.position.z = sum_y + transform.getOrigin().z();
            pose.pose.position.y = sum_x + transform.getOrigin().y() - 0.09;
            ROS_INFO("x %f y %f z %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.707106;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.707106;
            arm.setPoseTarget(pose);
            if (!arm.move())
            {
                ROS_WARN("Could not move to prepare pose");
            }
            ros::shutdown();
            //---------------------------------------------------------------
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ImageConverter ic;
    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}