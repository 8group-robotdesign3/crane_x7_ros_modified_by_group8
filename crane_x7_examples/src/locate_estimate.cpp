#include <ros/ros.h>
#include <std_msgs/Int32.h>
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
#include <tf/transform_broadcaster.h>

static const std::string OPENCV_WINDOW = "Image window";

class Locate_estimate_node
{
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    ros::Subscriber sub_depth;
    ros::Publisher pub_activation;

    tf::TransformListener tflisten;
    tf::StampedTransform transform;

    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface gripper;
    bool initialize_ok;
    int32_t point_count;
    int32_t called_count;
    bool called_flag;
    std::vector<std::pair<int32_t, int32_t>> x_z_point;

public:
    Locate_estimate_node()
        : it_(nh_), x_z_point(), point_count(0), arm("arm"), gripper("gripper"), initialize_ok(false), called_count(0),called_flag(false)
    {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &Locate_estimate_node::imageCb, this);
        sub_depth = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &Locate_estimate_node::depthImageCallback, this);
        pub_activation = nh_.advertise<std_msgs::Int32>("activate_node", 1);
        arm.setMaxVelocityScalingFactor(0.1);
        arm.setPoseReferenceFrame("base_link");

        std::vector<double> JointValues = {1.3, 1.3};
        gripper.setJointValueTarget(JointValues);
        if (!gripper.move())
        {
            ROS_WARN("Could not open gripper");
        }
        ROS_INFO("open gripper");

        geometry_msgs::PoseStamped home;
        home.header.frame_id = "base_link";
        home.pose.position.x = 0.2009;
        home.pose.position.y = 0.0112;
        home.pose.position.z = 0.07174 + 0.05;
        auto q = tf::createQuaternionFromRPY(-3.14 / 2.0, 0.0, -3.14 / 2.0);
        home.pose.orientation.x = q.getX();
        home.pose.orientation.y = q.getY();
        home.pose.orientation.z = q.getZ();
        home.pose.orientation.w = q.getW();
        arm.setPoseTarget(home);
        if (!arm.move())
        {
            ROS_WARN("Could not move to home pose");
        }
        ROS_INFO("Moving to home pose");
        initialize_ok = true;
    }

    ~Locate_estimate_node()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv::Mat hsv_image, color_mask;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(140, 100, 100, 0), cv::Scalar(180, 255, 255, 0), color_mask);

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
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        double sum_x = 0.0, sum_z = 0.0, sum_y = 0.0;
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
            }
        }
        ROS_INFO("point sum %d", point_sum);
        ++called_count;
        sum_x /= point_sum;
        sum_z /= point_sum;
        sum_y /= point_sum;
        if ((point_sum > 12000) && (!called_flag) && initialize_ok)
        {
            ROS_INFO("location of object is x = %lfmm,y = %lfmm z = %lfmm", sum_x, sum_y, sum_z);
            called_flag = true;
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
            pose.pose.position.x = transform.getOrigin().x() + sum_z;
            pose.pose.position.z = transform.getOrigin().z() + 0.1;
            pose.pose.position.y = transform.getOrigin().y() - 0.03 + sum_x; 
            ROS_INFO("target position x %f y %f z %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            auto q = tf::createQuaternionFromRPY(-3.14 / 2.0, 0.0, -3.14 / 2.0);
            pose.pose.orientation.x = q.getX();
            pose.pose.orientation.y = q.getY();
            pose.pose.orientation.z = q.getZ();
            pose.pose.orientation.w = q.getW();
            arm.setPoseTarget(pose);
            if (!arm.move())
            {
                ROS_WARN("Could not move to prepare pose");
            }

            std::vector<double> JointValues = {0.29, 0.29};
            gripper.setJointValueTarget(JointValues);
            if (!gripper.move())
            {
                ROS_WARN("Could not close gripper");
            }
            ROS_INFO("close gripper");

            std_msgs::Int32 activation;

            ROS_INFO("called %d", called_count);
            activation.data = point_sum % 4; //乱数
            for (int pub_i = 0; pub_i < 3; ++pub_i)
            {
                pub_activation.publish(activation);
            }
            ros::shutdown();
            //---------------------------------------------------------------
        }
        if (called_count > 300)
        {
            std_msgs::Int32 activation;
            ROS_INFO("called %d", called_count);
            activation.data = point_sum % 3 + 4; //乱数
            for (int pub_i = 0; pub_i < 3; ++pub_i)
            {
                pub_activation.publish(activation);
            }
            ros::shutdown();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "locate_estimate_node");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    Locate_estimate_node Le_node;

    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
