#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class depth_estimater
{
public:
    depth_estimater();
    ~depth_estimater();
    void depthImageCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth;
};

depth_estimater::depth_estimater()
{
    sub_depth = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &depth_estimater::depthImageCallback, this);
}

depth_estimater::~depth_estimater()
{
}

void depth_estimater::depthImageCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

    int x1, x2, y1, y2;
    int width = msg->width;
    int height = msg->height;
    ROS_INFO("colums %d rows %d", width, height);
    ROS_INFO(" row_step %d point_step%d ", msg->row_step, msg->point_step);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    //ROS_INFO("%d", cloud.is_dense);
    //https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20　によると+xが右、+yが下。imageの座標は
    double sum_x = 0,sum_z = 0;
    int32_t point_sum = 0;
    for (int i = 0; i < 6; ++i)
    {
        int32_t x = width / 2 + i* 20, y = height / 2;
        if (pcl::isFinite(cloud.at(x, y)) != 0)
        {
            auto cl = cloud.at(x,y);
            ROS_INFO("%d,%dx = %lfmm, y = %lfmm, z = %lfmm",x,y, cl.x * 1000, cl.y * 1000, cl.z * 1000);
        }
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_estimater");

    depth_estimater depth_estimater;

    ros::spin();
    return 0;
}
