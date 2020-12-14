#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>

class Move_arm
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_get_xz;
    moveit::planning_interface::MoveGroupInterface arm;
    tf::TransformListener tflisten;
    tf::StampedTransform transform;

public:
    Move_arm() : arm("arm")
    {
        sub_get_xz = nh_.subscribe<std_msgs::Float64MultiArray>("location_of_bottle", 1, &Move_arm::callback_move, this);
    }

    ~Move_arm()
    {
    }

    void callback_move(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        try
        {
            tflisten.lookupTransform("/base_link", "/camera_link",ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        ROS_INFO("subscribe mesg");
        arm.setMaxVelocityScalingFactor(0.1);
        arm.setNamedTarget("home");
        arm.setPoseReferenceFrame("base_link");
        arm.move();

        ROS_INFO("Moving to prepare pose");
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = msg->data[0] + transform.getOrigin().x();
        pose.pose.position.z = 0.0 + transform.getOrigin().z();
        pose.pose.position.y = msg->data[1] + transform.getOrigin().y();

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
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_arm");
    Move_arm MOVEIT;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}