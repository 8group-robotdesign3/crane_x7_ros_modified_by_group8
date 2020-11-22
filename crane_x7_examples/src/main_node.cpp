#include <ros/ros.h>
#include <string>
#include <std_msgs/Int32.h>
#include <cstdint>
class MainNode
{
public:
    MainNode(ros::NodeHandle &nodeHandle);

    ~MainNode();

    void send(void);

private:
    enum class PROG : int32_t
    {
        start_node1 = 0,
        end_node1,   //1
        start_node2, //2
        end_node2,   //3
        start_node3, //4
        end_node3,   // 5
        start_node4, // 6
        end_node4    //7
    };

    ros::NodeHandle &nh;

    PROG progress;

    void topicCallback(const std_msgs::Int32 &data);

    ros::Subscriber sub;

    ros::Publisher pub;

    std::string Topic_name; // report_progress
};

MainNode::MainNode(ros::NodeHandle &nodehandle) : nh(nodehandle), Topic_name("report_progress"), progress(PROG::start_node1)
{
    sub = nh.subscribe(Topic_name, 1, &MainNode::topicCallback, this);
    pub = nh.advertise<std_msgs::Int32>("activate_node", 1);
    ROS_INFO("Successfully launched main_node");
}

MainNode::~MainNode()
{
}
void MainNode::topicCallback(const std_msgs::Int32 &data)
{
    PROG receive = static_cast<PROG>(data.data);
    if (receive == PROG::end_node1)
    {
        progress = PROG::start_node2;
        ROS_INFO("start_node2");
    }
    else if (receive == PROG::end_node2)
    {
        progress = PROG::start_node3;
        ROS_INFO("start_node3");
    }
    else if (receive == PROG::end_node3)
    {
        ROS_INFO("finish performance");
    }
    return;
}

void MainNode::send()
{
    std_msgs::Int32 msg;
    int32_t sending_data = static_cast<int32_t>(progress);
    msg.data = sending_data;
    //ROS_INFO("%d", sending_data);
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nodeHandle("~");
    MainNode main_node(nodeHandle);
    ros::Rate r(100);
    while (ros::ok())
    {
        main_node.send();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}