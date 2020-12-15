#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    // コンストラクタ
    ImageConverter()
        : it_(nh_)
    {
        // カラー画像をサブスクライブ
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
        // 処理した画像をパブリッシュ
        //image_pub_ = it_.advertise("/image_topic", 1);
    }

    // デストラクタ
    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
    // コールバック関数
    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        ROS_INFO("%d %d", msg->width, msg->height);
        cv::Mat hsv_image, color_mask, gray_image, cv_find_color, cv_image3;
        cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
        // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

        cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);

        cv::inRange(hsv_image, cv::Scalar(150, 100, 100, 0), cv::Scalar(175, 255, 255, 0), color_mask);
        cv::bitwise_and(cv_ptr->image, cv_ptr->image, cv_find_color, color_mask);
        cv::cvtColor(cv_ptr->image, gray_image, CV_BGR2GRAY);
        cv::Canny(gray_image, cv_ptr3->image, 15.0, 30.0, 3);

        int_fast64_t sum_x = 0, sum_y = 0;
        int32_t point_count = 0;
        for (size_t y = 0; y < color_mask.rows; ++y)
        {
            for (size_t x = 0; x < color_mask.cols; ++x)
            {
                if (color_mask.at<uint8_t>(y, x) != 0)
                {
                    sum_x += x;
                    sum_y += y;
                    ++point_count;
                }
            }
        }
        if (point_count != 0)
            ROS_INFO("pt_x = %lld pt_y = %lld", sum_x / point_count, sum_y / point_count);
        cv::Mat cv_half_image, cv_half_find_color, cv_half_image3;
        cv::resize(cv_ptr->image, cv_half_image, cv::Size(), 0.5, 0.5);
        cv::resize(cv_find_color, cv_half_find_color, cv::Size(), 0.5, 0.5);
        cv::resize(cv_ptr3->image, cv_half_image3, cv::Size(), 0.5, 0.5);

        cv::imshow("Original Image", cv_half_image);
        cv::imshow("Result Image", cv_find_color);
        cv::imshow("Edge Image", cv_half_image3);
        cv::waitKey(3);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}