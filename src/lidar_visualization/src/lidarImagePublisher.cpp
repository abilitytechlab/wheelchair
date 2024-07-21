#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.h"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "obstacle_detection/LidarUtility.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class LidarImagePublisher : public rclcpp::Node
{
public:

  LidarImagePublisher()
  : Node("lidar2image") //color/image topic
  { 
    rclcpp::QoS custom_qos_profile(1);
    custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    custom_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    custom_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    custom_qos_profile.liveliness_lease_duration(std::chrono::milliseconds(100));
    custom_qos_profile.deadline(std::chrono::milliseconds(50));

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", rclcpp::SensorDataQoS(), std::bind(&LidarImagePublisher::imageCallback, this, _1));

    obstacleStatus_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "obstacleStatus/Stamped", rclcpp::SensorDataQoS(), std::bind(&LidarImagePublisher::obstacleCallback, this, _1));
    
    lidarimagepublisher_ = this->create_publisher<sensor_msgs::msg::Image>("obstacleStatus/Image", rclcpp::SensorDataQoS());  
    lidarimageCompressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("obstacleStatus/CompressedImage", custom_qos_profile);

    lidarImageTimer_ = this->create_wall_timer(
      33ms, std::bind(&LidarImagePublisher::timer_callback, this));
  }

private:

void drawLine(cv::Mat &image) {

  int line_y = image.rows / 2 ; // Adjust this to your needs
  int line_width = image.cols;
  int left_width = line_width * 0.25;
  //int right_width = line_width * 0.25;
  int center_width = line_width * 0.5;

  cv::Scalar center_color = obstacle.vector.x ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
  cv::line(image, cv::Point(left_width, line_y), cv::Point(left_width + center_width, line_y), center_color, 3);

  cv::Scalar left_color = obstacle.vector.y ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
  cv::line(image, cv::Point(0, line_y), cv::Point(left_width, line_y), left_color, 3);

  cv::Scalar right_color = obstacle.vector.z ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
  cv::line(image, cv::Point(left_width + center_width, line_y), cv::Point(line_width, line_y), right_color, 3);

}

void timer_callback(){

   if (!current_frame_.empty()) {
    auto now = this->get_clock()->now();
    std_msgs::msg::Header header;
    header.stamp = now;
    header.frame_id = "camera_frame";  // Example frame ID

    auto message = cv_bridge::CvImage(header, "bgr8", current_frame_).toImageMsg();
    lidarimagepublisher_->publish(*message);
    // Publish compressed image
    std::vector<uchar> buf;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 20};
    cv::imencode(".jpg", current_frame_, buf,compression_params);
    auto compressed_msg = sensor_msgs::msg::CompressedImage();
    compressed_msg.header = header;  // Maintain header consistency
    compressed_msg.format = "jpeg";
    compressed_msg.data = std::move(buf);
    lidarimageCompressed_->publish(compressed_msg);
  }
    

}

void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
  current_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  drawLine(current_frame_);

}

void obstacleCallback(const geometry_msgs::msg::Vector3Stamped msg){
    obstacle.vector.x = msg.vector.x;
    obstacle.vector.y = msg.vector.y;
    obstacle.vector.z = msg.vector.z;
}



rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr obstacleStatus_subscriber_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lidarimagepublisher_;
rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr lidarimageCompressed_;
rclcpp::TimerBase::SharedPtr lidarImageTimer_;
cv::Mat current_frame_;
geometry_msgs::msg::Vector3Stamped obstacle;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
