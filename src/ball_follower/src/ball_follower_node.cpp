#include "ball_follower/ball_follower_node.hpp"

BallFollower::BallFollower() : Node("ball_follower") {
    RCLCPP_INFO(this->get_logger(), "Initializing Ball Follower Node");

    camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&BallFollower::imageCallback, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void BallFollower::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Image received");

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        RCLCPP_INFO(this->get_logger(), "Image converted to OpenCV format");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat hsv_image, mask;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    RCLCPP_INFO(this->get_logger(), "Image converted to HSV");

    // Define range for yellow color and threshold image
    cv::Scalar lower_yellow(20, 100, 100), upper_yellow(30, 255, 255);
    cv::inRange(hsv_image, lower_yellow, upper_yellow, mask);
    RCLCPP_INFO(this->get_logger(), "Thresholding complete, searching for contours");

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    geometry_msgs::msg::Twist cmd_vel_msg;

    if (contours.empty()) {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z =0.7; // turn right

        RCLCPP_WARN(this->get_logger(), "No contours found");
    } else {
        RCLCPP_INFO(this->get_logger(), "Found %lu contours", contours.size());

        // Calculate centroid and control logic
        cv::Moments M = cv::moments(contours[0]);
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);

        RCLCPP_INFO(this->get_logger(), "Centroid of the largest contour: (%d, %d)", cx, cy);

        int tolerance = 20; // Adjust this value as needed (in pixels or as a percentage of image width)
        int center_x = mask.cols / 2;
        RCLCPP_INFO(this->get_logger(), "Center X: %d", center_x);


       if (cx > (center_x - tolerance) && cx < (center_x + tolerance)) {
            // The centroid is within the tolerance range around the center, so move forward
            cmd_vel_msg.linear.x = 1;
            cmd_vel_msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Centroid is near the center, moving forward");
        } else if (cx < center_x - tolerance) {
            // The centroid is to the left of the tolerance range, so turn left
            cmd_vel_msg.linear.x = 1;
            cmd_vel_msg.angular.z = 0.2;
            RCLCPP_INFO(this->get_logger(), "Centroid is left of the center, turning left");
        } else {
            // The centroid is to the right of the tolerance range, so turn right
            cmd_vel_msg.linear.x = 1;
            cmd_vel_msg.angular.z = -0.2;
            RCLCPP_INFO(this->get_logger(), "Centroid is right of the center, turning right");
        }

    }
    cmd_vel_publisher_->publish(cmd_vel_msg);

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BallFollower>();
    RCLCPP_INFO(node->get_logger(), "Spinning the node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Node shutdown");
    return 0;
}
