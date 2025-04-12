#include "monitoring/tf_to_path.hpp"

TFToPath::TFToPath() : Node("tf_to_path"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Declare and get parameters
  parent_frame_ = this->declare_parameter<string>("parent_frame", "world");
  child_frame_ = this->declare_parameter<string>("child_frame", "robot");
  string path_topic = this->declare_parameter<string>("path_topic", "path");
  float publish_rate = this->declare_parameter<float>("publish_rate", 1.0);

  // Initialize publisher
  path_pub_ = this->create_publisher<Path>(path_topic, 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / publish_rate)),
    std::bind(&TFToPath::timer_callback, this));

  // Initialize path message
  path_msg_.header.frame_id = parent_frame_;
}

void TFToPath::timer_callback()
{
  TransformStamped transform_stamped;

  try
  {
    // Lookup the transform
    transform_stamped = tf_buffer_.lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

    // Create a PoseStamped message from the transform
    PoseStamped pose_stamped;
    pose_stamped.header = transform_stamped.header;
    pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
    pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
    pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
    pose_stamped.pose.orientation = transform_stamped.transform.rotation;

    // Append the pose to the path
    path_msg_.header.stamp = this->get_clock()->now();
    path_msg_.poses.push_back(pose_stamped);

    // Publish the path
    path_pub_->publish(path_msg_);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Could not transform %s to %s: %s",
      child_frame_.c_str(), parent_frame_.c_str(), ex.what());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFToPath>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}