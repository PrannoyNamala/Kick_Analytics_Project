#include <string>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

/**
 * @brief Publisher class which takes in the node name, topic name,
 * and frequency of message in milliseconds and publishes a float on given topic
 */
class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(
    std::string node_name,
    std::string topic_name,
    float message_data,
    std::chrono::nanoseconds period)
  : Node(node_name), count_(0), message_data_(message_data)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        float r = message_data_ + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(3)));
        message.data = std::to_string(r);

        // Extract current thread
        auto curr_thread = string_thread_id();

        // Prep display message
        RCLCPP_INFO(
          this->get_logger(), "\n<<THREAD %s>> Publishing '%s'",
          curr_thread.c_str(), message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(period, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  float message_data_;
};

int main(int argc, char * argv[])
{
  // Initializing rclcpp
  rclcpp::init(argc, argv);
  // Enabling multithreading using executor
  rclcpp::executors::MultiThreadedExecutor executor;
  // Initializing publisher nodes
  auto pubnode1 = std::make_shared<PublisherNode>("Temp_sensor","temp_data", 10.0,33ms);
  auto pubnode2 = std::make_shared<PublisherNode>("Speed_sensor","speed_data", 5,30ms);
  auto pubnode3 = std::make_shared<PublisherNode>("Laser_sensor","laser_data", 1,12ms);
  // Adding them to executor nodes
  executor.add_node(pubnode1);
  executor.add_node(pubnode2);
  executor.add_node(pubnode3);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}