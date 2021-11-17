#include <memory>
#include <numeric>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// Vectors to collect moving averages
std::vector<float> temp_avgs{},speed_avgs{},laser_avgs{};

// Files to save the moving averag for plotting
std::ofstream temp_save("temp_save.csv");
std::ofstream speed_save("speed_save.csv");
std::ofstream laser_save("laser_save.csv");

/**
 * @brief This function calculates the average of upto last
 * 20 elements of input vector  
 */

float average(std::vector<float> const& v){
    if(v.empty()){
        return 0;
    }
    auto const count = static_cast<float>(v.size());
    if (count > 20){
        return std::accumulate(v.end()-20, v.end(),0) / 20;
    }
    return std::accumulate(v.begin(), v.end(),0) / count;
}

/**
 * @brief Subsriber class which takes in the node name and, topic name
 * and a input for type of sensorand prints out the average of last 
 * 20 elements in the topic
 */
class SubscriberNode : public rclcpp::Node
{
  public:
    SubscriberNode(
        std::string node_name,
        std::string topic_name,
        char t
    )
    : Node(node_name),sensor_type_(t)
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic_name, 10, std::bind(&SubscriberNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      std::string data_str = msg->data.c_str();
      float num_data = std::stof(data_str);

      if (sensor_type_ =='t')
      {
          temp_avgs.push_back(num_data);
          auto a = std::to_string(average(temp_avgs));
          temp_save << a+"\n";
          RCLCPP_INFO(this->get_logger(), "The moving average for Temperature Sensor"+ a);
      }
      else if (sensor_type_ =='s') {
          speed_avgs.push_back(num_data);
          auto a = std::to_string(average(speed_avgs));
          speed_save << a+"\n";
          RCLCPP_INFO(this->get_logger(), "The moving average for Speed Sensor"+ a);
      }
      else if (sensor_type_ =='l') {
          laser_avgs.push_back(num_data);
          auto a = std::to_string(average(laser_avgs));
          laser_save << a+"\n";
          RCLCPP_INFO(this->get_logger(), "The moving average for Laser Sensor"+ a);
      }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    char sensor_type_;
};

int main(int argc, char * argv[])
{
  // Initializing rclcpp
  rclcpp::init(argc, argv);
  // Enabling multithreading using executor
  rclcpp::executors::MultiThreadedExecutor executor;
  // Initializing subscriber nodes
  auto subnode1 = std::make_shared<SubscriberNode>("Temp_reviever","temp_data", 't');
  auto subnode2 = std::make_shared<SubscriberNode>("Speed_reviever","speed_data", 's');
  auto subnode3 = std::make_shared<SubscriberNode>("Laser_reciever","laser_data", 'l');
  // Adding them to executor nodes
  executor.add_node(subnode1);
  executor.add_node(subnode2);
  executor.add_node(subnode3);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}