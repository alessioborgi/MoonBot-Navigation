#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class TeleopNode : public rclcpp::Node {
 private:
  float tick_interval_;
  rclcpp::TimerBase::SharedPtr timer_tick_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

 public:
  TeleopNode(float tick_interval)
      : Node("teleop_node"), tick_interval_(tick_interval) {
    // Declare the parameter
    this->declare_parameter<std::string>("robot_namespace");
    if (!this->has_parameter("robot_namespace")) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'robot_namespace' is required but not set.");
      rclcpp::shutdown();
      throw std::runtime_error("Parameter 'laser_ns' is required but not set.");
    }

    // Get the parameter value
    std::string robot_namespace;
    this->get_parameter("robot_namespace", robot_namespace);

    // Create the publisher with the namespace
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + robot_namespace + "/cmd_vel", 10);
    timer_tick_ =
        this->create_wall_timer(std::chrono::duration<float>(tick_interval_),
                                std::bind(&TeleopNode::tick, this));
  }

 private:
  void tick() {
    char c = getch();
    geometry_msgs::msg::Twist twist;

    switch (c) {
      case 'w':
        twist.linear.x = 1.0;
        twist.angular.z = 0.0;
        break;
      case 's':
        twist.linear.x = -1.0;
        twist.angular.z = 0.0;
        break;
      case 'a':
        twist.linear.x = 0.0;
        twist.angular.z = 1.0;
        break;
      case 'd':
        twist.linear.x = 0.0;
        twist.angular.z = -1.0;
        break;
      case 27:  // ESC key
        std::cout << "Exiting..." << std::endl;
        exit(0);
      default:
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        break;
    }

    publisher_->publish(twist);
  }

  // Function to get a single character from the keyboard without echoing it to
  // the console
  char getch() {
    char buf = 0;
    struct termios old;

    // Get the current terminal attributes
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");

    // Disable canonical mode and echo
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    // Set the new terminal attributes
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");

    // Read a single character from the standard input
    if (read(0, &buf, 1) < 0) perror("read()");

    // Restore the old terminal attributes
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");

    return buf;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<TeleopNode> node = std::make_shared<TeleopNode>(0.1);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::cout << "Use WASD keys to control the robot. Press ESC to quit."
            << std::endl;
  while (rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
