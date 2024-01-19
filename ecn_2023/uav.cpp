#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <ecn_2023/client_spinner.h>
#include <ecn_2023/srv/target.hpp>

using namespace std::chrono_literals;
using ecn_2023::srv::Target;

constexpr auto dt{0.01};

class UAV : public rclcpp::Node
{
  // helper class to call a service in a synchronous way
  ServiceNodeSync<Target> client{this};

  // PI part
  double Kp{}, Ki{}, Kw{};
  double eix{}, eiy{};

  // TODO declare additional member variables if needed

public:
  UAV() : Node("uav")
  {

    // control params
    Ki = declare_parameter("Ki", .1);
    Kp = declare_parameter("Kp", 10.);
    Kw = declare_parameter("Kw", 5.);

    // TODO declare a parameter for the drone number

    // TODO init the service client

    // TODO init publisher and other things

  }

  // TODO declare additional member functions if needed


};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UAV>());
  rclcpp::shutdown();
}
