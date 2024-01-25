#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <ecn_2023/srv/target.hpp>

///
/// Do not modify this file, it is the UAV target designator
///


using ecn_2023::srv::Target;

inline double getYaw(const geometry_msgs::msg::Quaternion & q)
{
  const auto sqx{q.x * q.x};
  const auto sqy{q.y * q.y};
  const auto sqz{q.z * q.z};
  const auto sqw{q.w * q.w};

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  // normalization added from urdfom_headers
  const auto sarg{-2 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw)};

  if (sarg <= -0.99999)
    return -2 * atan2(q.y, q.x);
  else if (sarg >= 0.99999)
    return 2 * atan2(q.y, q.x);
  return atan2(2 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
}

struct Pose
{
  double x, y, z, yaw;
  explicit inline Pose(const geometry_msgs::msg::Transform &tf)
    : x{tf.translation.x}, y{tf.translation.y}, z{tf.translation.z},
      yaw{getYaw(tf.rotation)}
  {}
};

class Allocator : public rclcpp::Node
{
  tf2_ros::Buffer tf_buffer{get_clock()};
  tf2_ros::TransformListener tf_listener{tf_buffer};

  std::optional<Pose> getTF(const std::string &dst)
  {
    if(!tf_buffer.canTransform("world", dst, tf2::TimePointZero))
      return {};
    return Pose{tf_buffer.lookupTransform("world", dst, tf2::TimePointZero).transform};
  }

public:

  Allocator() : Node("target_computer")
  {
    static auto srv = create_service<Target>("/target",
                                             [&](const Target::Request::SharedPtr request,
                                             Target::Response::SharedPtr response)
    {
      resolve(request->uav, *response);
    });
  }

  void resolve(const std::string& uav_name, Target::Response& response)
  {
    //RCLCPP_WARN(get_logger(), "Got request for %s", uav_name.c_str());


    const auto usv{getTF(uav_name + "/target")};
    if(!usv.has_value())
      return;
    const auto uav{getTF(uav_name + "/base_link")};
    if(!uav.has_value())
      return;

    response.z = usv->z - uav->z;

    const auto dx{usv->x-uav->x};
    const auto dy{usv->y-uav->y};
    response.theta = fmod(atan2(dy, dx)-uav->yaw + M_PI, 2*M_PI)-M_PI;

    const auto c{cos(uav->yaw)};
    const auto s{sin(uav->yaw)};
    response.x = c*dx + s*dy;
    response.y = -s*dx + c*dy;
  }

};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Allocator>());
  rclcpp::shutdown();
}
