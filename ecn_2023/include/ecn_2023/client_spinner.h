#ifndef CLIENT_SPINNER_H
#define CLIENT_SPINNER_H

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

template <class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
public:
  ServiceNodeSync(rclcpp::Node * parent) : parent{parent}
  {
    assert(("A ServiceNodeSync should be initialized with a valid node as parent", parent != nullptr));
  }

  void init(const std::string& service, std::chrono::milliseconds timeout = 100ms)
  {
    auto name{std::string{parent->get_name()} + '_'};
    std::transform(service.begin(), service.end(), std::back_inserter(name),[](auto c)
    {return c == '/' ? '_' : c;});
    node = std::make_shared<rclcpp::Node>(name + "_client");
    this->service = service;
    this->timeout = timeout;    
  }

  std::optional<ResponseT> call(const RequestT &req)
  {
    ResponseT res;
    if(call(req, res))
      return res;
    return {};
  }

  bool call(const RequestT &req, ResponseT &res)
  {
    return call(std::make_shared<RequestT>(req), res);
  }

  bool call(const std::shared_ptr<RequestT> &req_ptr, ResponseT &res)
  {
    if(!node) return false;
    if(!client) client = node->create_client<ServiceT>(service);

    if(!client->wait_for_service(timeout))
    {
      RCLCPP_WARN(node->get_logger(), "Service %s is not reachable", service.c_str());
      // try to reconnect
      client = node->create_client<ServiceT>(service);
      if(!client->wait_for_service(timeout))
        return false;
      RCLCPP_INFO(node->get_logger(), (std::string("Reconnected to ") + service).c_str());
    }

    auto result = client->async_send_request(req_ptr);
    auto spin_result{rclcpp::spin_until_future_complete(node, result)};

    if(spin_result == rclcpp::FutureReturnCode::SUCCESS)
    {
      res = *result.get();
      return true;
    }
    return false;
  }

protected:
  rclcpp::Node* parent;
  std::string service;
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
  std::chrono::milliseconds timeout;

};

#endif // CLIENT_SPINNER_H
