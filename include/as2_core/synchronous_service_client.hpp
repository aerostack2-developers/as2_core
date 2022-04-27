#ifndef __SYNCHRONOUS_SERVICE_CLIENT_HPP__
#define __SYNCHRONOUS_SERVICE_CLIENT_HPP__

#include <string>

#include "as2_core/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2 {

template <class ServiceT>
class SynchronousServiceClient {
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
  std::string service_name_;

  public:
  SynchronousServiceClient(std::string name)
      : node(std::make_shared<rclcpp::Node>(name)), service_name_(service_name_) {}

  using SharedPtr = std::shared_ptr<SynchronousServiceClient<ServiceT>>;

  void init(std::string service) {
    client = node->create_client<ServiceT>(service);
    client->wait_for_service();
  }

  ResponseT sendRequest(const RequestT &req) {
    return sendRequest(std::make_shared<RequestT>(req));
  }

  // ResponseT sendRequest(const std::shared_ptr<RequestT> &req_ptr) {
  //   auto result = client->async_send_request(req_ptr);
  //   rclcpp::spin_until_future_complete(node, result);
  //   return *result.get();
  // }

  ResponseT sendRequest(const std::shared_ptr<RequestT> &req_ptr) {
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(node->get_logger(), "waiting for service ok");
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for the service. exiting.");
      }
      RCLCPP_INFO(node->get_logger(), "service: %s not available, waiting again...",
                  service_name_.c_str());
    }
    auto result = client->async_send_request(req_ptr);
    if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(1)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "failed to receive response from service '%s'",
                   service_name_.c_str());
    }
    return *result.get();
  }

  protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
};

}  // namespace as2
#endif

