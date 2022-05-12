#ifndef __SYNCHRONOUS_SERVICE_CLIENT_HPP__
#define __SYNCHRONOUS_SERVICE_CLIENT_HPP__

#include <memory>
#include <string>

#include "as2_core/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2 {

template <class ServiceT>
class SynchronousServiceClient {
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
  std::string service_name_;
  std::string node_name_;

  public:
  using SharedPtr = std::shared_ptr<SynchronousServiceClient<ServiceT>>;

  SynchronousServiceClient(std::string service_name) {
    service_name_ = service_name;
    node_name_ = service_name;
    // replace all '/' with '_' in name
    std::replace(node_name_.begin(), node_name_.end(), '/', '_');
  }

  bool sendRequest(const RequestT &req, ResponseT &resp, int wait_time = 0) {
    auto resp_ptr = std::make_shared<ResponseT>(resp);
    if (!sendRequest(std::make_shared<RequestT>(req), resp_ptr, wait_time)) {
      return false;
    }
    resp = *resp_ptr.get();
    return true;
  };

  bool sendRequest(const std::shared_ptr<RequestT> &req, std::shared_ptr<ResponseT> &resp,
                   int wait_time = 0) {
    auto node = std::make_shared<rclcpp::Node>(node_name_);
    auto client = node->create_client<ServiceT>(service_name_);

    if (wait_time <= 0) {
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for the service. exiting.");
          return false;
        }
        RCLCPP_INFO(node->get_logger(), "service: %s not available, waiting again...",
                    service_name_.c_str());
      }
    } else {
      if (!client->wait_for_service(std::chrono::seconds(wait_time))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for the service. exiting.");
          return false;
        }
        RCLCPP_INFO(node->get_logger(), "service: %s not available, returning False ",
                    service_name_.c_str());
        return false;
      }
    }

    auto result = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(1)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "failed to receive response from service '%s'",
                   service_name_.c_str());
      return false;
    }

    resp = result.get();
    return true;
  }  // namespace as2

  protected:
};  // namespace as2

}  // namespace as2
#endif

