// "Copyright [year] <Copyright Owner>"

#include "sensor.hpp"

namespace as2
{
  namespace sensors
  {

    Camera::Camera(const std::string &id, as2::Node *node_ptr) : GenericSensor(id, node_ptr)
    {
      image_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(this->topic_name_, 10);
      camera_info_publisher_ =
          node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(this->topic_name_ + "/info", 10);
    }

    Camera::~Camera(){};

    void Camera::setup()
    {
      image_transport_ptr_ = std::make_shared<image_transport::ImageTransport>(this->getSelfPtr());
      image_transport::ImageTransport &image_transport_ = *image_transport_ptr_;

      it_publisher_ = image_transport_.advertise(this->topic_name_, 1);
    }

    void Camera::updateData(const sensor_msgs::msg::Image &_img)
    {
      if (setup_)
      {
        RCLCPP_INFO(node_ptr_->get_logger(), "Camera setup starting...");
        setup();
        setup_ = false;
        RCLCPP_INFO(node_ptr_->get_logger(), "Camera setup finished");
      }

      if (this->pub_freq_ != -1)
      {
        this->image_data_ = _img;
      }
      else
      {
        this->publishCameraData(_img);
      }
    }

    void Camera::publishCameraData(const sensor_msgs::msg::Image &_msg)
    {
      it_publisher_.publish(_msg); // image_transport::image
      // this->camera_info_publisher_->publish(this->camera_info_data_);
    }

    void Camera::publishRectifiedImage(const sensor_msgs::msg::Image &msg)
    {
      // Rectify image and publish it
      // this->image_publisher_->publish(msg);
    }

    void Camera::publishCompressedImage(const sensor_msgs::msg::Image &msg)
    {
      // Compress image and publish it
      // this->image_publisher_->publish(msg);
    }

    void Camera::loadParameters(const std::string &file)
    {
      // read configuration file
    }

    std::shared_ptr<rclcpp::Node> Camera::getSelfPtr()
    {
      return node_ptr_->shared_from_this();
    }
  }
}; // namespace as2
