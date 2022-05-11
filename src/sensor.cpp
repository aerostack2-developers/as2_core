// "Copyright [year] <Copyright Owner>"

#include "sensor.hpp"

namespace as2
{
  namespace sensors
  {

    Camera::Camera(const std::string &id, as2::Node *node_ptr) : GenericSensor(id, node_ptr)
    {
      image_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
      camera_info_publisher_ =
          node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name_ + "/info", 10);
    }

    Camera::~Camera(){};

    void Camera::setup()
    {
      image_transport_ptr_ = std::make_shared<image_transport::ImageTransport>(getSelfPtr());
      image_transport::ImageTransport &image_transport_ = *image_transport_ptr_;

      it_publisher_ = image_transport_.advertise(topic_name_, 1);

      setup_ = false;
    }

    void Camera::updateData(const sensor_msgs::msg::Image &_img)
    {
      if (setup_)
      {
        setup();
      }

      if (pub_freq_ != -1)
      {
        image_data_ = _img;
      }
      else
      {
        publishCameraData(_img);
      }
    }

    void Camera::publishCameraData(const sensor_msgs::msg::Image &_msg)
    {
      it_publisher_.publish(_msg);

      if (camera_info_available_)
      {
        camera_info_publisher_->publish(camera_info_data_);
      }
    }

    void Camera::publishRectifiedImage(const sensor_msgs::msg::Image &msg)
    {
      // Rectify image and publish it
      // image_publisher_->publish(msg);
    }

    // void Camera::publishCompressedImage(const sensor_msgs::msg::Image &msg)
    // {
    //   // Compress image and publish it
    //   // image_publisher_->publish(msg);
    // }

    void Camera::loadParameters(const std::string &file)
    {
      // read configuration file
    }

    void Camera::setParameters(const sensor_msgs::msg::CameraInfo &_camera_info)
    {
      camera_info_data_ = _camera_info;
      camera_info_available_ = true;
    }

    std::shared_ptr<rclcpp::Node> Camera::getSelfPtr()
    {
      return node_ptr_->shared_from_this();
    }
  }
}; // namespace as2
