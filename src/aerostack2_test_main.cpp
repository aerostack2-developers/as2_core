// "Copyright [year] <Copyright Owner>"

#include "aerostack2_node.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  std::cout << "Starting sensor_combined listener node... " << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorCombinedListener>());

  rclcpp::shutdown();
  return 0;
}
