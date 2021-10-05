// "Copyright [year] <Copyright Owner>"

#include "aerostack2_core/node.hpp"
// namespace aerostack2{

    std::string aerostack2::Node::generate_topic_name(const std::string & name){
    if (name.find("/") == 0) {
      return "/" + this->get_drone_id() + name;
    } else {
      return "/" + this->get_drone_id() + "/" + name;
    }
  };

// }