// "Copyright [year] <Copyright Owner>"

#include "aerostack2_core/node.hpp"
// namespace aerostack2{

    std::string aerostack2::Node::generate_topic_name(const std::string & name){
    if (name.find("/") == 0) {
      return name.substr(1);
    } else {
      return name;
    }
  };

// }