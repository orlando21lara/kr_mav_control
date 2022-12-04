#include "mpc_input_bridge.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_input_bridge");
  MPCInputBridge controller_bridge;

  // Only need to call ros spin since we only publish when we receive a messages
  ros::spin();

  return 0;
}


