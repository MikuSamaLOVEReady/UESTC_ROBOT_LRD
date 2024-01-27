#include "pibot/pibot_facade.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pibot_facade");

  PibotManager* manager = new PibotManager();

  MultiThreadedSpinner s;
  spin(s);

  return 0;
}
