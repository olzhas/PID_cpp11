/**
  * example
  */

#include "pid.hpp"

constexpr double Kp = 0;
constexpr double Ki = 0;
constexpr double Kd = 0;

int main(int argc, char *argv[]) {
  DiscretePIDPtr pid = std::make_shared<DiscretePID>(Kp, Ki, Kd);

  std::function<void(const double &)> writeFn;
  std::function<double(void)> readFn;
  // these two function should be bindings to the actual read from sensor
  // and write to command the actuator
  pid->assignRWFunc(readFn, writeFn);

  pid->setReference(0);
  std::this_thread::sleep_for(std::chrono::seconds(10));

  return 0;
}
