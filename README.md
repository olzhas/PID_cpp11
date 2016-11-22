# Discrete PID implementation in C++11

Simplistic header only library for discrete PID control.

# Getting started
## Download
clone the repository from github
```
$ git clone git@github.com:olzhas/PID_cpp11.git
```

or follow this [link](https://github.com/olzhas/PID_cpp11/archive/master.zip) to download the zip file.

## Compilation
Copy the pid.hpp to your folder.

### Example source code 
```cpp
#include <pid.hpp>

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

  // pid->setSamplingTime(std::chrono::nanoseconds(1e6));
  // pid->setSaturation(10);
  // pid->setExecuteAtSetReference(false); // uncomment to disable start at void setReference(double)
  
  pid->setReference(0); // this sets the reference and starts the control action
  // pid->start(); // uncomment if you executed setExecuteAtSetReference(false)
  
  // wait until it settles
  std::this_thread::sleep_for(std::chrono::seconds(10));

  return 0;
}
```

### Author
2016 Olzhas Adiyatov, Robotics Department, Nazarbayev University.
