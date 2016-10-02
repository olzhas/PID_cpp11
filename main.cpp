#include <iostream>

#include "pid.hpp"

int main(int argc, char *argv[]) {
  DiscretePID pid(0.1, 0, 0);

  // pid.assignReadFunc(std::bind);
  // pid.assignWriteFunc(std::bind);

  pid.setReference(1);

  return 0;
}
