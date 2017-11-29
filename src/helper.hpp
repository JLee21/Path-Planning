#ifndef HELPER_H
#define HELPER_H

#include <iostream>
#include <math.h>

// unnamed namespace otherwise we'll get duplicate symbols during compile
namespace {
namespace help {
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void test() { std::cout << "asdf"; }
} // namespace help
} // namespace
#endif
