#include "ego.hpp"
#include "helper.hpp"
#include <iostream>

using namespace std;

Ego::Ego() {
  this->lane = 1; // have ego start off with lane 1

  // lane_left_free_count means a accumulated total if
  this->lane_left_free_count = 0;
  this->lane_right_free_count = 0;
};

Ego::~Ego(){};

void Ego::change_lane(int lane) {
  cout << "Ego Changing Lanes!!!" << endl;
  clear_lane_free_counter(0);
  clear_lane_free_counter(1);
  // 0 = left
  // 1 = middle
  // 2 = right
  switch (lane) {
  case 0:
    this->lane = 0;
    break;
  case 1:
    this->lane = 1;
    break;
  case 2:
    this->lane = 2;
    break;
  default:
    this->lane = 1;
    break;
  }
}

void Ego::clear_lane_free_counter(int lane) {
  if (lane == 0) {
    this->lane_left_free_count = 0;
  }
  if (lane == 1) {
    this->lane_right_free_count = 0;
  }
}

void Ego::decide(double j) {
  cout << "Deciding..." << endl;
  get_ego_lane(j);
}

// what lane are we currently in?
// this will help know which lanes we theoretically have available
// we have to move to a new lane...
// default to the left lane change
// detect if there are or will be any cars in the left lane
// if the above is true, check the right lane

int Ego::get_ego_lane(double j) { cout << "car_d " << j << endl; }
