#include "ego.hpp"
#include "helper.hpp"
#include <iostream>

using namespace std;

int lane_counter = 50;
int time_counter_value = 199;

Ego::Ego() {
  this->lane = 1; // have ego start off with lane 1

  this->allowed_left_counter = lane_counter;
  this->allowed_right_counter = lane_counter;

  this->allowed_left = true;
  this->allowed_right = true;

  this->allowed_counter = time_counter_value;
};

Ego::~Ego(){};

void Ego::change_lane(int lane) {
  cout << "\n\n*********\n\nEGO CHANGE\n\n********\n " << lane << endl;
  reset_allowed_counter();
  // 0 = left
  // 1 = middle
  // 2 = right
  switch (lane) {
  case 0:
    // blinker
    // ego is far left, so we can go any more left
    this->allowed_left = false;
    this->lane = 0;
    break;
  case 1:
    // ego in middle lane, we can go to left or right
    this->allowed_left = true;
    this->allowed_right = true;
    this->lane = 1;
    break;
  case 2:
    // ego is far right, so we can go any more right
    this->allowed_right = false;
    this->lane = 2;
    break;
  default:
    // ego in middle lane, we can go to left or right
    this->allowed_left = true;
    this->allowed_right = true;
    this->lane = 1;
    break;
  }
}

void Ego::reset_allowed_counter() {
  this->allowed_counter = time_counter_value;
}
void Ego::reset_allowed_left_counter() {
  this->allowed_left_counter = lane_counter;
}
void Ego::reset_allowed_right_counter() {
  this->allowed_right_counter = lane_counter;
}

bool Ego::check_counter() {
  if (this->allowed_counter == 0) {
    return true;
  } else {
    return false;
  }
}
bool Ego::check_left_counter() {
  if (this->allowed_left_counter == 0) {
    return true;
  } else {
    return false;
  }
}
bool Ego::check_right_counter() {
  if (this->allowed_right_counter == 0) {
    return true;
  } else {
    return false;
  }
}

void Ego::decrement_allowed_counter() {
  if (this->allowed_counter > 0) {
    this->allowed_counter--;
  } else {
    this->allowed_counter = 0;
  }
}
void Ego::decrement_left_counter() {
  if (this->allowed_left_counter > 0) {
    this->allowed_left_counter--;
  } else {
    this->allowed_left_counter = 0;
  }
}
void Ego::decrement_right_counter() {
  if (this->allowed_right_counter > 0) {
    this->allowed_right_counter--;
  } else {
    this->allowed_right_counter = 0;
  }
}
