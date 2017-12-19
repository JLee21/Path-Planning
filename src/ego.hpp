#ifndef EGO_H
#define EGO_H

using namespace std;

class Ego {
public:
  // Constructor
  Ego();
  // DeConstructor
  virtual ~Ego();
  // the lane number (0, 1, 2) that the ego resides in
  int lane;
  // lane_left_free_count means a accumulated total if
  int allowed_left_counter;
  int allowed_right_counter;
  // flags so if the car is on the far right/left lanes
  bool allowed_left;
  bool allowed_right;
  // create a counter that has to countdown to 0 before the ego can change lanes
  int allowed_counter;

  void decide(double j);
  void change_lane(int lane);
  void clear_lane_free_counter(int lane);

  void reset_allowed_counter();
  void reset_allowed_left_counter();
  void reset_allowed_right_counter();
  bool check_counter();
  bool check_left_counter();
  bool check_right_counter();
  void decrement_allowed_counter();
  void decrement_left_counter();
  void decrement_right_counter();

};

#endif
