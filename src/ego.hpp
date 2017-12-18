#ifndef EGO_H
#define EGO_H

using namespace std;

class Ego {
public:
  // Constructor
  Ego();
  // DeConstructor
  virtual ~Ego();

  int lane;
  int allowed_left_counter;
  int allowed_right_counter;
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

  // check if any cars in the lanes next to me
  // this will be called if i'm stuck behind a slow car
  void car_check();

  int get_ego_lane(double j);
};

#endif
