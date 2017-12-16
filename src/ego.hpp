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
  int lane_left_free_count;
  int lane_right_free_count;

  void decide(double j);
  void change_lane(int lane);
  void clear_lane_free_counter(int lane);

  // check if any cars in the lanes next to me
  // this will be called if i'm stuck behind a slow car
  void car_check();

  int get_ego_lane(double j);
};

#endif
