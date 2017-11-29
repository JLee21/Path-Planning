#ifndef EGO_H
#define EGO_H

using namespace std;

class Ego {
public:
  // Constructor
  Ego();
  // DeConstructor
  virtual ~Ego();

  void test();

  // check if any cars in the lanes next to me
  // this will be called if i'm stuck behind a slow car
  void car_check();
};

#endif
