#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

// M Y  C O D E
#include "ego.hpp"
#include "helper.hpp"

using namespace std;
using namespace help;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  double ref_vel = 0.0; // mph
  int lane = 1;         // start in lane 1
  int lane_width = 4;   // lane_width/2 == middle of lane
  Ego ego;

  h.onMessage([&lane_width, &ego, &ref_vel, &lane, &map_waypoints_x,
               &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx,
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"]; // in degrees
          double car_speed = j[1]["speed"];
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          // Sensor Fusion
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          vector<int> close_s;
          vector<int> enemy_exists_right_lane;
          vector<int> enemy_exists_left_lane;

          // Adaptive cruise control
          // and one single lane change
          // loop through all cars in sensor fusion
          printf("\nSensor Fusion Size: %2d\n\n", sensor_fusion.size());
          for (int i = 0; i < sensor_fusion.size(); i++) {

            // print if car has simlar S position as Ego
            int enemy_id = (int)sensor_fusion[i][0];
            double enemy_s = sensor_fusion[i][5];
            double enemy_d = sensor_fusion[i][6];
            double enemy_x = sensor_fusion[i][1];
            double enemy_y = sensor_fusion[i][2];
            float thresh_s = 35;
            float thresh_s_forw = 12;
            int min_vel_to_change = 40;

            // print all enemys regardless if they're close
            // printf("Enemy %02d -- Enemy_s %5.2f -- Enemy_d %5.2f\n", enemy_id,
            //        enemy_s, enemy_d);

            // if enemy is outside ego lane
            // if (enemy_d > (2 + lane_width * ego.lane + 2) &&
            //     enemy_d < (2 + lane_width * ego.lane - 2)) {
            //   if ((enemy_s - car_s) < thresh_s_forw) {
            //     // printf("\tEnemy %02d has close S of %5.2f -- Ego S %5.2f\n",
            //     //        enemy_id, enemy_s, car_s);
            //     close_s.push_back(enemy_id);
            //     enemy_exists_left_lane.push_back(enemy_id);
            //   }
            //   if ((car_s - enemy_s) < thresh_s) {
            //     // printf("\tEnemy %02d has close S of %5.2f -- Ego S %5.2f\n",
            //     //        enemy_id, enemy_s, car_s);
            //     close_s.push_back(enemy_id);
            //   }
            // }

            // if car is in my lane
            if (
                (enemy_d < (2 + lane_width * ego.lane + 2.5)) &&
                (enemy_d > (2 + lane_width * ego.lane - 1.5))
               )
            {

              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              // project the s value out in time b/c we're using prev_path points
              check_car_s += ((double)prev_size * 0.02 * check_speed);
              // check s values greater than mine and s gap
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                too_close = true;
              }
              // find all other cars in other lanes and see if they are S-close
            } else {
              // if ((abs(enemy_s - car_s) < thresh_s_forw) ||
              //    (abs(car_s - enemy_s) < thresh_s)) {
              //
              //      if (enemy_d < (lane_width * ego.lane + 2 - 2) &&
              //          enemy_d > (lane_width * ego.lane + 2 - lane_width)) {
              //        enemy_exists_left_lane.push_back(enemy_id);
              //        ego.reset_allowed_left_counter();
              //      }
              //      // if enemy is to the immmediate right of ego
              //      if (enemy_d > (2 + lane_width * ego.lane + 2) &&
              //          enemy_d < (lane_width + lane_width * ego.lane + 2)) {
              //        enemy_exists_right_lane.push_back(enemy_id);
              //        enemy_exists_right_lane.push_back(enemy_id);
              //        ego.reset_allowed_right_counter();
              //      }
              //    }

              double diff_s = enemy_s - car_s;
              if (
                   (abs(enemy_s - car_s) < thresh_s_forw) ||
                   (abs(car_s - enemy_s) < thresh_s)
                 )
              {
                // printf("\tEnemy %02d has close S of %5.2f -- Ego S %5.2f\n",
                //        enemy_id, enemy_s, car_s);

                // print lane enemy is in
                // printf("\t\t is in oth lane -- Enemy_d %5.2f -- Ego_d %5.2f\n",
                //        enemy_d, car_d);
                // if enemy is to the immmediate left of ego
                //
                // enemy_d < (4 * 2 + 2 - 2)
                // enemy_d < 8
                // &&
                // enemy_d > (4 * 2 + 2 - 4 -1.5)
                // enemy_d > 4.5
                //
                //
                //
                if (
                    (enemy_d < (lane_width * ego.lane + 2 - 2)) &&
                    (enemy_d > (lane_width * ego.lane + 2 - lane_width - 1.5))
                   )
                {
                  printf("  Enemy  Left  lane -- Enemy_d %5.2f -- Ego_d %5.2f -- Diff_s %+6.2f\n",
                         enemy_d, car_d, diff_s);
                  enemy_exists_left_lane.push_back(enemy_id);
                  ego.reset_allowed_left_counter();
                }
                // if enemy is to the immmediate right of ego
                if (
                    (enemy_d > (2 + lane_width * ego.lane + 2)) &&
                    (enemy_d < (lane_width + lane_width * ego.lane + 2.4))
                   )
                {
                  printf("  Enemy  Right lane -- Enemy_d %5.2f -- Ego_d %5.2f -- Diff_s %+6.2f\n",
                         enemy_d, car_d, diff_s);
                  enemy_exists_right_lane.push_back(enemy_id);
                  ego.reset_allowed_right_counter();
                }
              }
            }
          }
          ego.decrement_allowed_counter();
          printf("Allowed counter --> %3d\n", ego.allowed_counter);
          printf("Allowed counter LEFT --> %3d\n", ego.allowed_left_counter);
          printf("Allowed counter RIGHT --> %3d\n", ego.allowed_right_counter);

          // D E B U G
          if (enemy_exists_left_lane.size() == 0) {
            cout << "EFT FREE ";
            if (ego.allowed_left) {
              cout << " and ALLOWED LEFT" << endl;
              ego.decrement_left_counter();
            } else {
              cout << " ego in far LEFT lane" << endl;
            }
          } else {
            cout << "LEFT NO with #" << enemy_exists_left_lane.size() << endl;
          }
          if (enemy_exists_right_lane.size() == 0) {
            cout << "RIGHT FREE";
            if (ego.allowed_right) {
              cout << " and ALLOWED RIGHT" << endl;
              ego.decrement_right_counter();
            } else {
              cout << " ego in far RIGHT lane" << endl;
            }
          } else {
            cout << "RIGHT NO with #" << enemy_exists_right_lane.size() << endl;
          }

          // adjust speed
          if (too_close) {
            ref_vel -= 0.224;
          } else if (ref_vel < 48.3) {
            ref_vel += 0.3;
          }

          // ********************************************************
          int lane_change = ego.lane;

          if (too_close) {
            // C H A N G E  L E F T
            if ((ego.allowed_left) &&
                (enemy_exists_left_lane.size() == 0) &&
                (ego.check_counter()) &&
                (ego.check_left_counter()) &&
                (ref_vel > min_vel_to_change)) {
              ego.change_lane(ego.lane - 1);
              ego.reset_allowed_left_counter();
            }
            // C H A N G E  R I G H T
            if ((ego.allowed_right) &&
                (enemy_exists_right_lane.size() == 0) &&
                (ego.check_counter()) &&
                (ego.check_right_counter()) &&
                (ref_vel > min_vel_to_change)) {
              ego.change_lane(ego.lane + 1);
              ego.reset_allowed_right_counter();
            }
          }
          enemy_exists_left_lane.clear();
          enemy_exists_right_lane.clear();

          // *************************** E N D *****************************

          // create list of widely-spaced map_waypoints_s
          vector<double> ptsx;
          vector<double> ptsy;
          // ref x,y,yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          // cout << " ref_x " << ref_x;
          // cout << " ref_y " << ref_y;
          // cout << " ref_yaw " << ref_yaw;
          // cout << " ref_vel " << ref_vel;
          // cout << " car_yaw " << car_yaw << endl;

          // if prev size almost empty, use the car as starting ref
          if (prev_size < 2) {
            // use two points that make the path tangent to ego
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // redefine ref state as prev path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            // use two pts that make the path tangent to the prev path's end pt
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // din Frenet evenly add 30m spaced points ahead of the starting ref
          vector<double> next_wp0 =
              getXY(car_s + 30, (2 + 4 * ego.lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 =
              getXY(car_s + 60, (2 + 4 * ego.lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 =
              getXY(car_s + 90, (2 + 4 * ego.lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // translate global coords to local ego coords
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
          // create a spline
          tk::spline s;
          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all of the prev path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calc how to break up spline points so that we travel at our desired
          // ref speed
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist =
              sqrt((target_x) * (target_x) + (target_y) * (target_y));
          double x_add_on = 0;

          // fill up the rest of our path planner after filling it with the
          // previous pts
          for (int i = 0; i <= 50 - previous_path_x.size(); i++) {
            double N = (target_dist / (0.02 * ref_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            x_point += ref_x;
            y_point += ref_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
