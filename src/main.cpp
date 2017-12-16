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

          // Adaptive cruise control
          // and one single lane change
          // loop through all cars in sensor fusion
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // if car is in my lane
            float d = sensor_fusion[i][6];
            if (d < (2 + 4 * ego.lane + 2) && d > (2 + 4 * ego.lane - 2)) {
              cout << "car_id " << sensor_fusion[i][0];
              cout << " d " << d << endl;
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              // project the s value out in time b/c we're using prev_path
              // points
              check_car_s += ((double)prev_size * 0.02 * check_speed);
              // check s values greater than mine and s gap
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                // lower ref_vel so we don't crash
                too_close = true;
              }
            }
          }

          // adjust speed
          if (too_close) {
            ref_vel -= 0.224;
          } else if (ref_vel < 49.5) {
            ref_vel += 0.224;
          }

          // ********************************************************
          int lane_change = ego.lane;

          if (too_close) {
            for (int i = 0; i < sensor_fusion.size(); i++) {

              int enemy_id = sensor_fusion[i][0];
              double enemy_s = sensor_fusion[i][5];
              double enemy_d = sensor_fusion[i][6];

              // try to change to the left lane for passing
              // if enemy_d is to the left of
              // cout << "ego_s " << car_s << " ego_d " << car_d << endl;
              if (enemy_d < car_d) {
                cout << endl;
                cout << "---LEFT--- " << endl;
                cout << "Enemy " << enemy_id << " has d of " << enemy_d << endl;
                // check if the lane is free. if it is
                cout << "Ego_s " << car_s << " enemy_s " << enemy_s << endl;
                cout << (enemy_s + 10 < car_s) << " -- "
                     << (enemy_s - 10 > car_s) << endl;
                // if free space
                if ((enemy_s + 0 < car_s) && (enemy_s - 0 > car_s)) {
                  cout << "***lane_left_free for " << enemy_id
                       << " s: " << enemy_s << " d: " << enemy_d << endl;
                  ego.lane_left_free_count++;
                } else {
                  // ego.clear_lane_free_counter(0);
                }
              }
              if (enemy_d > car_d) {
                cout << "---RIGHT---" << endl;
                cout << "Enemy " << sensor_fusion[i][0] << " has d of "
                     << enemy_d << endl;
                // check if car has similar s as ours
                cout << "Ego_s " << car_s << " enemy_s " << enemy_s << endl;
                cout << (enemy_s + 10 < car_s) << " -- "
                     << (enemy_s - 10 > car_s) << endl;
                if ((enemy_s + 0 < car_s) && (enemy_s - 0 > car_s)) {
                  cout << "***lane_right_free for " << enemy_id
                       << " s: " << enemy_s << " d: " << enemy_d << endl;
                  ego.lane_right_free_count++;
                } else {
                  // ego.clear_lane_free_counter(1);
                }
              }
            }
          }
          cout << "left_lane_count " << ego.lane_left_free_count
               << " right_lane_count " << ego.lane_right_free_count << endl;
          if ((ego.lane_left_free_count > 0) && (ego.lane != 0)) {
            lane_change--;
            ego.change_lane(lane_change);
          }
          if ((ego.lane_right_free_count > 0) && (ego.lane != 2)) {
            lane_change++;
            ego.change_lane(lane_change);
          }

          // choose next state for ego based on returned `prediction`

          // realize next state for ego
          // this should return the ref_vel and the lane
          // since this is taken care of below

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
