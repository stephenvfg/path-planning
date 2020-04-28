#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // starting lane
  int lane = 1;

  // reference and maximum velocity
  double ref_v = 0.0;
  double max_v = 49.5;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &lane,&ref_v,&max_v]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // size of REMAINING points from previous path since last time interval
          int path_size = previous_path_x.size(); 

          // BEGIN finite state machine assessment

          // END finite state machine assessment; BEGIN sensor fusion data processing

          // if the car has already moved along a path, update our vehicle position
          if (path_size > 0) {
            car_s = end_path_s;
          }

          // store whether or not we are at risk of collision
          bool col_risk_front = false;
          double min_gap = 30.0;

          // cycle through sensior fusion data to understand which cars are nearby
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // check if the detected car is in my lane
            float sense_d = sensor_fusion[i][6];
            if ((sense_d < (2+4*lane+2)) && (sense_d > (2+4*lane-2))) {
              double sense_vx = sensor_fusion[i][3];
              double sense_vy = sensor_fusion[i][4];
              double sense_spd = sqrt(pow(sense_vx,2) + pow(sense_vy,2));
              double sense_s = sensor_fusion[i][5];

              // project where the car will be in the future
              sense_s += ((double)path_size * 0.02 * sense_spd);

              // check if the car is within the minimum gap range, flag collision risk if true
              if ((sense_s > car_s) && ((sense_s - car_s) < min_gap)) {
                col_risk_front = true;
              }
            }
          }

          // update the car velocity based on whether or not it is close to other vehicles
          if (col_risk_front) {
            ref_v -= .224;
          } else if (ref_v < max_v) {
            ref_v += .224;
          }

          // END sensor fusion data processing; BEGIN path generation code

          // creating a list to store widely spaces points to use for a spline to smooth the path
          vector<double> spline_pts_x;
          vector<double> spline_pts_y;

          // define reference x, y, yaw states for the vehicle
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // identify initial path points
          if (path_size < 2) {
            // use two points to define a path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // append these points to the previous points list
            spline_pts_x.push_back(prev_car_x);
            spline_pts_x.push_back(car_x);
            spline_pts_y.push_back(prev_car_y);
            spline_pts_y.push_back(car_y);
          } else {
            // use recent path data to position the vehicle
            ref_x = previous_path_x[path_size-1];
            ref_y = previous_path_y[path_size-1];

            // obtain an additional point to form a path tangent to the car
            double prev_ref_x = previous_path_x[path_size-2];
            double prev_ref_y = previous_path_y[path_size-2];
            ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

            // append these points to the previous points list
            spline_pts_x.push_back(prev_ref_x);
            spline_pts_x.push_back(ref_x);
            spline_pts_y.push_back(prev_ref_y);
            spline_pts_y.push_back(ref_y);
          }

          // add a few additional points spaced out by 30m each (in Frenet) to help create the spline
          vector<double> next_spline0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_spline1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_spline2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // append these points to the previous points list
          spline_pts_x.push_back(next_spline0[0]);
          spline_pts_x.push_back(next_spline1[0]);
          spline_pts_x.push_back(next_spline2[0]);

          spline_pts_y.push_back(next_spline0[1]);
          spline_pts_y.push_back(next_spline1[1]);
          spline_pts_y.push_back(next_spline2[1]);

          // shift coordinates to local vehicle coordinates
          for (int i = 0; i < spline_pts_x.size(); ++i) {
            // calculate the shift to bring the car reference angle to 0 degrees
            double shift_x = spline_pts_x[i] - ref_x;
            double shift_y = spline_pts_y[i] - ref_y;

            // shift the reference for each point
            spline_pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            spline_pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // create a spline for the path and set the points to it
          tk::spline s;
          s.set_points(spline_pts_x, spline_pts_y);

          // define the actual path points
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // collects remanining points from prior path to repurpose for new path 
          for (int i = 0; i < path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up the spline points
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          double x_start = 0.0;

          // complete the path with the help of the spline
          // append as many path points as necessary to extend the previous path to a fresh 50 points
          for (int i = 0; i < 50-path_size; i++) {    
            // place the next point along the spline such that it puts the vehicle at the reference velocity
            double N = (target_dist / (.02*ref_v/2.24));
            double x_point = x_start + (target_x)/N;
            double y_point = s(x_point);

            // update the x starting point
            x_start = x_point;

            // save these values for later
            double x_ref = x_point;
            double y_ref = y_point;

            // rotate (x,y) back to its original position
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            x_point += ref_x;
            y_point += ref_y;

            // add points to the path
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // END path generation code

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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