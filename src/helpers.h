#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Returns the list of successor states provided a current state
vector<string> getSuccessorStates(string state) {

  vector<string> states;

  // always consider the option to remain in the current lane
  states.push_back("KL");

  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    states.push_back("PLCL");
    states.push_back("LCL");
  } else if (state.compare("PLCR") == 0) {
    states.push_back("PLCR");
    states.push_back("LCR");
  }
    
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

// Calculates cost of switching into a new lane based on the speed of the vehicle in that lane
// Returns higher costs for lanes with slower vehicles compared to the ego vehicle
float cost_lane_speed(double car_speed, double lane_speed) {

  // Return the delta between the speed of the car and the speed of the lane
  return car_speed - lane_speed;
}

// Calculates cost of switching into a new lane based on the risk of colliding into another vehicle
// Returns higher costs for lanes with a vehicle near the position of the ego vehicle
float cost_lane_change_risk(double car_s, int target_lane, int path_size, vector<vector<double>> &sensor_fusion) {

  float max_cost = 0.0;
  double min_gap = 30.0;

  // cycle through sensior fusion data to understand which cars are nearby
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // check if the detected car is in my lane
    float sense_d = sensor_fusion[i][6];
    if ((sense_d < (2+4*target_lane+2)) && (sense_d > (2+4*target_lane-2))) {
      double sense_vx = sensor_fusion[i][3];
      double sense_vy = sensor_fusion[i][4];
      double sense_spd = sqrt(pow(sense_vx,2) + pow(sense_vy,2));
      double sense_s = sensor_fusion[i][5];

      // project where the car will be in the future
      sense_s += ((double)path_size * 0.02 * sense_spd);

      // check if the car is within the minimum gap range
      // if true, calculate the cost based on the proximity to the ego vehicle
      if ((sense_s > car_s) && ((sense_s - car_s) < min_gap)) {
        float cost = min_gap - fabs(sense_s - car_s);
        if (cost > max_cost) {
          // retian the highest cost so far
          max_cost = cost;
        }
      }
    }
  }

  // Return the cost representing the vehicle closest to the ego car
  return max_cost;
}

// Calculates the cost of a movement based on different cost factors
float calculate_cost(double car_speed, double lane_speed, double car_s, int target_lane, 
                     int path_size, vector<vector<double>> &sensor_fusion) {
  // define weights for the costs
  double lane_speed_weight = 1.0;
  double lane_change_risk_weight = 100.0;

  // calculate costs of individual factors
  float lane_speed_cost = cost_lane_speed(car_speed, lane_speed);
  float lane_change_risk_cost = cost_lane_change_risk(car_s, target_lane, path_size, sensor_fusion);

  // determine the total cost based on individual costs and weights
  return (lane_speed_weight * lane_speed_cost) + (lane_change_risk_weight * lane_change_risk_cost);
}

#endif  // HELPERS_H