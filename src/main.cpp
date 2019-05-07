#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
// Include spline library acc. Q&A
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

  // start in middle lane (from Q&A)
  int own_lane = 1;

  // have a reference velocity to target for (from Q&A)
  double ref_vel = 0.0; //in mph

  h.onMessage([&ref_vel, &own_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          // from Q&A
          int prev_size = previous_path_x.size();

          if (prev_size > 0)
          {
              car_s = end_path_s;
          }

          // Other car predictions
          bool other_car_ahead = false;
          bool other_car_left = false;
          bool other_car_right = false;
          int other_car_lane;
          int lane_diff;

          // Determine if there are other cars in front
          // Determin if there are other cars to left and right where overtaking should not be done
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
              // Determine other cars lane
              float d = sensor_fusion[i][6];
              if(d > 0 && d < 4)
              {
                  other_car_lane = 0;
              }
              else if (d > 4 && d < 8)
              {
                  other_car_lane = 1;
              }
              else if (d > 8 && d < 12)
              {
                  other_car_lane = 2;
              }
              // Find other car's speed
              double other_vx = sensor_fusion[i][3];
              double other_vy = sensor_fusion[i][4];
              double check_speed = sqrt(other_vx*other_vx+other_vy*other_vy);
              double check_car_s = sensor_fusion[i][5];

              // if using previous points can project s value out for other car
              check_car_s +=  ((double)prev_size*0.02*check_speed);
              lane_diff = other_car_lane - own_lane;

              // Determine if there is an other car ahead of mine, or left or right of mine
              if(lane_diff == 0)
              {
                  // An other car is in my lane
                  if((check_car_s > car_s) && (check_car_s - car_s < 30))
                  {
                      // if an other car is in front of us and gap is too small (<30)
                      other_car_ahead = true;
                  }
              }
              if(lane_diff == -1)
              {
                  // An other car is in lane left of mine
                  if((check_car_s - car_s < 30) && (check_car_s - car_s > -30))
                  {
                      // if an other car is in left lane and gap behind and in front is too small (<30)
                      other_car_left = true;
                  }
              }
              if(lane_diff == 1)
              {
                  // An other car is in right left of mine
                  if((check_car_s - car_s < 30) && (check_car_s - car_s > -30))
                  {
                      // if an other car is in right lane and gap behind and in front is too small (<30)
                      other_car_right = true;
                  }
              }

          }

          // Behavior planning
          if(other_car_ahead)
          {
              if(own_lane > 0 && !other_car_left)
              {
                  // if there is a left lane and no other car to avoid in the left lane
                  own_lane--;
              }
              else if (own_lane < 2 && !other_car_right)
              {
                  // if there is a right lane and no other car to avoid in the right lane
                  own_lane++;
              }
              else
              {
                  ref_vel -= 0.224;
              }
          }
          else
          {
              // if no car car ahead
              if(own_lane != 1)
              {
                  if((own_lane == 0 && !other_car_right) || (own_lane == 2 && !other_car_left))
                  {
                      // if safe to change left from left or right to the middle lane
                      // we want to be in middle lane for more future overtaking options
                      own_lane = 1;
                  }
              }

              // if no car ahead want to increase speed up to speed limit
              if(ref_vel < 49.5)
              {
                  ref_vel += 0.224;
              }
          }

          // Path planning (from Q&&A)

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m (from Q&A)
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x,y,yaw states (from Q&A)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if prev size is almost empty, use the car as starting reference
          if(prev_size < 2)
          {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);

          }
          else
          {
              // Refine reference state as previous path end points
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use two points that make the path tangent to the previous path end points
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

          }

          // In Frenet add evently spaced 30m points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*own_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*own_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*own_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i = 0; i < ptsx.size(); i++)
          {
              // Shift car reference angle to zero degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline s;

          // Set (x,y) points to the spline
          s.set_points(ptsx,ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous points from last time
          for(int i = 0; i < previous_path_x.size(); i++)
          {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired velocity
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;

          // Fill up the rest of the path planner
          // After filling with previous path points now fill up to 50 points
          for(int i = 0; i < 50-previous_path_x.size(); i++)
          {
              double N = target_dist/(0.02*ref_vel/2.24); //converted ref_vel to m/s
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Rotate back to normal after rotating it earlier
              x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

          }


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
