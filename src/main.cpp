#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx_map = j[1]["ptsx"];
          vector<double> ptsy_map = j[1]["ptsy"];
          double px_map = j[1]["x"];
          double py_map = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          //Calculate map waypoint to car coordinate
          int ptsx_size = ptsx_map.size();
          Eigen::VectorXd ptsx_car(6);
          Eigen::VectorXd ptsy_car(6);


          for(int i = 0; i < ptsx_size; i++){

              ptsx_car[i] = (ptsx_map[i] - px_map) * cos(psi) + (ptsy_map[i] - py_map) * sin(psi) ;
              ptsy_car[i] =  -1 * ((ptsx_map[i] - px_map) * sin(psi) - (ptsy_map[i] - py_map) * cos(psi)) ;

          }

          // In car coordinate , the Car's position is 0, 0;
          double px_car = 0.0;
          double py_car = 0.0;


          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          // Try to fit polynomial 1-order , 2-order, 3-order, 5-order , The 3-order is chosen
          //auto coeffs = polyfit(ptsx, ptsy, 1);
          //auto coeffs = polyfit(ptsx, ptsy, 2);
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
          //auto coeffs = polyfit(ptsx, ptsy, 5);

          double cte = polyeval(coeffs, px_car) - py_car;
          //double epsi = psi - atan( coeffs[1]);
          //double epsi = psi - atan( coeffs[1] * pow(px, 0) + 2 * coeffs[2] * pow(px, 1) );
          //double epsi = psi - atan( coeffs[1] * pow(px, 0) + coeffs[2] * pow(px, 1) + coeffs[3] * pow(px, 2) );
          double epsi = psi - atan( coeffs[1] * pow(px_car, 0) + 2 * coeffs[2] * pow(px_car, 1) + 3 * coeffs[3] * pow(px_car, 2) );
          //double epsi = psi - atan( coeffs[1] * pow(px, 0) + 2 * coeffs[2] * pow(px, 1) + 3 * coeffs[3] * pow(px, 2) + 4 * coeffs[4] * pow(px, 3) + 5 * coeffs[5] * pow(px, 4) );



          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //From ptsx_car[0] to ptsx_car[5], fit y_value by step 5
          for(int i = ptsx_car[0]; i <= ptsx_car[5] ; i = i + 5){

              next_x_vals.push_back(i);
              next_y_vals.push_back(polyeval(coeffs, i) );

          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          // In the car coordinate , psi of the car is 0
          psi = 0.0;

          // The state of the car 
          Eigen::VectorXd state(6);
          state << px_car, py_car, psi, v, cte, epsi;

          // Get the optimal values from Solver
          auto vars = mpc.Solve(state, coeffs);

          steer_value = vars[2][0];
          throttle_value = vars[3][0];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          mpc_x_vals = vars[0];
          mpc_y_vals = vars[1];

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //Display the waypoints/reference line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
