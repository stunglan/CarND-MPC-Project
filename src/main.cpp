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
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          cout << " steering_angle: " << delta << std::endl;
          cout << " throttle_value_car: " << a << std::endl;

   
          /*
           * TODO: Calculate steering angle and throttle using MPC.
           *
           * Both are in between [-1, 1].
           *
           */
          json msgJson;
          /*
           //Display the waypoints/reference line
           */
          vector<double> next_x_vals(ptsx.size());
          vector<double> next_y_vals(ptsy.size());
          
          double steer_value;
          double throttle_value;
          
          // find the path for the reference line - correct the coordinates
          double x ;
          double y ;
          for (int i = 0; i < ptsx.size(); i++) {
            x = ptsx[i] - px;
            y = ptsy[i] - py;
            ptsx[i] = x * cos(psi) + y * sin(psi);
            ptsy[i] = y * cos(psi) - x * sin(psi);
            
            next_x_vals[i] = ptsx[i];
            next_y_vals[i] = ptsy[i];
          }
          // the points in the simulator are connected by a Yellow line
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          
          /*
           * calculate steering and throttle, and projected path
           */
          // Convert to correct datatype and fit coeffisents to polyfit
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());
          for (int i = 0 ; i < ptsx.size(); i++) {
            xvals[i] = ptsx[i];
          }
          for (int i = 0 ; i < ptsy.size(); i++) {
            yvals[i] = ptsy[i];
          }
          auto coeffs = polyfit(xvals, yvals, 3);
          

          // find errors and run the solver from lectures
          // submission 2 changed the second parameter as we now use the cars coordinate systems
          double cte = polyeval(coeffs, 0.0);
          // derivative for third order when in the car coordinate system are -atan(coeffs[1]);
          // https://discussions.udacity.com/t/mpc-quiz-help-needed-for-understanding/251178/21
          double epsi = -atan(coeffs[1]);

          
          
          Eigen::VectorXd state(6);
          state[0] = 0;
          state[1] = 0;
          state[2] = 0;
          state[3] = v;
          state[4] = cte;
          state[5] = epsi;
          
          state[0] = v * cos(0) * latency;
          state[1] = v * sin(0) * latency;
          state[2] = (-v / Lf) * delta* latency;
          state[3] = v + a * 0.1;
          state[4] = cte + v*sin(epsi)*0.1;
          state[5] = epsi - (v / Lf) * delta * 0.1;
          
          auto vars = mpc.Solve(state, coeffs);
 
          //vars[0] = vars[0]/deg2rad(25.0); // works smooter and better without normalisation // divide by deg2rad(25)
          //may be that constants minimizers in MPC routine should me increased
          // cout << "vars " << -vars[0] << " deg2rad(25) " << deg2rad(25) << std::endl;
          
          // steering and throttle forst
          steer_value = -vars[0];
          throttle_value = vars[1];
          
          // number of timesteps and size of value vector for projection
          const int N = (int) vars[2];
          const int vars_len = (int) vars[3];

          size_t x_start = 4;
          size_t y_start = x_start + vars_len;
          // projected vector
          vector<double> mpc_x_vals(&vars[x_start],&vars[x_start + vars_len]);
          vector<double> mpc_y_vals(&vars[y_start],&vars[y_start + vars_len]);
          
          /*
          cout << "mpc_x_vals";
          for (int i = 0;i < mpc_x_vals.size();i++) {
            cout  << " " << std::setprecision(5) << mpc_x_vals[i];
          }
          cout   << std::endl;
          
          cout << "mpc_y_vals";
          for (int i = 0;i < mpc_y_vals.size();i++) {
            cout  << " " << std::setprecision(5) << mpc_y_vals[i];
          }
          cout   << std::endl;
          */

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          
          
          
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
