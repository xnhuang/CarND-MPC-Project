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
#include "tools.h"

// for convenience
using json = nlohmann::json;

int main(int argc, const char *argv[]) {
  uWS::Hub h;
  const double Lf = 2.67;
  double ref_v = 40;
  size_t N;
  double dt;
  int polyfit_degree;

  if (argc == 4 ) {
    std::stringstream sstream(argv[1]);
    sstream >> N ;
    dt = strtod( argv[2], NULL ) ;
    polyfit_degree = stoi( argv[3], NULL ) ;
  } else {
    cout << "Use default value" << endl ;
    N = 25;
    dt = 0.05;
    polyfit_degree = 3;
  }

  // MPC is initialized here!
  MPC mpc(N,dt,Lf,ref_v,polyfit_degree,0.0,0.0);
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    Tools tools;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = tools.hasData(sdata);
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
          double a_last = mpc.get_a_init();
          double delta_last = mpc.get_delta_init();
          double Lf_mpc = mpc.get_Lf();

          vector<double> xvals_local;
          vector<double> yvals_local;

          double x_predict = px+v * cos(psi) * 0.1;
          double y_predict = py+v * sin(psi) * 0.1;
          double psi_predict = psi+(-v / Lf_mpc) * delta_last * 0.1;

          tools.coordinate_transform(x_predict,y_predict,psi_predict,ptsx,ptsy,xvals_local,yvals_local);

          long pts_size = xvals_local.size();
          double* ptsx_ptr = &xvals_local[0];
          Eigen::Map<Eigen::VectorXd> ptsx_eigen(ptsx_ptr, pts_size);

          double* ptsy_ptr = &yvals_local[0];
          Eigen::Map<Eigen::VectorXd> ptsy_eigen(ptsy_ptr, pts_size);
          auto coeffs = tools.polyfit(ptsx_eigen,ptsy_eigen, mpc.get_polyfit_degree());
          double epsi = -atan(coeffs[1]);

          double cte = tools.polyeval(coeffs, 0);

          Eigen::VectorXd state(6);
          state[0] = v * cos(0) * 0.1;
          state[1] = v * sin(0) * 0.1;
          state[2] = (-v / Lf_mpc) * delta_last * 0.1;
          state[3] = v + a_last * 0.1;
          state[4] = cte + v*sin(epsi)*0.1;
          state[5] = epsi - (v / Lf_mpc) * delta_last * 0.1;
          state << 0, 0, 0, v, cte, epsi;
          /*
          *
          * Both are in between [-1, 1].
          *
          */
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          auto vars = mpc.Solve(state, coeffs,mpc_x_vals,mpc_y_vals);
          double steer_value = vars[0];
          double throttle_value = vars[1];
          mpc.set_a_init(throttle_value);
          mpc.set_delta_init(steer_value);
          steer_value /= tools.deg2rad(25);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          next_x_vals = xvals_local;
          next_y_vals = yvals_local;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          std::cout <<"cte = "<< cte << std::endl;
          std::cout <<"epsi = "<< tools.rad2deg(epsi) << std::endl;
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
