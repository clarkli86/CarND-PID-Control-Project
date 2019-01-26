#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <cmath>

// for convenience
using json = nlohmann::json;

namespace {
// Number of frames to evalutate
static constexpr size_t N = 200;
// Min sum of dps
static constexpr double TOL = 0.2;

// Run twiddle?
static constexpr bool TWIDDLE = false;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

class Twiddle
{
public:
  Twiddle(PID & pid) : pid_(pid)
  {}

  void Run()
  {
    pid_.Twiddle(Kp_, Ki_, Kd_);
  }

  void Update(double error)
  {
    assert(!std::isinf(error));

    if (!BestErrorInit_)
    {
      BestError_ = error;
      BestErrorInit_ = true;
      return;
    }

    // Stage 1: Apply dps
    if (PIndex_ % 3 == 0)
    {
      *pps_[PIndex_] += *pdps_[PIndex_];
      PIndex_ = (PIndex_ + 1) % 9;
    }
    // Stage 2: Try negating dps
    else if (PIndex_ % 3 == 1)
    {
      if (error < BestError_)
      {
        BestError_ = error;
        BestPID_ = {{Kp_, Ki_, Kd_}};
        *pdps_[PIndex_] *= 1.1;
        // Start twiddling next param
        PIndex_ = (PIndex_ + 2) % 9;
      }
      else
      {
        *pps_[PIndex_] -= 2 * (*pdps_[PIndex_]);
        // Try the second stage twiddling
        PIndex_ = (PIndex_ + 1) % 9;
      }
    }
    // Stage 2: Shrink dps
    else if (PIndex_ % 3 == 2)
    {
      if (error < BestError_)
      {
        BestError_ = error;
        BestPID_ = {{Kp_, Ki_, Kd_}};
      }
      else
      {
        *pps_[PIndex_] += *pdps_[PIndex_];
        *pdps_[PIndex_] *= 0.9;
      }
      PIndex_ = (PIndex_ + 1) % 9;
    }
  }

  double Sum() const
  {
    return Dp_ + Di_ + Dd_;
  }

  void Debug() const
  {
    std::cout << "(" << Kp_ << ", " << Ki_ << ", " << Kd_ << ") (" << Dp_ << ", " << Di_ << ", " << Dd_ << ")" << std::endl;
    std::cout << "Best Error: " << BestError_
              << " Best PID: " "(" << BestPID_[0] << ", " << BestPID_[1] << ", " << BestPID_[2] << ")" << std::endl;
    switch (PIndex_ / 3)
    {
      case 0:
        std::cout << "Adjusting p" << std::endl;
        break;
      case 1:
        std::cout << "Adjusting i" << std::endl;
        break;
      case 2:
        std::cout << "Adjusting d" << std::endl;
    }
  }

private:
  double Kp_ = 0.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;

  double Dp_ = 1.0;
  double Di_ = 1.0;
  double Dd_ = 1.0;

  std::array<double *, 9> pps_  = {{ &Kp_, &Kp_, &Kp_, &Ki_, &Ki_, &Ki_, &Kd_, &Kd_, &Kd_ }};
  std::array<double *, 9> pdps_ = {{ &Dp_, &Dp_, &Dp_, &Di_, &Di_, &Di_, &Dd_, &Dd_, &Dd_ }};

  size_t PIndex_ = 0;

  double BestError_;
  std::array<double, 3> BestPID_;
  bool   BestErrorInit_ = false;

  PID & pid_;
};

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // Found by twiddle
  pid.Init(0.109, 0.0, 0.901);
  // Large P
  //pid.Init(2.0, 0.0, 0.901);
  // Large I
  //pid.Init(0.109, 1.0, 0.901);
  // Small D
  //pid.Init(0.109, 0.0, 0.1);

  Twiddle twiddle { pid };

  h.onMessage([&pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if (TWIDDLE)
          {
            // Number of frames that have been evaluated
            static size_t num_frames = 0;
            // Error in the second N frames
            static double err = 0;
            if (twiddle.Sum() < TOL)
            {
              std::cout << "Twiddle has finished" << std::endl;
            }
            else
            {
              twiddle.Run();

              // Give twiddle some time to work on it
              if (num_frames > N)
              {
                err += cte * cte;
              }

              ++num_frames;
              if (num_frames > N * 2)
              {
                twiddle.Update(err / N);
                num_frames = 0;
                err = 0;
                // To restart the traiing process
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }
            std::cout << num_frames << std::endl;
            twiddle.Debug();
          }

          // Execute PID controller
          {
            pid.UpdateError(cte);
            steer_value = -pid.TotalError();
            //steer_value = std::max(-1.0, steer_value);
            //steer_value = std::min(1.0, steer_value);
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
