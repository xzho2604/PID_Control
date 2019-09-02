#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  // ----------------------------------------------------------------------
  // pid.Init(0.15, 2.5,0.0);
  std::vector<double> init_p{0.2,3,0.0};
  // std::vector<double> init_p{0,3,0.0};
  std::vector<double> dp{1,1,1};
  pid.Init(init_p);

  int flag = 0;
  int iteration = 0; // keep record of the iteartion of swiddle
  int loop = 0;
  double best_err = 0.0;
  double best_loop = 0.0;

  // control to twiddle
  std::vector<int> increase{1,1,1};
  std::vector<int> decrease{0,0,0};
  // ----------------------------------------------------------------------

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          // --------------------------------------------------------------------
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          //std::cout << " outside cte:&&&&&&&&&" << cte << std::endl;
          pid.UpdateError(cte); // update the internel error state
          steer_value = -pid.TotalError(); // calculate the stearing using PID control
          
          std::cout << "best loop: " << best_loop<<"cout:" << pid.count_<< 
                        " "<<pid.sq_cte_ << std::endl;
          // get the sum of dp
          auto total = std::accumulate(dp.begin(),dp.end(),0.0);
          std::cout << "cte: "<< fabs(cte) << "sped: " << speed<< std::endl;
            
          /*
          // probabilty car is off track calculate twiddle and rest the game
          if(total > 0.2){ // only swiddle if larger than thresh hold
              auto avg_err = pid.GetAerr(); // get the average cte sqaure error
              auto pram = pid.GetCo(); // get the p array of coefficients

              std::cout << "=================total dp:" << total << "---" <<
                        dp[0]<< ","<<dp[1]<<","<<dp[2]<<std::endl;
              std::cout << "prams: "<<pram[0] << " " << pram[1] << " " << pram[2] <<std::endl;

              if(fabs(cte) > 2) {
                std::vector<double> new_pram;
                if(!flag) {best_loop= 0;} // first time in the loop init best error
                auto round_ind = iteration%1; // this round index update
    
                if (increase[round_ind]) {
                    if(loop > best_loop) {
                        best_loop = loop;
                        dp[round_ind] *= 1.1;

                        increase[round_ind] = 1;  // mark previous action increaes 
                        decrease[round_ind] = 0;
                        
                        pram[round_ind] += dp[round_ind];
                    }else {
                        pram[round_ind] -= 2*dp[round_ind];

                        decrease[round_ind] = 1; // remmeber prevous step decrease 
                        increase[round_ind] = 0;
                    } 
                }
    
                if(decrease[round_ind]) {
                    if(loop > best_loop) { // error imporves
                        best_loop = loop;
                        dp[round_ind] *= 1.1;
                        pram[round_ind] -= dp[round_ind];

                    } else {
                        pram[round_ind] += dp[round_ind];
                        //dp[round_ind] *= 0.9;

                    }
                    increase[round_ind] = 1;  // mark previous action increaes 
                    decrease[round_ind] = 0;

                    pram[round_ind] += dp[round_ind];
                }

                PID pid_new;
                pid_new.Init(pram);
                pid = pid_new;
                ++iteration;
                ++flag; 

                loop = 0;
                // reset simulator
                std::string msg("42[\"reset\", {}]");
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              } 
              
          } else { // indicate of the end of the fine turn p
              auto temp = pid.GetCo();
              std::cout << " ------------------------" << std::endl;
              std::cout<< "stablinsed :" << temp[0] << " " << temp[1] << " " << temp[2] << '\n';
          }
          */
          ++loop;
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
          //          << std::endl;

          // --------------------------------------------------------------------
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
