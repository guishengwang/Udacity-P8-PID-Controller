#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

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
  double p[3]  = {0.13,  0.000027, 2.7};           //0.1199, 0.00002,  2.7}; //twiddle start point {0.15, 0.0005, 3}
  double dp[3] = {0.005, 0.000002, 0.10};
  double tol = 0.002;
  double best_p[3] = {0.1199,0.000027, 2.7}; // result from twiddle
  double total_cte = 0.0;
  int tw =0; // condition of last twiddle condition
  int i_PID=0; // 0 (P), 1(I) , 2(D) 
  int steps_per_it;
  int it=0;
  int n_step=0;
  int n_settle=100;
  int reach_speed=0;
  //int i_step; // to find out how many step in a loop
  int i_twiddle; // 1 twiddle, 0 - run
  double p_input;
  double i_input;
  double d_input;
  ofstream f;
  f.open("../output.txt");
  f<<"cte, speed, angle, pid.Kp, pid.Ki,pid.Kd,p_error,i_error,d_error,steer_value  "<<endl;


  //cout<<"Please input  '0' : run with Optimaized PID   '1': twiddle   '2' : input PID for manula trial"<<endl;
  //cin>>i_twiddle;
  i_twiddle=0;
  if (i_twiddle==0){
    pid.Init(best_p[0],best_p[1],best_p[2],dp[0],dp[1],dp[2]);
    cout<<"running with PID (" <<best_p[0]<<","<<best_p[1]<<","<<best_p[2]<<" )"<< " and see how it works!"<< endl;
  }
  else if(i_twiddle==1){

    pid.Init(p[0],p[1],p[2],dp[0],dp[1],dp[2]);
    cout<<"input how many steps (msg between c++ and simulator) to per each twiddle iteration (1850 steps per loop on the track)"<<endl;
    cin>>steps_per_it;
    cout<<"start twiddle from PID (" <<p[0]<<","<<p[1]<<","<<p[2]<<" )"<< " and "<<steps_per_it<<" steps per each iteration!"<< endl;
    cout<<"twiddle will start after speed reach 30MPH"<<endl;

   }else if(i_twiddle==2){
    cout <<"running  with  fixed PID, please input P="<<endl;
    cin>>p_input;
    cout<<"please input I="<< endl;
    cin>>i_input;
    cout<<"Please input D="<<endl;
    cin>>d_input;
    pid.Init(p_input,i_input,d_input,dp[0],dp[1],dp[2]);
    cout<<"Your input is P="<<p_input<<" I="<<i_input<<" D="<<d_input<<"... manual trial begin and good luck!"<< endl;
    f<<pid.getKp()<<","<<pid.getKi()<<","<<pid.getKd()<<","<<pid.get_p_error()<<","<<pid.get_i_error()<<","<<pid.get_d_error()<<endl;
  }



  h.onMessage([&tw, &i_PID, &it, &n_settle, &n_step,&steps_per_it, &reach_speed, &f, &pid, &p, &dp,  &tol,  &total_cte, &i_twiddle, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double throttle_value = 0.3;

          json msgJson;

          if (i_twiddle ==1){

            if(reach_speed==0){
              if (speed>30 ){
              reach_speed=1;
              cout<<"twiddle start now.."<<endl;            // twiddle start after speed reach 30;
              }
            }

            if (reach_speed==1){
              if (n_step>n_settle) { total_cte+=pow(cte,2);}

              if ((dp[0]+dp[1]+dp[2])>tol) {

                if (n_step==steps_per_it+n_settle) { //finish one iteration

                  tw=pid.twiddle(tw,total_cte);
                  p[0]=pid.getKp();
                  p[1]=pid.getKi();
                  p[2]=pid.getKd();
                  dp[0]=pid.getdp();
                  dp[1]=pid.getdi();
                  dp[2]=pid.getdd();
                  it+=1;

                  cout <<"iternation="<<it<<" n_step="<<n_step<<" PID= ("<<p[0]<<","<<p[1]<<","<<p[2]<<" ) dp= (";
                  cout <<dp[0]<<","<<dp[1]<<","<<dp[2]<<" ) ";
                  total_cte=0;
                  n_step=0;

                }
              }
              else {
                cout<<"Twiddle finished and the best PID is ("<<p[0]<<","<<p[1]<<","<<p[2]<<" )"<<" Please record it" <<endl;
              }
              //cout<<"iteration="<<it<<" n_step"<<n_step<<" total_cte="<<total_cte<<" speed="<<speed<<endl;
              n_step+=1;
            }
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

           //if (steer_value>0.4) {
            //  steer_value=0.4;
            // }
            //else if (steer_value<-0.4){
            // steer_value=-0.4;
            //}

            f<<cte<<","<<speed<<","<<angle<<","<<pid.getKp()<<","<<pid.getKi()<<","<<pid.getKd()<<","<<pid.get_p_error()<<","<<pid.get_i_error()<<","<<pid.get_d_error()<<","<<steer_value<<endl;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          } else if (i_twiddle==0 || i_twiddle==2) {
            pid.UpdateError(cte);
            //i_step+=1;  to find out how many steps per loop
            //cout<<"i_step="<<i_step;
            steer_value = pid.TotalError();
            f<<cte<<","<<speed<<","<<angle<<","<<pid.getKp()<<","<<pid.getKi()<<","<<pid.getKd()<<","<<pid.get_p_error()<<","<<pid.get_i_error()<<","<<pid.get_d_error()<<","<<steer_value<<endl;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            cout <<" i_twiddle="<<i_twiddle<<" cte=" << cte << " Steering Value=" << steer_value;
            cout<<","<<pid.getKp()<<","<<pid.getKi()<<","<<pid.getKd()<<","<<pid.get_p_error()<<","<<pid.get_i_error()<<","<<pid.get_d_error();
            cout<<" speed="<<speed<<" angle="<<angle<<endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
  f.close();
}
              

