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
  bool twiddle = false;
  int n = 0;
  int max_n = 600;
  double error = 0.0;
  int p_iterator = 0;
  int total_iterator = 0;
  int sub_move = 0;
  bool first = true;
  bool second = true;
 // double best_p[3] = {p[0],p[1],p[2]};


  uWS::Hub h;
  PID pid;
  double p[3] = {0.15, 0.0005, 3}; //twiddle start point
  double dp[3] = {0.00005,0.00005, 0.00005};
  double tol = 0.000015;
  double best_p[3] = {0.13,0.00027,3.05}; // result from twiddle
  double total_cte = 0.0;
  int tw =0; // condition of last twiddle condition
  int i_switch=0;
  int i_PID=0; // 0 (P), 1(I) , 2(D) 
  int steps_per_it;
  int it=0;
  int n_step;
  //int i_step; // to find out how many step in a loop
  int i_twiddle; // 1 twiddle, 0 - run
  double p_input;
  double i_input;
  double d_input;
  ofstream f;
  f.open("../output.txt");
  f<<"cte, pid.Kp, pid.Ki,pid.Kd,p_error,i_error,d_error  "<<endl;


  cout<<"Please input '0'-Optimaized PID() '1'-twiddle or '2'-manual input PID"<<endl;
  cin>>i_twiddle;
  if (i_twiddle==0){
    pid.Init(best_p[0],best_p[1],best_p[2],dp[0],dp[1],dp[2]);
    cout<<"Optimaized PID from twiddle is (" <<best_p[0]<<","<<best_p[1]<<","<<best_p[2]<<" )"<< " and see how it works!"<< endl;
  }
  else if(i_twiddle==1){

    pid.Init(p[0],p[1],p[2],dp[0],dp[1],dp[2]);
    cout<<"input how many steps (msg between c++ and simulator) to per each twiddle iteration (1850 steps per loop on the track)"<<endl;
    cin>>steps_per_it;
    cout<<"start twiddle from PID (" <<p[0]<<","<<p[1]<<","<<p[2]<<" )"<< " and "<<steps_per_it<<" steps per each iteration!"<< endl;

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

  h.onMessage([&tw, &i_switch,&i_PID,&it, &n_step,&steps_per_it,&f, &pid, &p, &dp, &n, &max_n, &tol, &error, &p_iterator, &total_iterator, &total_cte, &first, &sub_move, &second, &twiddle, &i_twiddle, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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

            // twiddle magic 
            if ((dp[0]+dp[1]+dp[2])>tol) {

              if (n_step==steps_per_it) { //finish one iteration

                if (it==0) { // finish the first iteration, it=0
                  best_error=total_cte;
                  f<<"finished 1st Iteration and the set best_err to total_cte="<<total_cte<<endl;
                }
                else {  // from second iteration, it>=1
                    tw=pid.twiddle(tw,total_cte);
                    cout<<"called pid.twiddle()"<<endl;
                    p[0]=pid.getKp();
                    p[1]=pid.getKi();
                    p[2]=pid.getKd();
                    dp[0]=pid.getdp();
                    dp[1]=pid.getdi();
                    dp[2]=pid.getdd();
                }
                it+=1;
                total_cte=0;
                n_step=0;
              }
              cout <<"twiddling...  iternation="<<it<<" PID= ("<<p[0]<<","<<p[1]<<","<<p[2]<<" ) dp= (";
              cout <<dp[0]<<","<<dp[1]<<","<<dp[2]<<" )"<<endl;
            }
            else {
              cout<<"Twiddle finished and the best PID is ("<<p[0]<<","<<p[1]<<","<<p[2]<<" )"<<" Please record it" <<endl;
            }

            n_step+=1;
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            if (steer_value>0.4) {
              steer_value=0.4;
            }
            else if (steer_value<-0.4){
              steer_value=-0.4;
            }
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          } else if (i_twiddle==0 || i_twiddle==2) {
            pid.UpdateError(cte);
            //i_step+=1;  to find out how many steps per loop
            //cout<<"i_step="<<i_step;
            f<<cte<<","<<pid.getKp()<<","<<pid.getKi()<<","<<pid.getKd()<<","<<pid.get_p_error()<<","<<pid.get_i_error()<<","<<pid.get_d_error()<<endl;
            steer_value = pid.TotalError();
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            cout <<" i_twiddle="<<i_twiddle<<" cte=" << cte << " Steering Value=" << steer_value;
            cout<<" speed="<<speed<<" angle="<<angle<<"  "<< msg <<endl;
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
              

