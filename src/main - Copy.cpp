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
  double best_error = 10000.00;
  double tol = 0.001;
  int p_iterator = 0;
  int total_iterator = 0;
  int sub_move = 0;
  bool first = true;
  bool second = true;
 // double best_p[3] = {p[0],p[1],p[2]};


  uWS::Hub h;
  PID pid;
  double p[3] = {0.15, 0.0005, 3};
  double dp[3] = {.005,0.00005, 0.1};
  double best_p[3] = {p[0],p[1],p[2]};
  double total_cte = 0.0;
  int steps_per_it;
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
    pid.Init(0.13,0.00027,3.05);
    cout<<"Optimaize PID Is (0.13,0.00027,3.05) and see how it works!"<< endl;
  }
  else if(i_twiddle==1){
    pid.Init(p[0],p[1],p[2]);
    cout<<"please input how many steps to accumulate the total error (1850 steps per loop on the track)"<<endl;
    cin>>steps_per_it;
    cout<<"start twiddle..."<<endl;
  }else if(i_twiddle==2){
    cout <<"running  with  fixed PID, please input P="<<endl;
    cin>>p_input;
    cout<<"please input I="<< endl;
    cin>>i_input;
    cout<<"Please input D="<<endl;
    cin>>d_input;
    pid.Init(p_input,i_input,d_input);
    cout<<"Your input is P="<<p_input<<" I="<<i_input<<" D="<<d_input<<"... manual trial begin and good luck!"<< endl;
    f<<pid.getKp()<<","<<pid.getKi()<<","<<pid.getKd()<<","<<pid.get_p_error()<<","<<pid.get_i_error()<<","<<pid.get_d_error()<<endl;

  }

  h.onMessage([&n_step,&steps_per_it,&f, &pid, &p, &dp, &n, &max_n, &tol, &error, &best_error, &p_iterator, &total_iterator, &total_cte, &first, &sub_move, &second, &twiddle, &i_twiddle, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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


            if (n_step==steps_per_it) { //finish one iteration
              total_cte = 0;
            }
            else{
              total_cte = total_cte + pow(cte,2);
            }

            //Steering value
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            cout <<"i_twiddle="<<i_twiddle<<" cte=" << cte << " Steering Value=" << steer_value<<" "<< msg <<endl;

            cout <<"i_twiddle="<<i_twiddle<<" cte=" << cte << " Steering Value=" << steer_value;
            cout<<" speed="<<speed<<" angle="<<angle<<"  "<< msg <<endl;


/*            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
            n = n+1;
            if (n > max_n){ 
              
              
              //double sump = p[0]+p[1]+p[2];
              //std::cout << "sump: " << sump << " ";
              if(first == true) {
                cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                p[p_iterator] += dp[p_iterator];
                //pid.Init(p[0], p[1], p[2]);
                first = false;
              }else{
                error = total_cte/max_n;
                
                if(error < best_error && second == true) {
                    best_error = error;
                    best_p[0] = p[0];
                    best_p[1] = p[1];
                    best_p[2] = p[2];
                    dp[p_iterator] *= 1.1;
                    sub_move += 1;
                    cout << "iteration: " << total_iterator << " ";
                    cout << "p_iterator: " << p_iterator << " ";
                    cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    cout << "error: " << error << " ";
                    cout << "best_error: " << best_error << " ";
                    cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
                }else{
                  //std::cout << "else: ";
                  if(second == true) {
                    cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    p[p_iterator] -= 2 * dp[p_iterator];
                    //pid.Init(p[0], p[1], p[2]);
                    second = false;
                  }else {
                    cout << "iteration: " << total_iterator << " ";
                    cout << "p_iterator: " << p_iterator << " ";
                    cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    if(error < best_error) {
                        best_error = error;
                        best_p[0] = p[0];
                        best_p[1] = p[1];
                        best_p[2] = p[2];
                        dp[p_iterator] *= 1.1;
                        sub_move += 1;
                    }else {
                        p[p_iterator] += dp[p_iterator];
                        dp[p_iterator] *= 0.9;
                        sub_move += 1;
                    }
                    cout << "error: " << error << " ";
                    cout << "best_error: " << best_error << " ";
                    cout <<"Best p[0] p[1] p[2]:" << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
                  }
                }
                
              }
              

              if(sub_move > 0) {
                p_iterator = p_iterator+1;
                first = true;
                second = true;
                sub_move = 0;
              }
              if(p_iterator == 3) {
                p_iterator = 0;
              }
              total_cte = 0.0;
              n = 0;
              total_iterator = total_iterator+1;

              double sumdp = dp[0]+dp[1]+dp[2];
              if(sumdp < tol) {
                //pid.Init(p[0], p[1], p[2]);
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
                //ws.close();
                //std::cout << "Disconnected" << std::endl;
              } else {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                cout <<"twiddle="<<twiddle<<" cte=" << cte << " Steering Value=" << steer_value<<" "<< reset_msg <<endl;
              }

            } else {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              cout <<"twiddle="<<twiddle<<" cte=" << cte << " Steering Value=" << steer_value<<" "<< msg <<endl;

            }
*/           
          } else if (i_twiddle==0 || i_twiddle==2) { //i_twiddle ==0
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
