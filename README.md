#  **PID Controller** 
## Udacity Self-Driving Car Nanodegree Project 8


**The goals / steps of this project are the following:**

* By using PID controller to keep the car running in the track 
* To optimize the PID paramaters by the magic of twiddle

[//]: # (Image References)

[image1]: ./images/way_points.png "way points"
[image2]: ./images/offset.png "offset"
[image3]: ./images/start.png "start"

###  Environment set up

The environment setting is the exactly same as previous three projects , I only need do the followings: 

* Clone project from Udacity's repo on Github
```sh
git clone https://github.com/udacity/CarND-PID-Control-Project
```

* Edit *.cpp and *.h files and compile/build the excutalbe file to pass the simulator.

* Push project to my repo on Github and submit the project
```sh
git push https://github.com/guishengwang/Udacity-P8-PID-Controller
```


### Submission

File Name | Description
----------|-----------
main.cpp  | get data from simulator, calculate the steering angle and send data to simulator, play the magic of twiddle
PID.cpp| Completed the funciton of Init() and update the error
PID.h| Completed the funciton of Init() and update the error
README.md | summary of the project


### Data between simulator and C++ code

The simulator sends following information to main() functions.

variable   | Description 
----------|-----------
cte| Cross Track Error
speed| vehicle speed,  MPH
angle| vehicle steering angle

The main() fucntion sends following data to the simulator

variable   | Description 
----------|-----------
steering_angle| Steering angle calucated from PID controller
throttle| opening of throttle


### Steering angle bais to right side.

The steering angle of the simulator vehicle has a bias to the right side with 0.4363 degree.

Steer_value   | angle showed on simulator | angle from simulator to main.cpp 
----------|-----------|-----------
0 | 0.44 |0.4363
0.2|0.44 |5.4363
-0.2|-4.56|-4.5637
1.0| 25 | 25, maximum steering angle
1.1| 25 | 25, maximum steering angle

Below is a screenshot if I set the steering angle to zero but the similutor will still show a anlge of 0.44 degree.
 
![alt text][image2]


### PID Class

Following functions was completed to initialize parameters and update the error,

* void PID::Init(double Kp_, double Ki_, double Kd_)
* void PID::UpdateError(double cte)
* double PID::TotalError() 

I also added some simple functions for the purpose of debug

* double PID::getKp()
* double PID::getKi() 
* double PID::getKd()
* double PID::get_p_error()
* double PID::get_i_error() 
* double PID::get_d_error() 

### Starting point of parameters

Before runing the magic of twiddle, I need to find out a starting point of the parameters to as least keep the car running inside the track. I updated the coded so the parameter could be input on the terminal instead of to compile/build the file for each trial, that made it very convenient for me. After many trial and I select the start point as ( 0.15, 0.0005, 3) and twiddle from there.

With this set of parameter, the car can finish the whole loop and but sometime the car will swing from left to right as screenshot shown as below.

![alt text][image3]



### The magic of twiddle



## Discussion

At the begining I had some trouble with the simulator/C++ program, the car seems not to follow the steering angle I send to the similator but keep climbing up the hill. Then I realized that the car's steering angle has a bais of 0.44 degree to the right side and I fixed the Init() of PID, after that, the car started to listen my orders!

Many thanks to the Udacity team. Looking forward to the next challenge!




