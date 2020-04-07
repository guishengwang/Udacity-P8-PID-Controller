#  **PID Controller** 
## Udacity Self-Driving Car Nanodegree Project 8


**The goals / steps of this project are the following:**

* By using PID controller to keep the car running in the track 
* To optimize the PID paramaters by the magic of twiddle

[//]: # (Image References)

[image1]: ./images/way_points.png "way points"
[image2]: ./images/offset.PNG "offset"
[image3]: ./images/start.png "start"

###  Environment set up

The environment setting is exactly the same as previous projects , I only need do the followings: 

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
main.cpp  | get data from simulator, calculate the steering angle and send data to simulator
PID.cpp| initialize the parameter, update error and add the twiddle funciton  
PID.h|  header file 
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



### Components of a PID controller

There are 3 parameters in the PID controller : 

The P (proportional), component will make the car steer proportional ( in a negative way ) to the cross track error (CTE, the distance to the center of the lane) and it is the most influcical component.  Set P to zero, the car will continue drive away from the center, as shown in the video.  [[P=0 Video]](https://youtu.be/RVtpNZFbCN0 "PID controller P=0")

The D (differential) corrects present error versus previous error. It counteracts the overshoot caused by P component. To set the D to zero, the car will swing from left to right or oscillate around the center line and finally out of the track, as shown in the video.[[D=0 Video]](https://youtu.be/6xqpiBRzttA "PID controller D=0") 

The I (integral) is to compensate the bias in the system ( the car has a bias of -0.44 degree to the right side). To set I to zero, the car will not travel out of the track as quickly as when the P or D was set to zero, but since without the compensation to the system bias, the car will drive out of the right side of the track finally as shown in the video.[[I=0 Video]](https://youtu.be/-uQAmcpbc-o "PID controller I=0") 


### Starting point of parameters

Before runing the magic of twiddle, I need to find out a starting point of parameter set to keep the car running inside the track. After many manual trial and based on the suggestion from Udacity knowledge base, I selected the start point as ( 0.15, 0.0005, 3) and twiddle from there.

With this set of parameter, the car can finish the whole loop and but sometime the car will swing from left to right as screenshot shown as below.

![alt text][image3]


### PID Class and The magic of twiddle

Following functions was completed to initialize parameters, update cross track error:

* void PID::Init(double Kp_, double Ki_, double Kd_)
* void PID::UpdateError(double cte)
* double PID::TotalError() 

I also added a function of twiddle

* int PID::twiddle(int tw ,double total_cte)

The demo python sample for the twiddle function will call RUN fucntion of robot, while in this project, the car is always running in the simulator and the main () will call the twiddle funciton in the PID class. I was struggling at the begining with how to implement the twiddle fuction in this senario. Finaly, I figured out that I can use the return value of the twiddle function to keep the record the logic of the previous twiddle loop and send the return value back to the twiddle function so it can continue it logic. 

```sh
tw=pid.twiddle(tw,total_cte)
```


## Discussion

At the begining I had some trouble with the simulator/C++ program, the car seems not to follow the steering angle I send to the similator but keep climbing up the hill. Then I realized there was a bug in my Init() function.

The next difficulty is to implement the twiddle funciton which was taking me lots of time and effort and finally I figured it out as I explained in previous section. Then during the twiddle process, sometimes the car went out of track and was not able to come back again. It was another difficulty for me. While later I realized that I do not have to restart the twiddle process from original start point of the parameter set, instead, I chose the parameter which got the best error and continue the twiddle from this new set of paramter. 

This is another interesting project! Many thanks to the Udacity team and looking forward to the next challenge!




