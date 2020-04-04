#  **PID Controller** 
## Udacity Self-Driving Car Nanodegree Project 8


**The goals / steps of this project are the following:**

* xxx

[//]: # (Image References)

[image1]: ./images/way_points.png "way points"
[image2]: ./images/result.png "result"


###  Environment set up

The environment setting is the exactly same as previous three projects , I only need do the followings: 

* Clone project from Udacity's repo on Github
```sh
git clone https://github.com/udacity/CarND-PID-Control-Project
```

* Edit the file of "main.cpp" and compile/build the excutalbe file to pass the simulator.

* Push project to my repo on Github and submit the project
```sh
git push https://github.com/guishengwang/Udacity-P8-PID-Controller
```


### Submission

File Name | Description
----------|-----------
main.cpp  | get data from simulator, plan the next path and send data to simulator
README.md | summary of the project

 ![alt text][image2]

###  Highway map and way points

The way point data is from the file of “highway_map.csv” read into variables as below, 

No. | map_waypoints_x | map_waypoints_y | map_waypoints_s | map_waypoints_dx | map_waypoints_dy
----------|-----------|-----------|-----------|-----------|-----------
1 | cte | 1135.571 | 0 | -0.02359831 | -0.9997216
2 | 815.2679 | 1134.93 | 30.6744785308838 | -0.01099479 | -0.9999396
3 | 844.6398 | 1134.911 | 60.0463714599609 | -0.002048373 | -0.9999979
… |… |… |… | … | …
180 | 711.2 | 1143.5 | 6871.54959487915 | -0.2637061 | -0.9646032
181 | 753.2067 | 1136.417 | 6914.14925765991 | -0.107399 | -0.9942161


The waypoints are in the middle of the double-yellow dividing line.

 ![alt text][image1]


Track is 6945.554 meters (4.32 mile) long  / 181 way points , around 38 meters apart   

Speed limit is 50 MPH  or 22.35 m/s,  it will take 310.7 second ( 5.18 minutes ) to finish one loop

Each Lane 4 meters wide / 3 lanes on each direction



### Data from simulator


variable   | Description 
----------|-----------
cte| Cross Track Error
speed| vehicle speed,  MPH
angle| vehicle steering angle

### Steering angle bais to right side.

Steer_value   | angle showed on simulator | angle from simulator to main.cpp 
----------|-----------|-----------
0 | 0.44 |0.4363
0.2|0.44 |5.4363
-0.2|-4.56|-4.5637
1.0| 25 | 25
1.1| 25 | 25

The steering angle of the simulator vehicle has a bias to the right side with 0.4363 degree and the maximum steering degree is +-25.


### Special note on unit of speed from simulator 

The simulator provides both the ego car speed and other car's speed but these information are in differnt unit as below, 

variable   | Description | Unit
----------|-----------|-----------
car_speed| the speed of ego car| MPH
sensor_fusion[3]| vx ,velocity components | m/s
sensor_fusion[4]| vx ,velocity components | m/s

When the code gets the speed of other cars as below
```sh
    double other_vx=sensor_fusion[i][3];
    double other_vy=sensor_fusion[i][4];
    double other_speed=sqrt(other_vx*other_vx+other_vy*other_vy);
```
The speed of other car will be in the unit of m/s, a conversion of the ego car speed to the same unit is necessary before being compared with other cars. This is very important step, at the begining, even after I have set up flag for safely changing lane after comparing speed of ego car and other car around, collisons still happened. Finally I realized the difference of the unit of speed between ego car and other ars and fixed the problem.


### Strategy to prevent collisons

The walk through video had demonstrated how to reduce speed of ego car if there was a car ahead in 30 meters to prevent collison. The chance of collison druing lane changing also need to be eliminated. The code will collect the speed and location of other cars around ego car and make desicion if it is safe to change lane or not. 

At the same time, I do not want the ego car to reduce the speed too much which might cause collison with fast car behind and/or lost the chance to over take other cars in other lane. So I upgraded the orginal code from walk through to make the ego car to adjust speed not only based on distance but also the speed difference with other cars.  

### Strategy to change lane and over take other cars

According to the walk through, the car will brake or reduce the speed when the distance to the car ahead is less than 30 meters while our target is to keep the ego car running as fast as the speed limit, so it'd better make the ego car running in a lane with less car ahead of it. The code will detect the distance between the car ahead and the ego car, if there is no car in 100 meters ahead in other lane but a car in 50 meters ahead in the same lane, the ego car will change lane. This strategy works quite well, the ego car will change lane in advance and do not wait until it is too close to the car ahead. 



## Discussion

This is a very interesting project but at the beginning, I even didnot know where to start. My mentor suggested me to start with the walk through video which later I found out was extremely helpful. It was so exited to see the ego car to start moving, braking, chaning lane in the simultor. To make ego car to maneuver among other cars took me lots of thoughts and the class from Sebastian on path planning gave me a clue and I created a matrix for recording cars around the ego car within +150 and -30 meter range and also their speed. Apparently this method works quite well and my ego car could run though the highway to meet the requirement. There was lots of fun!!

Many thanks to the Udacity team and my mentor. Looking forward to the next challenge!




