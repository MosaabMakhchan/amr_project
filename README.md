# amr_project
Implementation of a Autonomous mobile robot using Ros2

### Abstract
The aim of this project is to develop an autonomous navigation mobile robot (AMR) using Robot Operating System (ROS2), starting from building and controlling the robot intro developing an odometry motion model using sensor fusion algorithms using the {Extended Kalman Filter}, then exploring mapping and localization techniques and algorithms such as {Markov's Localization}, {Monte Carlo Localization},
    and finally Simultaneous Localization and Mapping -SLAM-.
# ROS2-Based Autonomous Mobile Robot (AMR)

```
M.Makhchan
National School of Applied Sciences, University of Abdelmalik Essaadi, Tangier, Morocco
(Dated: July 8, 2025)
```
```
The aim of this paper is to develop an autonomous navigation mobile robot (AMR) using Robot
Operating System (ROS2), starting from building and controlling the robot intro developing an
odometry motion model using sensor fusion algorithms using theExtended Kalman Filter, then
exploring mapping and localization techniques and algorithms such asMarkov’s Localization,Monte
Carlo Localization, and finally Simultaneous Localization and Mapping -SLAM-.
```
```
I. INTRODUCTION
```
The goal of this paper is to develop an autonomous mo-
bile robot (AMR) using the Robotic Operating System
ROS2. In the First section, we will provide an overview
of ROS2 and its architecture, explaining its main commu-
nication tools and techniqus as well as its backbone,Data
Distribution Service, In the second section,we will dis-
cuss the main steps and considerations in building the
robot, visualizing it in Gazebo simulation and the key
differences between controlling the robot in the simula-
tion and in the real world, then we will try to estimate
the position of the robot using wheel encoders in refer-
ence to a fixed odometry frame, exploring the effects of
noise on our estimations. In the Third Section, we will
try to reduce the effect of noise and try to reduce the
gap between the simulation and reality, by using multi-
ple sensors to provide a more accurate estimate and using
sensor fusion techniques such as Kalman Filter and the
Extended Kalman Filter.
In the Following section, we will develop an odometry
motion model proposed by the authors ofProbabilis-
tic Roboticsbook,which takes intro cosideration the in-
evitable effects of noise and the uncertainty that com-
panies it, thus we will take a probabilistic approach in
estimating all the robot’s possible positions,
In the last three sections, we will explore different ap-
proaches towards mapping and localization, from build-
ing a map with known poses and localizing with a known
map using Markov localization and Monte Carlo Local-
ization towards taking a much more robust approach us-
ing Simultaneous Localization and mapping

```
II. ROS2 ARCHITECTURE
```
Although the name suggest that ROS2 is an operating
system, as a matter of fact, it is a set of open source
algorithms, hardware driver software and tools developed
to develop robot control software, it is

- Framework & Tools (Build system & dependency
    management, Visualization, Record and Replay)
- Communication System (Publish Subscribe and
    Remote Method Invocation)
       - Ecosystem (Language bindings, Drivers, libraries
          and simulation (Gazebo))

```
ROS2 uses nodes and topics as the main form of com-
munication, a node can subscribe or publish into a topic,
topics act as bus for nodes to exchange information, if a
node is subscribing into a topic it can access the messages
published their by other nodes, as illustrated in FIG.1.
```
```
FIG. 1: Nodes and Topics illustration
Rectangles: topics, Circles: nodes
```
```
ROS2 uses other forms of communication such as Ac-
tions,services and servers but explaining them it out of
the scope of this paper, on the other hand, the back-
bone of ROS2 is considered to be DDS -Data Distribu-
tion Service-, it uses DDS as its middleware between the
client libraries which the developer interacts with and the
operating system it runs on it.
DDS have proven to reduces coupling, increases scal-
ability, and improves performance, reliability, security,
and flexibility. It is for this reason why it is common to
see the DDS communication protocol used in the defense
industry. In the last Distributions of ROS2, Fast-DDS
comes as standard but can be changes with other DDS
solutions. Instead of interacting with the DDS directly
ROS2 adds it’s own layer of abstraction, ROS Middle-
ware Interface -rmw-, which a library written in C/C++.
above it the client libraries can be build in different pro-
gramming languages such asrcpy for python,rcpp for
C++ and recently for RUST enthusiastsrclrusthas been
introduced.
```

```
FIG. 2: ROS2 Architecture
```
```
III. BUILDING AND CONTROLLING THE
ROBOT
```
After Generating a URDF - Unified Robot Desciption
Format-, describing the robot we want to simulate, its
links , collision , joints and mass and dimensions, or up-
loading a mesh file from a CAD software, preparing it
for simulation inGazebo, An important step comes ahead
which is controlling the robot for this purpose ROS2 com-
munity offers a very powerful framework to control both
the real and the simulated robot and the reduce the gap
between them calledros2-control.
At the heart ofros2-controlframework there is hard-
ware components that can either be real hardware com-
ponents such as sensors or actuators or simulated ones
in physics simulation engines such asGazebo, the re-
source manager is responsible for loading the hardware
components, and providing two interfaces between it and
the hardware components, Command interfaces to send
commands to the components and state interfaces to get
the data from the components. The resource manager
is linked to the controller manager which is an inter-
face between all the existing controllers for our hardware
components.(FIG.3) This architecture is very simple yet
powerful and allows for an easy transition from simula-
tion into real robot and vice versa since the only compo-
nents that needs to change is the hardware component
provided by the resource manager. For our project, we
are interested in differential drive control where only two
wheels,each linked to a separate motor, are responsible
for all the movements of the robot. Now that we can
control the robot and move it freely, the next step would
be to know the position of the robot,for that we are go-
ing to determine the position of the robot in reference to
where the origin of the movement and to a fixed origin
of the map, in other words, the position of the frameFR

```
that is linked to the robot in reference to the frameFO
that represents the origin where the robot has started
moving and in reference to the fixed origin of the map
FM.. For that purpose we are going to use thetf2li-
```
```
FIG. 3: ROS2 Control architecture
```
```
brary. FIG.4, is an illustration of the frames and the
links between them. Describing the position of a frame
in reference to an other frame in the 3-D world,breaks
down to calculating the rotations and translations that
```
```
FIG. 4: Enter Caption
```

occurred in time,the rotation matrix over the three axis
are given by:

```
Rx(θ) =
```
## 

## 

## 1 0 0

```
0 cosθ −sinθ
0 sinθ cosθ
```
## 

## 

```
Ry(θ) =
```
## 

## 

```
cosθ 0 sinθ
0 1 0
−sinθ 0 cosθ
```
## 

## 

```
Rz(θ) =
```
## 

## 

```
cosθ −sinθ 0
sinθ cosθ 0
0 0 1
```
## 

## 

```
R(θ) =
```
## 

## 

```
r 1 r 2 r 3
r 4 r 5 r 6
r 7 r 8 r 9
```
## 

## 

While the rotation matrixR(θ) is the product ofRx,Ry
andRz. this representation is called Euler Angles repre-
sentation, the draw back of this representation is when
we want to make successive rotations,since it is repre-
sented as a matrix it is computationally expensive since
it comes down to matrix multiplication or in other words,
28 multiplications and 18 sums for each multiplication.
For this reason in practice we use a different angle rep-
resentation calledQuaternion, unlike Euler representa-
tion which needs 9 elements r1-r9, quaternion only needs
4 elements a,b,c,d as follows:

```
Re
```
```
Im
```
```
0 a
```
```
b
```
```
a+bi
```
```
q=a+bi+cj+dk (1)
```
The Quaternion uses the imaginary number i for its
q−^1 =a−bi−cj−dkand it has the following prop-
erties:

- a^2 +b^2 +c^2 +d^2 = 1
- q−^1 =a−bi−cj−dk

with the Quaternion angles representation the cost of the
computations as the robot moves through the space is
optimized which allows for a reduced latency and higher
response time.
Wheel Odometry & Wheel Encoders:
Previously we calculated the transformation matrix be-
tween the origin of the movement and the robot,however,

```
we didn’t go into details of what measurement system we
used to determine the distance and the orientation of the
robot, the most basic system that every mobile robot
should be equipped with is Wheel Encoders, by using
wheel encoders we can extract the number of rotations
of each wheel, and based on this information we can esti-
mate can calculate the transformation matrix, and thus
the position of the robot. We call this system of defining
the robot position based on the wheel encoders :Wheel
Odometry System.
Our calculations so far don’t take into account the noise
that will affect the wheel encoders, in the real world the
noise will for sure affect our odometry system and thus
our estimations of the robot position will not be entirely
correct, for that reason we are going to add the informa-
tion coming from the wheel encoders a Gaussian noise
(indicated by FIG.5) - with a variance of 0.05 an a mean
of 0 -,in order to better estimate the robot position, while
the free-noise wheel encoders are pointing to 0 - the robot
is not moving along the x axis, the noisy signal is indicat-
ing little movements, and when the robot is in motion, we
can still observe the difference between the noisy signal
and the noise free signal as indicated by FIG.6, FIG.
demonstrates how the odom frame (which represented
the origin of the movement) is linked both to a base-link-
fooprint which is a frame linked to the origin of the robot
without taking noise intro consideration and to a base-
link-footrpint-noisy which takes intro consideration the
Gaussian Noise we injected,
```
```
FIG. 5: Noise and Noise free while robot is stopped
```
```
FIG. 6: Noise and Noise-free signals while the robot in
motion
```

FIG. 7: Noisy Frame vs Noise Free frame comparison in
rviz2 at t=1min

As we move the robot the expected result is that the
position of the robot indicated by the base-footprint-
noisy frame should be different from the real position
which is indicated by the the base-footrpint, in ROS
environment we can userviz2 to visualize the frames
published by the tf2 library, FIG.7 represent the results
obtained after 1min of movement.

```
IV. SENSOR FUSION
```
As we observed in the previous section,while the wheel
odometry system provides an estimate of the robot posi-
tion, the noise effect makes our estimations diverge over
time, so in order to get a better estimate of the robots
position and velocity, we are going to not only rely on
the wheel encoders but to use multiple sensors such as
Lidar and IMU and later a Camera to develop a Visual
Odometry system using Computer Vision, for that pur-
pose we suggest using theKalman Filter Algorithmto
fuse the output of this different sensors.

```
A. Kalman Filter
```
This algorithm belongs to the Bayesian filter family,
which introduces a way to update the probability of a
certain even (e.g:position of the robot ) based on new
sensors and observations,it is one of the most commonly
used algorithms for the sensor fusion of data coming from
different sensors and filtering the noise affecting them,the
Kalman filter assumes that the noise due to the robot
motion and the one due to the sensor measurement is a
Gaussian noise.
The basic idea of the Kalman Filter is to use a model
of the system being measured, and to update the model
as new measurements become available, the filter works
by making a prediction of the current state of the sys-
tem based on the previous state estimate and the system
model, and then combining this prediction with a new

```
measurement to obtain an updated state estimate.
```
```
FIG. 8: Illustration of Kalman Filter Steps
```
```
Figure 8 is an illustration of this algorithm, (a) rep-
resents the initial belief (e.g: the initial position), (b) a
measurement from a sensor that indicates a different po-
sition (in bold) with the associated uncertainty, (c) rep-
resents the new belief after integrating the measurement
into the belief using the Kalman filter algorithm which is
also a guassian distribution with a variance smaller the
the previous two gaussians, in other worlds the certainty
is higher, (d) is the belief (position) after motion to the
right (which introduces uncertainty since the variance is
now larger), (e) a new measurement with associated un-
certainty, and (f) the resulting belief.
```
```
B. Extended Kalman Filter
```
```
One of the main drawbacks of the Kalman Filter algo-
rithm is the fact that it is based on two main assump-
tions:
```
- The first assumption the motion model of the robot
    and measurement model of the sensors are both
    linear models
- The second assumption being that the probability
    ditsribution of the state estimation process is Gaus-
    sian and so that the filter only needs the mean and
    the variance of this Gaussian distribution.

```
The assumptions of linear state transitions and linear
measurements with added Gaussian noise are rarely ful-
filled in practice. For example, a robot that moves
with constant translation and rotational velocity typi-
cally moves on a circular trajectory, which cannot be
```

described by linear next state transitions. This obser-
vation, along with the assumption of unimodal beliefs,
renders plain Kalman filters, as discussed so far, inap-
plicable to all but the most trivial robotics problems.
However in the real world, this two assumptions are of-
ten not valid,for this reason the Extended Kalman Filter
-EKF- Has been introduced, since it takes into account-
ability non-linearity’s in the motion and measurement
model. The extended Kalman filter (EKF) overcomes
one of these assumptions: the linearity assumption. Here
the assumption is that the next state probability and
the measurement probabilities are governed by nonlinear
functions g and h, respectively:

```
xt=g(u(t),x(t−1)) +t (2)
```
```
zt=h(xt) +δt (3)
```
EKF adds an intermediate step in the standard Kalman
Filter algorithm called Linearization, which consists of
approximating a point in the non linear model curve with
a linear function. Thanks to the Extended Kalman fil-
ter we can now take into account the non linear models
in sensor fusion to get an even better estimate of robot
position.

```
V. ODOMETRY SYSTEMS’ ERROR AND THE
NEED FOR A MOTION MODEL
```
```
A. Odometry Systems’ error
```
Regardless of the sensors we are using,their precision,
their resolution or how sophisticated the sensor fusion
algorithm we implemented for the odometer calculation,
it will always be affected by intrinsic and extrinsic errors,
that tend to diverge (increase over time) and the more
the robot moves the greater the error becomes let’s give
an example:

Wheel Odometry:Let’s take the example and start by
considering the wheel odometry, although it is a
very simple and straight forward process, it can
produce incorrect estimates of the velocity or the
position. let’s take for the sake of example a wheel
radius of exactly 3cm with the effects of ware it
has been reduced to 2.9cm with time. although it
might not seem very big a difference but in fact be-
cause of this very tiny and natural change in the
wheel radius, after a 1000 rotation of the wheels,
the geometry calculations will estimate that robot
has travelled by a distance of 188m, however taking
into consideration 2,9cm instead of 3cm the robot
would have travelled 182m. a 6 whooping meters
errors.
An other source of error that might effect the odom-
etry calculation is due to the assumption that the
wheels are moving without slipping in a real life sit-
uation where the floor is wet or slippery, the wheels

```
are rotating which means the wheel encoders are
measuring a rotation of the wheel that a movement
of the robot. while the robot is actually not mov-
ing due to the lack of traction between the wheel
and the ground. let’s now take into consideration
a second odometry system : Laser odometry
```
```
Laser Odometry:The laser odometry system is based
on a 2d sensor sending beams of laser between two
moments in time T1 and T2 and comparing the
differences to conclude the transformation matrix
that occurred to the robot between T1 and T2 thus
the movement of the robot. while this algorithm is
quite elegant and powerful it strongly depends on
the features of the environment, let’s take the ex-
ample of a straight corridor where the external fea-
tures of the environment are the two straight walls
on the sides, if the robot is a making a straight
translation parallel to the two walls, the results of
the observations in t1 and t2 are exactly the same.
while the robot is moving forward our laser odom-
etry system will conclude wrongly that the robot is
not in motion
```
```
B. Odometry Motion Model
```
```
Knowing that our error in odometry calculations is in-
evitable. we suggest using a probabilistic approach for
the odometry problem. proposed by S.THRUN in his
bookProbabilistic Robotics, in this approach the odome-
try is a random variable in a normal distribution and the
position is a not definit but a probability, we rather talk
about the probability of being in a position (x,y,z), this
probability approach is mainly known as the odometry
motion model
Before diving into the probabilistic approach let’s start
with with the following odometry model where the ini-
tial position of the robot is indicated by ( ̄x,y ̄) and the
rotation angleδrot1, while the final position after a trans-
lationδtrans and a translationδrot2−δrot1 is given by
x ̄′,y ̄′
```
```
FIG. 9: Odometry model between the time interval
[t-1,t]
```

Using a little bit of geometryδrot1,δrot2δtrans are
given by:

```
δrot1 = atan2( ̄y′−y, ̄x ̄′− ̄x)−θ ̄ (4)
```
```
δtrans =
```
## √

```
( ̄x− ̄x′)^2 + ( ̄y−y ̄′)^2 (5)
δrot2 =θ ̄′−θ ̄−δrot1 (6)
```
this parameters are only an estimate of the initial po-
sition, in other worlds, they are effected by noise, we
consider noise to guassian noise that depends on the cer-
tain parametersα 1 ,α 2 ,α 3 , andα 4 , this noise parameters
are intrinsic, they depend on the robot itself and vary
from a robot to an other, Figure 10. represents different
odometry motion models using different parametersα,
while, Figure.11 is a visualision in rviz2 to our odometry
motion model usingα= 0.1 and 300 samples after a set
of movements.
The new version of the previous equations taking into ac-
countability the noise affecting each parameter is given
by:

```
δrot1 =ˆ δrot1−p 1 (7)
```
```
δtrans =ˆ δtrans−p 2 (8)
```
```
δrot2 =ˆ δrot12−p 3 (9)
```
p1 = prob(δrot1−δrot1ˆ ,α 1 δrot1 +ˆ α 2 δtrans)ˆ

p3 = prob(δrot2−δrot2ˆ ,α 1 δrot2 +ˆ α 2 δtrans)ˆ

p2 = prob(δtrans−δtransˆ ,α 3 δtrans +ˆ α 4 (δrot1 +ˆ δrot2))ˆ

FIG. 10: sampling from the odometry motion model us-
ing differentαconfigurations

FIG. 11: Odometry motion model visualisation in rviz
using 300 samples

```
VI. MAP REPRESENTATION
```
```
In autonomous robots, mapping the environment is a
very important process since, precise and accurate maps
play a very crucial role in path planning and autonomous
navigation. In addition to this, probabilistic maps add a
layer of accuracy to the imprecision generated by odome-
try systems. maps can differ in their resolution, precision
and types : 3-D 2-D, 1-D, in low memory or real time
applications is a a trade off between precision and com-
putational expenses. For the purpose of this project, we
will use a type of 2-D maps calledOccupancy grid map,
this types of maps are divided into three zones:
```
1. Occupied Zone:represents obstacles in the environ-
    ment and represented by black points,
2. Free Zone:represents the free spaces of the environ-
    ment where the robot can move freely ,represented
    in white.
3. Unknown Zone:represent unknown locations
    where robot is yet to explore,represented in grey.

```
FIG. 12: Enter Caption
```
```
A. Mapping with Known Poses
```
```
Tho goal of any occupancy grid mapping algorithm is
to calculate the posterior probability of the mapm, given
the data:
```
```
p(m|z1:t,x1:t) (10)
```
```
whilex1:tis the set of positions the robot has been in
up to time t, andz1:tare the measurements up to time
t. In other worlds we will take a probabilistic approach
rather than a deterministic approach towards mapping
the environment, we will talk about the probability of
a cell being occupied or free , this probability can be
updated using new measurements as the robot is moving
around the world. Letmidenote the map cell with index
i. An occupancy grid map divides the space into finitely
many grid cells:
```
```
m=
```
## ∑

```
i
```
```
mi (11)
```

```
Xt− 1 Xt Xt+
```
```
Ut− 1 Zt− 1 Ut Zt Ut+1 Zt+
```
```
m
```
```
FIG. 13: Illustration of the mapping process
```
Each mi has attached to it a binary value, which spec-
ifies whether a cell is occupied or free or unknown. We
will write “1” for occupied and “0” for free and ”-1” for
unknown. The notationp(mi= 1) orp(mi) refers to the
probabilitythat a grid cell is occupied. Noting this the
problem posed by equation (13) breaks down into a col-
lection of separate problems of estimating the probability
for each cell:

```
p(mi|z1:t,x1:t) (12)
```
And the posterior over maps -equation(12)- is no other
than,the marginal product ofmi

```
p(m|z1:t,x1:t) =
```
## ∏

```
i
```
```
p(mi|z1:t,x1:t) (13)
```
For detailed calculations ofp(mi) refer toProbabilistic
Roboticsby S.THRUN.

```
VII. LOCALISING WITH A KNOWN MAP
```
What we have done so far can be summarized as follows
:

- Odometry motion model (All possible positions of
    the robot)
- Sensor model (determening Which areas are free or
    no using a sensor)
- An occupancy grid map

The following step seems to be obvious, using this
elements, the data from the sensor, the odometry and
the generated map to estimate the position of the robot
in the map, in other worlds :Localizingthe robot. One
of the most famous Localization Algorithms given a
known map areMarkov LocalizationandMonte Carlo
Localization

```
A. Markov localization
```
```
Markov’s localization is an application of Bayesian fil-
ter algorithm, which is a fundamental and core concept
in probabilistic robotics. The Bayesian filter algorithm
is a two-step process, Prediction and Update it alawys
for tracking multiple independent possible position, it
assigns a probability to each of the possible positions of
the robot on the map, which is the Prediction step, as
we start receiving sensor data, we can update the prob-
ability associated with each cell of the map,this steps
are repeated until arriving at the most probable posi-
tion, which the Gaussian distribution with the smallest
variance.
```
```
B. Monte Carlo Localization
```
```
Monte Carlo localization is a widely used algorithm for
globally localizing autonomous robots, employing ran-
domized sampling techniques to enhance efficiency and
speed. This method, a subset of Markov localization,
operates through two key steps: a measurement update
and a prediction step.
Rather than considering all possible robot positions
on the map, Monte Carlo localization focuses on a sam-
pled subset of these positions. However, unlike Markov
localization, it requires an initial approximate pose and
associated uncertainty (covariance).
Also known as the particle filter, Monte Carlo localiza-
tion employs a set of particles to represent various robot
position hypotheses. Each particle acts as a simulation
of the robot’s position, evaluating how well sensor read-
ings align with the map for each hypothetical pose. This
evaluation assigns weights to particles, reflecting their
likelihood of representing the true robot position.
The algorithm proceeds with a measurement update
phase, using sensor data until arriving at the cell with
highest probability.
the Nav2 library provides an implementation of the
Adaptive Monte Carlo Localization algorithm which we
will use in our project.
Figure 14 is the start of the algorithm and the green
points represent the particles after the measurement up-
date using the sensor data,
```
## FIG. 14


```
FIG. 15: Enter Caption
```
```
FIG. 16: Enter Caption
```
As we start moving the robot in the map (Figure 15), the
particles start spreading around in order and updated us-
ing the sensor data and also taking into consideration the
motion of the robot then we can see in figure 16 that the
particles begin to be more compact around the actual
position of the robot which indicates that the certainty
of the estimation is getting higher as we move.

VIII. SLAM : SIMULTANEOUS LOCALIZATION
AND MAPPING

The approach we used for localization using a known
map is actually possible but it doesn’t provide the best
results, the drawback of this method lies in the assump-
tion we made to generate the map, which that we know
for fact the position of the robotX(t) only based on the
odometry calculations inputU(t), and we didn’t account
for the estimation error that affects the odometry calcula-
tions, a small odometry error can generate an inaccurate
map due to the localization error,figure 17 represnets a
map without odometry errors, while figure 18 is taking
erros into account. For this reason we are going to use
SLAM since it provides more accurate results,there exist
multiple implementations of the SLAM algorithm such
as Extended Kalman Filter SLAM and Graph SLAM,
we are going to use Graph SLAM, we are going to use
slam-toolboxa library offered by ROS2 community to im-
plement SLAM algorithms, Figure 20 represents a map of
the environment generated using SLAM with account for
odometry errors, while figure 19 represents a map gener-

```
ated using the method discussed earlier with odometry
errors.
```
```
FIG. 17: Generated map without odometry errors
```
```
FIG. 18: Generated map with Odometry erros
```
```
FIG. 19: Map with Odometry Error
```
```
FIG. 20: Map using SLAM
```

