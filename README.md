# Multi_Robots_DMPC 
Cooperative control and trajectory planning of the multiple mobile vehicles via distributed model predictive control to implement the following tasks including formation control, inter-vehicle obstacle avoidance and environment obstacle avoidance. Stability, feasibility and optimality must be guaranteed. For real test, these codes will be deployed in the three Raspberry PI 3. A cameral and AprilTags visual localization system are used for localisation. Codes and demos are not complete and are debugged and kept updating.
## Simulation for the Trajectory Planning using Centralized Structure
Centralized structures usually leads to large optimization problem, which is time-consuming. The second graph below shows the case in which three vehicles are moving in the formation of a triangle.

<img width="300" heigth="300" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/flower.gif"/><img width="300" heigth="300" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/form_cen.gif">

## Simulation for the Trajectory Planning using Decentralized Structure
Decentralized structures contributes to faster solving of the optimization problem. The trajectories of MPC are shown with the horizon 10

<img width="300" heigth="300" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/dmpc1.gif"><img width="300" heigth="300" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/dmpc2.gif">

## Simultation for the Formation Movement using Decentralized Structure
The three vehicles form a group as a triangle and break the form due to the emergency.

<img width="300" heigth="300" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/de07form.gif">

## Hardwares for Real Test
Three raspberry pi 4 are used for running the main programs while a Realsense D435 RGBD cameral and AprilTag system are used for localization.

<img width="300" heigth="300" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/3v.png">

## Trajectory Planning using MPC for the Obstacle Avoidance
The following animation shows the MPC implementation for obstacle avoidance, which is the important fundation for DMPC of the multiple vehicles.

<img width="550" heigth="750" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/mpc3.gif">

## Trajectory Planning using the DMPC for Two Vehicles
The following animation shows the DMPC implementation for two vehicles to avoid inter-vehicles collision and approach their destinations.

<img width='550' heigth='750' src='https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/twovehicles2.gif'>


## Trajectory Planning using the DMPC for Three Vehicles and the Formation Real Test

The following animations show the DMPC implemetation for three vehicles to avoid inter-vehicles collision and approach their destinations.

<img width='550' heigth='750' src='https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/three01.gif'>

<img width='550' heigth='750' src='https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/three02.gif'>
