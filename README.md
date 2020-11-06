# Multi_Robots_DMPC
Cooperative control of the multiple mobile vehicles via distributed model predictive control to implement the following tasks including formation control, inter-vehicle obstacle avoidance and environment obstacle avoidance. Stability, feasibility and optimality must be guaranteed. For real test, these codes will be deployed in the three Raspberry PI 4. A cameral and AprilTags visual localization system are used for localisation.
# Simulation for Centralized Structure
Centralized structures usually leads to large optimization problem, which is time-consuming. The second graph below shows the case in which three vehicles are moving in the formation of a triangle.

<img width="400" heigth="400" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/flower.gif"/>
<img width="400" heigth="400" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/form_cen.gif">

# Simulation for Decentralized Structure
Decentralized structures contributes to faster solving of the optimization problem. The trajectories of MPC are shown with the horizon 10

<img width="400" heigth="400" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/dmpc1.gif">
<img width="400" heigth="400" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/dmpc2.gif">

# Hardwares for Real Test
<img width="400" heigth="400" src="https://github.com/HAOLI-TUKL/Multi_Robots_DMPC/blob/master/pic/vehicle.jpeg">
