## Model Predictive Control

### The Model

#### State, Actuators and Update Equations

State

State is received from simulator, with the following parameters: x, y - the vehicle coordinates psi - vehicle orientation speed - vehicle speed x, y, and psi are used to convert waypoint coordiates from map space to vehicle space. Vehicle space is used for path planning and polynomial fitiing.

Actuators

There are two actuators returned by the solver. delta (in file mpc.cpp) which is steering actuator - solver returns values which should be inverted to have correct steering angle. a (in file mpc.cpp) which is throttle actuator with positive values meaning acceleration and negative meaning braking. The values are returned by solver and passed ot the simulator.

Update equations

The update equations are used to compute the state of the car at the next time step, based on the state at the current time step:

x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta * dt
v_t+1 = v_t + a_t * dt

### Timestep Length and Elapsed Duration (N & dt)
Started with speed = 30.Initially assumed T=0.05, N=30 ie dt=0.05/30. Which was too less and resulting in larger control inputs very close to the car. Hence car was  oscillating much. Then increased T=1.5 and N=10. So car predicts the future for next 1.5 sec. It must not be too high which results in the failure at curves where car has to react quickly as compared to the smooth-straight road.
It was observed that car was following the path correctly.
Then increased the speed 40 keeping the time parameters as above. Car follows the path correctly but frequent throttling and braking. So with less speed ie upto 30. After multiplying th cost function for accelerating by 100 or 50 or 10, the problem solved. But car goes off track on the steep curves. 
with Cte cost 100 , car was driving smoothly on the curves too. But with slight oscillations. It was observed that predicted states at front of the car was correct, but at the later stage ie away from the car is deviating heavily and of no use. At speed of 40 it has no effect, but certainly will affect at high speed.
Next tried with speed = 60.
Car was started with slight oscillations. And was moving at speeds of 55. But lot of oscillations at corners.But it was moving on track except at one or two locations touched the side track.  Still the predictions at end of the green line(away from the car) was terribly deviating. I think the car adjusts well for local and nearest reactions.

Changed T=1, N=10, speed = 60. Results in lot of oscillations at the start itself. T=1.5 to 1 affects heavyily. So have fixed on the N and dt values of 10 and 1.5 respectively.

So tried with increasing steering error cost function. With values of 50 it worked well. But car goes off track at corners.

After increasing cte cost function to 500 and eps cost function to 100, car was behaving well at speed of 60.


### Polynomial Fitting and MPC Preprocessing

In a first step, the waypoints are transformed into the vehicle coordinate system. The resulting x-direction is the forward direction of the vehicle, while the y-direction represents the lateral displacement of waypoints relative to the center of the vehicle. 3rd degree polynomial fitting was used by utilizing the provided functions. When constructing the initial state vector for the solver, the values for x, y, and psi can now be conveniently set to zero because the coordinate system is defined relative to the vehicle. The initial cross track error is equal to the constant term of the fitted polynomial in this coordinate system, and the initial orientation error is calculated from the first derivative of the polynomial at x=0. 

### Model Predictive Control with Latency
Total computing time for the model is 1.5 Sec. Latency(0.1 Sec) definately plays role, but its effect is lesser in my model as it looks ahead of 1.5 Sec. 

## Below are the links to the Car Simulation Video
### Model Predictive Control
[https://youtu.be/84K_C37vgK8](https://youtu.be/84K_C37vgK8)

## Second Submission

To include the effect of latency, I have predicted the vehicles  forward position after the 100 ms.
Rest of the procedure was similar. After lot of trial and error I can get the vehicle run at speed of 60.


