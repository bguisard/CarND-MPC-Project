# Model Predictive Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project's objective is to use a Model Predictive Control (MPC) to drive an autonomous car around a test track. The car will be provided with GPS checkpoints and it's current location and needs to successfully navigate around the track, without leaving the drivable surface.

The MPC also needs to deal with a 100ms latency between it's commands and the actuators.

## MPC

Differently than the [PID](https://github.com/bguisard/CarND-PID-Control-Project), that uses only observations of the current measured error, the MPC relies on appropriately modeling the behavior of the process that it will control.

This gives it a great advantage, as it can estimate future errors and how these would be affected by changes in the controls - in our case speed and steering angle.

The ability to anticipate future events and take actions to prevent bad outcomes is what makes MPC a much more robust option for controlling autonomous vehicles.

## The Kinematic Model

This implementation uses a kinematic model to control it's actuators. Kinematic models are a simplified version of how cars actually behave and they ignore tire forces, lateral forces, gravity and mass.

Although such simplifications really limit the use of these models in a real world application, they are extremely useful to develop control methods such as an MPC. It is also relevant to mention that at low and moderate speeds, the kinematic models offer a good approximation of the real vehicle dynamics.

### 1. Vehicle State

The first step in our model is to define what kind of information we want to model, this is reflected in our "State" - a state is a set of variables that describe the situation the vehicle is in.

We are going to use four variables to represent the state of our vehicle at every time step:
- Cartesian position on the X axis of our map (similar to longitude) [x]
- Cartesian position on the Y axis of our map (similar to latitude) [y]
- Orientation [psi]
- Velocity [v]

### 2. Actuators State

In order to accurately predict the vehicle's position in the future we also need to consider the actuators state, which describes how the orientation and velocity of our vehicle will change in time.

The two actuators we have in our model are the following:
- Steering angle [delta]
- Throttle [a]

### 3. Global Kinematic Model

The six variables above define what we call "State" of the vehicle, that means they describe the exact current situation of the vehicle and give us enough information to model where the car will be in a future time.

That is achieved by modeling the four vehicle state variables as functions of themselves, the actuators state and the time between observations. We will call the time gap between observations delta t (dt).

Using trigonometry and basic physics we can derive the four equations that will describe how the vehicle state changes over time:

1) x(t + dt) = x(t) + v(t) \* cos(psi(t)) \* dt

2) x(t + dt) = y(t) + v(t) \* sin(psi(t)) \* dt

3) psi(t + dt) = psi(t) + v(t)/Lf \* delta \* dt

4) v(t + dt) = v(t) + a(t) \* dt

Lf in Eq. 3 is the distance between the front wheels of the vehicle and it's center of gravity.

### 4. Hyperparameters

Given the state and update functions we have just described, the MPC calculates a number of timesteps ahead of it's current state and measures if the predicted outcome of the current trajectory is adequate or not.

The number of predicted steps, the time gap between steps and the predicted error are all parameters that have to be optimized for each application.

#### Number of Timesteps (N)

N determines how many timesteps will be calculated at each point in time. The caveat is that it is also the major driver of computational cost, not for the obvious reason that more timesteps need to be calculated, but also that for each timestep you add you increase the number of variable to be optimized when minimizing your cost function.

#### Timestep duration

The MPC model tries to approximate the continue path that cars drive by a discrete trajectory of N timesteps, that means that the larger the gap between these points, the higher our uncertainty will be.

In this model I set the prediction horizon to be two seconds, split into ten timesteps, so the timestep duration (dt) is 0.2s. This was optimized through trial and error and I found out that the latency introduces a significant amount of uncertainty into the model and that the later predictions were actually harming the optimization of the actuators inputs.

## Dealing with different coordinate systems

The path our car is supposed to follow is given to us in a global reference, just like a real car would get GPS locations, but the actions it takes are calculated with reference to the car's local coordinate system, so similarly to the Localization problem, we need to convert the GPS coordinates to the car reference.

Conversely what could be done is perform all calculations using the map reference, but converting the waypoints to car reference simplifies some of the state update equations in a way that our approach is more efficient, since the coordinate conversion is a very simple and cheap computation.

## Fitting third degree polynomials to the waypoints

At this point the last decision to be made is the complexity of the polynomial we are going to use to model our predicted path. Simpler polynomials are cheap to calculate, but will deliver higher error, more complex polynomials are expensive to calculate, but will more accurately predict the future states of the vehicle.

Since our prediction horizon is fairly small, two seconds in this case, most roads can be well approximated by a third degree polynomial, for most cases probably even a second degree polynomial would be fine, but the third degree allows for better path estimation when two tight turns are closely connected.

## Error modeling

The most challenging part of designing an MPC is modeling how to optimize the use of the actuators and which of the many different errors you can measure are going to be prioritized.

The several different cost parameters we have are:
1) Squared cross track error - measures the lateral distance between the center of the car and the path it should follow.

2) Squared Orientation error - measures the difference between the desired vehicle orientation and the current vehicle orientation.

3) Squared Reference Velocity error - measures the difference in desired speed and the current vehicle speed. This is a lot less important than the other two errors above, but it serves the purpose of not allowing the MPC to find an optimal solution where the vehicle stops moving.

But just modeling state errors is not enough, in driving constant changing directions or jumping from acceleration to braking and back to accelerating are dangerous behaviors. These type of behavior received the highest weight on our error modeling, meaning that the MPC would allow for smaller deviations from the "best path" in order to have smoother controls.

I have also implemented a cost that would prevent abrupt changes in directions, so the MPC would prefer a constant smooth turn rather than an abrupt change of direction.

## Dealing with latency

All that we have implemented so far are enough to drive in a perfect world, where the effects of your inputs are immediate. In reality, however, there is a delay between your decision to act, the change in the controls and ultimately the change in the behavior of the vehicle.

To account for this latency, the MPC uses the same model that it uses to predict future states, meaning that it will make all it's decision knowing that by the time it needs to change the controls, the state of the vehicle would not be the same as the current state.

In other words, the vehicle will first predict it's state using the current status of all controls (throttle and steering angle in this case) and using the latency as timestep, and only then try to optimize it's path.

## Final Considerations

MPCs are a much more robust model than PIDs and this fine tuned MPC is capable of autonomously driving around our test track carrying significant speeds.

As it was discussed before, the simplifications we assumed under our Kinematic model are not valid at such high speeds as we have on the test video, but since it is a lot more challenging to make the model go fast, even under simplified assumptions, I decided to go for a fast lap.

The final result can be seen on this [video](./videos/fast_lap.mp4). The yellow line is the ground truth path coming from the GPS waypoints and the green line is the MPC designed path.
