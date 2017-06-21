# CarND MPC Project
My hand in for the Self-Driving Car Engineer Nanodegree Program

## What I did
The structure for MPC.cpp and main.cpp are more or less the structure
given in the lectures & quizzes imbedded into the project.

Adjustments where needed for the parameters and coeffisients in the ()
operator in FG_eval and in the solve function. I used a combination of
reading discussion on slack and the discussion site, combined with try
and failure.

I still do not divide the steering angle result from the solve() with
deg2rad(25) in the main function.

My other challenge was how to get projected steering (green line)
displayed in the simulator.

I still do not understand the implications of all coeffisients, what
an optimal timestep and deltatime are, but the car keeps on the road ;)

## Addendum; second submission
### Code comments
I  implemented the code suggestions from the reviewer. Lowering the
delta (dt) in MPC.cpp to 0.1; setting the second argument for polyeval
to 0, and the epsi to third order deriviative.


### The states and equations
The model I use is the kinematic model described in the quiz *mpc_to_line*:
https://github.com/udacity/CarND-MPC-Quizzes/tree/master/mpc_to_line/solution
```
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

```
where x and y are the position of the car, psi is the orientation, v
the velocity and cte is the cross track error and epsi the error in
orientation.

This is a simplified model that ignore things like mass.

### The reasoning behind the chosen N
I chose the number of timestamps (N) and time between measurements
(dt) empirically. To long horizon increased the chances of erronous
predictions.

I also found that the max velocity was very dependent of different
coefficients for the cost function (f[0]). 

### 100 millisecond latency
I implemented latency using method two suggested in the submission
review.

The solver uses the same steering and actuator values as was used the
last iteration.
```
 
  for (int i = delta_start; i < delta_start + dtperlatency; i++) {
    vars_lowerbound[i] = last_delta;
    vars_upperbound[i] = last_delta;
  }
  ...
  
 
  for (int i = a_start; i < a_start + dtperlatency; i++) {
    vars_lowerbound[i] = last_a;
    vars_upperbound[i] = last_a;

```


## Addendum; third submission
### Code comments
I added the latency into the state vector by embedding the kinematic
equations in main.cpp: 

```
          state[0] = v * cos(0) * latency;
          state[1] = v * sin(0) * latency;
          state[2] = (-v / Lf) * delta* latency;
          state[3] = v + a * 0.1;
          state[4] = cte + v*sin(epsi)*0.1;
          state[5] = epsi - (v / Lf) * delta * 0.1;
```

The car now runs ok at much higher speed, and never leaves the road in
my simulation.

### The reasoning behind the chosen N
I kept the N at 15 and dt at 0.1, creating a horizon at 1.5
seconds. This seems to make a good reasonable horizon for the prediction. The car runs ok with N
at 20, two seconds ahead, but I cannot observe any significant
improvement in the projected path (green line).
