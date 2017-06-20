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

## Addendum, second submit
I  implemented the code suggestions from the reviewer. Lowering the
delta (dt) in MPC.cpp to 0.1; setting the second argument for polyeval
to 0, and the epsi to third order deriviative.

The model I use is the kinematic model described in the solution to
the solution for mpc_to_line:
https://github.com/udacity/CarND-MPC-Quizzes/tree/master/mpc_to_line/solution
```
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

```


