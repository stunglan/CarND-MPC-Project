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
