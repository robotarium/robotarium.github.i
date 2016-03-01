---
layout: page
title: Get Started!
permalink: /get_started/
---

Currently the robotarium supports MATLAB scripting to specify robot dynamics. Below we walk through a simple example which implements a cannonical algorithm in network control theory known as the consensus protocol. Download the example files [here](pathtofiles.com).

The consensus protocol in code
==============================
Below is an example consensus algorithm using the Robotarium's MATLAB API.

{% highlight matlab %}

% Declare number of robots
N = 7;

% Initialize Robotarium
robotarium = Robotarium(N);

% Graph Laplacian for a complete graph
L = ones(N, N) - N * eye(N);

while(true)

    % Get the latest pose data from the robots in format 3 x N
    x = robotarium.getPoses()

    % Calculate the control input for the continuous-time consensus
    % dynamics in 2 dimensions
    dx = kron(eye(2), L) * [x(1, :)' ; x(2, :)'];

    % Restructure data
    dx = [dx(1:N)' ; dx(N+1:(2*N))'];

    % Use a provided diffeomorphism to transform 
    % single-integrator to unicycle dynamics
    dx = diffeomorphism(dx, x);

    % Set the velocities of the robots
    robotarium.setVelocities(dx);

end

{% endhighlight %}

Simulate the example code
=========================

Running your code on real robots!
=================================
In order to do this write your own script and verify it in the simulator.  Then contact either [Magnus Egerstedt](mailto:magnus@gatech.edu) or [Zak Costello](mailto:zak.costello@gmail.com) and they will contact you with instructions on how to run your code!