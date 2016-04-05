---
layout: page
title: Get Started!
permalink: /get_started/
---

Currently the robotarium supports MATLAB scripting to specify robot dynamics. Below we walk through a simple example which implements a cannonical algorithm in network control theory known as the consensus protocol. Download the example files [here](https://github.com/robotarium/robotarium-matlab-simulator).

# The consensus protocol in code

Below is an pseudo-example consensus algorithm using the Robotarium's MATLAB API.

{% highlight matlab %}

% Initialize the Robotarium
r = Robotarium();
N = r.getAvailableAgents();
r.initialize(N);

% Create a cyclic graph Laplacian using the built-in graph utilities
L = cycleGL(N);

for i = 1:N

    % Initialize velocity to zero for each agent.  This allows us to sum
    % over agent i's neighbors
    dx(:, i) = [0 ; 0];

    % Get the topological neighbors of agent i based on the graph
    % Laplacian L
    neighbors = r.getTopNeighbors(i, L);

    % Iterate through agent i's neighbors
    for j = neighbors

        %%% CONSENSUS %%%

        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        dx(:, i) = dx(:, i) + (x(1:2, j) - x(1:2, i));

        %%% END CONSENSUS %%%
    end
end

r.setVelocities(dx);
{% endhighlight %}

# Running your code on real robots!

In order to do this write your own script and verify it in the simulator.  Then contact either [Magnus Egerstedt](mailto:magnus@gatech.edu) or [Daniel Pickem](mailto:daniel.pickem@gatech.edu) and they will contact you with instructions on how to run your code!
