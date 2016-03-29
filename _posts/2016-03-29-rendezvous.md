---
layout: post
title:  "Rendezvous"
date:   2016-03-29
---

Rendezvous is a network control algorithm which allows a collection of robots to meet at at the centroid of their initial positions. This document is a mathematical companion to the rendezvous sample code for the Robotarium.  It summarizes the mathematics behind the consensus algorithm and highlights the transfer of the formal specification to the Robotarium's MATLAB API.  Additionally, this document contains experimental data from the Robotarium's robots. 

Problem Statement
=================

Consider a group of $$N$$ agents, where we define the state of each agent as $$x_{i} \in \mathbb{R}^{2},~ i = 1,\ldots,N$$.  This particular algorithm models each agent with the single-integrator dynamics
$$
\dot{x}_{i} = u_{i}
$$
where $$u_{i} \in \mathbb{R}^{2}$$ is the control input to agent $$i$$.  The rendezvous problem requires the design of a control input $$u_{i}$$ such that 
<div>
\begin{equation}
    \lim_{t \to \infty} (x_{i} - x_{j}) = 0, ~ \forall ~ i,j = 1,\ldots,N
    \label{eq:solution}
\end{equation}
</div>

Solution
========

A common solution to the rendezvous problem is to let $u_{i}$ be defined using the local interaction rule (e.g., as in [[1](#jadbabaie)]).
$$
u_{i} = \sum_{j \in N_{i}} (x_{j} - x_{i})
$$
yielding the node-level dynamics
<div>
\begin{equation}
    \dot{x}_{i} = \sum_{j \in N_{i}} (x_{j} - x_{i})
    \label{eq:node-level-dynamics}
\end{equation}
</div>

where $$N_{i}$$ represents the neighbors of agent $$i$$ induced by a particular communication topology.  For this algorithm, consider the communication topology of the agents to be static and represented by an undirected graph $$G = (V, E)$$ where the following condition holds 
$$
j \in N_{i} \iff (i, j), (j, i) \in E
$$
Then, the node-level dynamics may be re-written as ensemble-level dynamics by combining the agents' states together into the vector
$$
x = \left[ x_{1,1} ~ x_{2,1} \ldots ~ x_{N,1} ~ x_{1, 2} ~ x_{2, 2} ~ \ldots ~ x_{N, 2} \right]^{T}
$$
yielding the ensemble-level dynamics 
$$
\dot{x} = -(I \otimes L)x
$$
where $$L$$ is the graph Laplacian to the graph $$G$$, and $$I$$ is an identity matrix of the appropriate dimension.  Note that $$x_{i, j}$$ refers to the $$j$$th state variable of agent $$x_{i} \in \mathbb{R}^{2}$$.  This algorithm is known as the consensus algorithm.  Using the properties of $$L$$, we can also show each agent's final position is given by
$$
\lim_{t \to \infty} x_{i}(t) = \dfrac{1}{N} \sum_{j=1}^{N} x_{j}(0), ~ \forall ~ i = 1,\ldots,N
$$
where $$x_{j}(0)$$ is the initial condition of agent $$j$$.  This result solves the problem stated in(\ref{eq:solution}).  Interestingly, the agents always meet at the average of the initial conditions.  For relevant properties of the graph Laplacian, refer to sources such as [[2](#godsil)].  

Implementation
==============

Using the previously defined algorithm and the Robotarium's provided MATLAB interface, we implemented the consensus algorithm in (\ref{eq:node-level-dynamics}).  The code snippet below demonstrates the transfer of the consensus algorithm into the Robotarium's MATLAB API.

{% highlight matlab %}
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
{% endhighlight %}

Deployment
==========

For the experiment, we selected $$N = 6$$ and $$G = C_{6}$$ (i.e., an undirected cycle graph containing 6 agents).  This choice for $$G$$ ensured that the agents remained connected during the experiment.  Deploying the consensus algorithm onto the Robotarium yielded the results shown in [Figure 1](#consensus-data).  Note that, due to the physical size of the robots, the agents did not reach the same point.

<a name="consensus-data"></a>
![](/assets/rendezvous.png)
**Fig 1. Physical robots' trajectories during the deployment of the consensus algorithm onto the Robotarium**


References
==========

<a name="jadbabaie"></a>
[1] A. Jadbabaie, J. Lin, and A. S. Morse, “Coordination of groups of mobile
autonomous agents using nearest neighbor rules,” *IEEE Trans. Autom. Control*, vol. 48, no. 6, pp. 988–1001, Jun. 2003

<a name="godsil"></a>
[2] C. Godsil and G. Royle, *Algebraic Graph Theory*. Berlin, Germany:
Springer, 2001
