---
layout: post
title:  "Formation Control"
date:   2016-03-29
---

Formation control is a network control algorithm which allows a collection of robots to form and maintain a particular shape in a distributed manner. This document provides a mathematical derivation and an implementation of sample code to implement formation control  for the Robotarium.  It summarizes the mathematics behind a decentralized formation control algorithm and highlights the transfer of the formal specification to the Robotarium's MATLAB API.  Additionally, this document contains experimental data from the Robotarium's robots. 

#Problem Statement

Consider a group of $$N$$ agents, where we define the state of each agent as $$x_{i} \in \mathbb{R}^{2},~ i = 1,\ldots,N$$.  This particular algorithm models each agent with the single-integrator dynamics

$$\dot{x}_{i} = u_{i}$$

where $$u_{i} \in \mathbb{R}^{2}$$ is the control input to agent $$i$$.  The goal of formation control is to drive the agents to a translationally independent formation that satisfies a set of given constraints.  For this particular problem, the static communication topology is given by an undirected graph $$G = (V, E)$$ and a function $$d : E \to \mathbb{R}^{2}$$ that models the desired inter-agent distances.  To achieve a particular formation specified by $$E$$ and $$d$$, the communication topology enforced by $$E$$ must be rigid, a concept discussed in [[1](#olfati-saber)], [[2](#beard)], and [[3](#eren)].  For $$\mathbb{R}^{2}$$, the rigidity requirements to ensure a translationally independent formation are

\begin{equation}
\dfrac{\text{car}(E)}{2} = 2N - 3
\label{eq:rigidity}
\end{equation}

where $$\text{car}(E)/2$$ represents the number of edges in an undirected graph (i.e., $$(i, j), (j, i) \in E$$).  Intuitively, (\ref{eq:rigidity}) implies the removal of three degrees of freedom, which is required for translational independence in $$\mathbb{R}^{2}$$.

#Solution

The proposed algorithm utilizes the edge tension energy described in [[4](ji)].  The total edge tension energy of the system may be written as 

$$w(x) = \dfrac{1}{2}\sum_{i = 1}^{N}\sum_{j \in N_{i}} w_{ij}(x)$$

where $$N_{i}$$ is the neighborhood set to agent $$i$$ induced by the graph topology $$G$$.  [[4](ji)] shows that the control law

$$u_{i} = -\dfrac{\partial w(x)}{\partial x_{i}}$$

can be used to drive the agents to the desired configuration, with the proper choice of $$w_{ij}(x)$$.  This control law can be seen as a gradient descent on the edge tension energy of the agents.  Let $$w_{ij}$$ be defined as 

$$w_{ij}(x) = \dfrac{\alpha}{4}(\|x_{i} - x_{j}\|^{2} - d_{ij}^{2})^{2}$$

where $$d_{ij} \in \mathbb{R}^{2}$$ is the desired distance between two agents defined by $$d$$, and $$\alpha \in \mathbb{R}^{+}$$ is a gain.  The gradient of this function is

$$\dfrac{\partial w(x)}{\partial x_{i}} = -\sum_{j \in N_{i}} \alpha(\|x_{i} - x_{j}\|^{2} - d_{ij}^{2})(x_{j} - x_{i})$$

substituting this result into the agent's dynamics yields 
<div>
\begin{equation}
\dot{x}_i = \sum_{j \in N_{i}} \alpha(\|x_{i} - x_{j}\|^{2} - d_{ij}^{2})(x_{j} - x_{i})
\label{eq:formation-control-law}
\end{equation}
</div>

which is a decentralized formation control law.

#Implementation

Using the previously defined algorithm and the Robotarium's provided MATLAB interface, we implemented the formation control algorithm described by (\ref{eq:formation-control-law}).  The code snippet below demonstrates the transfer of (\ref{eq:formation-control-law}) into the Robotarium's MATLAB API.

{% highlight matlab %}
%Calculate single integrator control inputs using edge-energy consensus
for i = 1:N

    % Initialize velocity to zero for each agent.  This allows us to sum
    % over agent i's neighbors
    dx(:, i) = [0 ; 0];

    % Get the topological neighbors of agent i from the communication
    % topology
    for j = r.getTopNeighbors(i, L)

        % For each neighbor, calculate appropriate formation control term and
        % add it to the total velocity

        %%% FORMATION CONTROL %%%

        dx(:, i) = dx(:, i) + ...
        formationControlGain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
        *(x(1:2, j) - x(1:2, i));

        %%% END FORMATION CONTROL %%%
    end 
end
{% endhighlight %}

#Deployment

We deployed this algorithm onto the Robotarium with $$N = 6$$ agents and the formation specification 
<div>
$$
d = 
\begin{bmatrix} 
0 & 0.2 & 0 & 0.2 & 0 & 0.4472 \\
0.2 & 0 & 0.2 & 0 & 0.2 & 0 \\
0 & 0.2 & 0 & 0.4472 & 0 & 0.2 \\
0.2 & 0 & 0.4472 & 0 & 0.2 & 0 \\
0 & 0.2 & 0 & 0.2 & 0 & 0.2 \\
0.4472 & 0 & 0.2 & 0 & 0.2 & 0
\end{bmatrix}
$$
</div>
where each index $$d_{ij}$$ is the distance constraint placed on edges $$(i,j)$$ and $$(j,i)$$.  Note that $$d$$ induces $$9$$ edges, which ensures rigidity by (\ref{eq:rigidity}).  [Figure 1](#fc-data) displays the communication topology and the trajectories of the robots during the experiment.  Note that the agents achieve the desired rectangular formation.  The erratic motion in some agents' trajectories stems from the rigidity of the formation, which restricts the agents' movements.

<a name="fc-data"></a>
![](/assets/formationControl.png)
**Fig 1. Physical robots' trajectories and communication topology ("Edge" in the legend) during the deployment of the decentralized formation control algorithm onto the Robotarium**

##References

<a name="olfati-saber"></a>
[1] R. Olfati-Saber and R. M. Murray, “Distributed structural stabilization
and tracking for formations of dynamic multi-agents,” *Proc. 41st IEEE
Conf. Decision Control*, vol. 1, pp. 209–215, Dec. 2002.

<a name="beard"></a>
[2] R. W. Beard, J. R. Lawton, and F. Y. Hadaegh, “A coordination architecture
for spacecraft formation control,” *IEEE Trans. Control Syst. Technol.*,
vol. 9, no. 6, pp. 777–790, Nov. 2001

<a name="eren"></a>
[3] T. Eren, P. Belhumeur, B. Anderson, and A. Morse, “A framework for
maintaining formations based on rigidity,” in *Proc. 2002 IFAC Congr.*,
pp. 2752–2757.

<a name="ji"></a>
[4] M. Ji and M. Egerstedt. Distributed Coordination Control of Multi-Agent Systems While Preserving Connectedness. *IEEE Transactions on Robotics*, Vol. 23, No. 4, pp. 693-703, Aug. 2007.

