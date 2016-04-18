---
layout: page
title: FAQs
permalink: /faq/
---

* #### **We do not currently consider collision avoidance.  Do we need to design our own collision avoidance algorithm?**
Nope!  The Robotarium implements barrier certificates to handle and prevent collisions.  Check out the [simulator](https://github.com/robotarium/robotarium-matlab-simulator) for more detailed information on this function's usage!

* #### **We utilize a single-integrator model for in our algorithm. Do we need to worry about translating this dynamical model to the robots?**
You can, if you wish.  However, the Robotarium already provides multiple functions (e.g., int2uni, int2uni2) that map from single-integrator to unicycle dynamics.  Check out the [simulator](https://github.com/robotarium/robotarium-matlab-simulator) for more detailed information on these function's usage!  

* #### **What are the first steps to implementing our algorithm?  Should we start implementing the controller in the Matlab simulator that is uploaded on the Github?**
Yes!  Check out the [get started](/get_started/) page for a step-by-step description of the workflow and submission process.

* #### **What graph/communication topologies does the Robotarium support?**
Currently, the Robotarium supports using a graph Laplacian (or adjacency matrix) or a "sensing" radius.
