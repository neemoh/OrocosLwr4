This is a component I wrote in 2015 for experimenting with non-energy-storing active constraints.

Due to lack of time (as always!) the architecture is quite messy. 

A master student who was giving me a hand developed a virtual environment for the experiments in H3D but she did not manage to get a good frequency for the haptic loop. So in a rush, I wrote this component in orocos that interfaces with two Sigma haptic devices and generates the forces based on simple geometries. Position of the tools and some other info were sent through UDP to her VE for visualization in 3D, so basically no haptic functionality of h3d was used!

So this component in its entirety should not be usefull anymore, specially since I developed different setups later, but some parts of it may come handy.


Nima, June 2016


