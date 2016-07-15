# Dynamic non-energy-storing active constraints for position guidance.

This folder contains an orocos component that generates guidance forces based on the curretn position of a tool and its desired one.

Three dynamic active constraint methods have been implemented.
For the describption of these methods and discussions on their behaviour please refere to:

" PAPER FIX "

The implementation of the methods is accesiible as a separate C++ library (acMethods.cpp and acMethods.hpp) that can be used with any c++ application as long as KDL types are defined.
