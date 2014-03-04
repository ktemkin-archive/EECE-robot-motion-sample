EECE-robot-motion-sample
========================

A simple "dead reckoning" movement engine for the BU robotic platform; intended as sample code for students. 

<i>An important note: this algorithm is implemented purely in software-- it doesn't take advantage of any of the microcontroller's hardware features!</i> You can, and almost definitely should, write up a better algorithm to control your robot-- this is intended as sample code.

See the four source files for more details:

- *basys.h*: Header file for using device on the Basys SoftAVR.
- *movement.h*: Header file for using movement library.
- *movement.c*: Source for the primary movement engine.
- *main.c*: The user interface for the sample project.
