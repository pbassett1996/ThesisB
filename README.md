# Thesis B

The following repository satisify's the faculty thesis rules. It includes all the MATLAB simulation's used to validated the software designs in Group 33's Thesis B. 

## Getting Started

The programs are designed to be run on MATLAB/Simulink. Included in the repository are the .mat files for GPS and IMU data used in some of the simulations. Simply downloading the program files and running them on MATLAB will suffice. The zip file "Octocopter_Simulink_Model.zip" contains the simulink model, it must be extracted prior to its use.

### Prerequisites

All programs are to be run on MATLAB. The following is a list of add ons necessary:
- Simulink
- Simulink Control Design
- Signal Processing Toolbox
- Control Systems Toolbox
- Communications Toolbox
- Aerospace Toolbox
- Aerospace Blockset

### Break down of programs

Octocopter_Simulink_Model.zip
The zip file contains all the necessary components for an octocopter simulation model. The purpose of this is to see the interaction of the control system design with the octocopters dynamics. The model includes an inertial measurement unit (IMU), pressure simulation, and a sensor fusion model. The model also allows the extraction of realistic data necessary for the simulation of other software designs.

Control_System.m
This file simply takes the GPS informaion from the octocopter simulnk model and plots the horizontal and vertical displacments with time. It is intended for real-time analysis so that the behaviour of the contorl systems can be verified.

EKF_Sim.m
This files uses the IMU and GPS information from the octocopter simulink model and fuses them together using an Extended Kalman Filter (EKF). The program plots the vehicle's position using the actual, inertial and EKF measurements. This allows direct comparison between the different methods.

obstacle_avoidance.m
This program is used for verification of the collision avoiance system. Laser data attained from Jose Guivant in MTRN4010 is clustered into potential obstacles in the vehicle's path, and then a potential field algorithm is applied to allow the vehicle to navigate through the objects.

spectrum_sensing.m
This program is used for the validation of the spectrum sensing algorithm. A virtual radio spectrum is created with noise and attenuation added. A small pocket of "low interference" is created. The algorithm is used to identify this desirable frequency band usin

## Built With

* [MATLAB](https://www.mathworks.com/products/matlab.html) - Desktop environment used

## Contributing

* [Markus Mikkelsen](https://pdfs.semanticscholar.org/4e00/fa5e741040b19252e88d4a781c423402c9bc.pdf) - Development of a multirotor Simulink model

## Authors

* **Peter Bassett**

## Acknowledgments

* Thanks to David Lyon for providing guidance throughout the Thesis A and Thesis B projects
* Thanks to Jose Guivant who created the laser data for the obstacle avoidance simulation







