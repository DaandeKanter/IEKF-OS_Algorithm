# IEKF-OS_Algorithm
IEKF-OS motion reconstruction algorithm - MSc Thesis Project - Systems and Control - Delft University of Technology

Script created by Daan de Kanter for MSc Thesis project.                
Project is supervised by Dr. M. Kok, Delft University of Technology     
                         Dr. Ir. A. Seth, Delft University of Technology

The IEKF-OS Motion Reconstruction Algorithm is based on the following components:
* Raw 3D gyroscope measurements.
* Raw 3D accelerometer measurements.
* The Iterated Extended Kalman Filter.
* The system's dynamical model generated for a user-defined OpenSim model.

Running the algorithm:
1. Run the script: KUKA_iiwa_7_IEKF.m
* To run the IEKF-OS algorithm which reconstructs the motion based on experimentally obtained IMU data. The IMUs were attached to the six links of the KUKA robot.
