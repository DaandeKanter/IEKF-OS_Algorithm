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
1. Run the script: ModelDoublePendulumTwoHingeJoints.m to create the Dobule Pendulum OpenSim model.

2. Run the script: DoublePendulumTwoHingeJointsCreatingMeasurements.m to create the virtual IMU measurements, being the 3D angular velocity and 3D linear acceleration measurements of each attached IMU.

3. Run the script: DoublePendulumTwoHingeJointsIEKF.m to run the IEKF-OS algorithm which reconstructs the motion.
