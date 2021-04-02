%% Inverse Kinematics script for the KUKA iiwa 7 robot
% Waypoint tracking demonstration using Robotics System Toolbox.
% This script performs inverse kinematics of a robot manipulator to follow 
% a desired set of waypoints.
% _________________________________________________________________________
% Maybe change the urdf file with the GITHUB specs:
% https://github.com/epfl-lasa/iiwa_ros/blob/master/iiwa_description/urdf/iiwa7.xacro

%% Clear Workspace
clear;
close all;
clc;

%% Load the KUKA iiwa 7 robot

lbr = importrobot('iiwa7.urdf'); % 7 kg payload version
lbr.DataFormat = 'row'; % Outcomment to obtain a struct

% _________________________________________________________________________
% % Only uncomment after an Inverse Kinematic simulation has been run!
% % To check if initial guess corresponds to starting position of the KUKA.
% % Uncomment lbr.DataFormat = 'row'; above for the code below to work!
% config = homeConfiguration(lbr);
% load('Initial.mat')
% config(1).JointPosition = Initial(1,1);
% config(2).JointPosition = Initial(1,2);
% config(3).JointPosition = Initial(1,3);
% config(4).JointPosition = Initial(1,4);
% config(5).JointPosition = Initial(1,5);
% config(6).JointPosition = Initial(1,6);
% config(7).JointPosition = Initial(1,7);
% show(lbr, config)
% axis([-0.5 1 -1 1 0 1.5]);
% _________________________________________________________________________

%% Add the gravity vector
gravityVec = [0 0 -9.81];
lbr.Gravity = gravityVec;

axes = show(lbr);
axis([-0.5 1 -1 1 0 1.5]);

%% Create a set of desired waypoints to form an infinity loop sign
offsetX = 0.3;
wayPoints = [ offsetX  0     0.75;
              offsetX -0.25  0.95;
              offsetX -0.375 1;
              offsetX -0.6   0.95;
              offsetX -0.75  0.75;
              offsetX -0.6   0.55;
              offsetX -0.375 0.5;
              offsetX -0.25  0.55;
              offsetX  0     0.75;
              offsetX  0.25  0.95;
              offsetX  0.375 1;
              offsetX  0.6   0.95;
              offsetX  0.75  0.75;
              offsetX  0.6   0.55;
              offsetX  0.375 0.5;
              offsetX  0.25  0.55;
              offsetX  0     0.75];
          
% Plot the waypoints
set(gcf,'color','w');
exampleHelperPlotWaypoints(wayPoints);
axis([-0.5 1 -1 1 0 1.5]);

%% Create a smooth trajectory from the waypoints
SampleFreq = 100; %Hz
tInterval = [0 5];
tSamples = 0:1/SampleFreq:tInterval(end);
[q,qd,qdd,pp] = bsplinepolytraj(wayPoints',tInterval,tSamples);

% Plot trajectory spline and waypoints
hold on
plot3(q(1,:),q(2,:),q(3,:),'r-','LineWidth',2);

%% Perform Inverse Kinematics
% Use desired weights for solution (First three are orientation, last three are translation)
% No weights on the rotations but only weights placed on the translations

ik = robotics.InverseKinematics('RigidBodyTree',lbr);
weights = [0 0 0 1 1 1];
initialguess = lbr.homeConfiguration;

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as the initial guess for the next iteration.
for idx = 1:size(q,2)
    tform = trvec2tform(q(:,idx)');
    configSoln(idx,:) = ik('iiwa_link_ee_kuka',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end

%% Visualize robot configurations
title('Robot waypoint tracking visualization')
set(gcf,'color','w');
framerate = 60;
r = rateControl(framerate);
for idx = 1:size(q,2)
    show(lbr,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    waitfor(r);
end
hold off

%% Save desired q states:
q_desired = configSoln;
save('JointAngles_q_Desired_100Hz_5seconds.mat', 'q_desired', 'tSamples', 'SampleFreq');

%% Save the desired q states in the desired .txt delimited space format
% writematrix(q_desired,'q_desired_tab.txt','Delimiter','space')
% type 'q_desired_tab.txt'