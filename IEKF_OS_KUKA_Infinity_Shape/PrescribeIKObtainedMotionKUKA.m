%% Script simulating the Kukaiiwa7_6Link model to create "measurements"
%% Clear Workspace
clear;
close all;
clc;

%% Import OpenSim Libraries into Matlab and import the .osim Model
import org.opensim.modeling.*

% Import geometry files:
path='/Applications/OpenSim 4.1/OpenSim 4.1.app/Contents/Resources/OpenSim/Geometry';
ModelVisualizer.addDirToGeometrySearchPaths(path);

% Open Kuka iiwa 7 OpenSim model
modelFilename = 'Kukaiiwa7_Final.osim';
osimModel = Model(modelFilename);

%% Load the IK obtained solution for tracking an infinity shaped trajectory
%  Obtained from the script KUKA_iiwa_7_IK_Infinity.m

% load('JointAngles_q_Desired_200Hz_5seconds.mat')
load('JointAngles_q_Desired_100Hz_5seconds.mat')
% load('JointAngles_q_Desired_10Hz_5seconds.mat')
% FinalTime = length(q_desired(:,1));

%% Prescribe motion:

% Get coordinate set from model, and count the number of coordinates
modelCoordSet = osimModel.getCoordinateSet();
nCoords = modelCoordSet.getSize();

Time = tSamples';

% For all coordinates in the model, create a function and prescribe
for i=0:nCoords-1

    % Get the coordinate set from the model
	currentcoord = modelCoordSet.get(i);
    coordvalue   = q_desired(:,i+1);

    % Construct a SimmSpline object (previously NaturalCubicSpline)
    Spline = SimmSpline();
    
    % Now to write Time and coordvalue to Spline
    % If the motion type is rotational we must convert to radians from degrees
        for j = 0:length(coordvalue(:,1))-1
            Spline.addPoint(Time(j+1,1),coordvalue(j+1,1));
        end

    % Add the SimmSpline to the PrescribedFunction of the Coordinate
    % being edited
    currentcoord.setPrescribedFunction(Spline);
    currentcoord.setDefaultIsPrescribed(1);
end

%% Save the .osim model to a file to visualize in OpenSim
modelFilenameForOpenSim = 'Kukaiiwa7_Tracking.osim';
osimModel.print(modelFilenameForOpenSim);
disp('Kukaiiwa7_Tracking.osim printed!');