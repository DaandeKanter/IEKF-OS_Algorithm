%% Script to simulate the IEKF-OS estimated states. Used for creating video during thesis presentation.
% Perform this script after the IEKF-OS algorithm has been ran.
% This script then simulates the IEKF-OS estimated states by prescribing
% them to the KUKA OpenSim model.

%% Clear Workspace
clear;
close all;
clc;

%% Import OpenSim Libraries into Matlab and import the .osim Model
import org.opensim.modeling.*

% Import geometry files:
path='/Applications/OpenSim 4.1/OpenSim 4.1.app/Contents/Resources/OpenSim/Geometry';
ModelVisualizer.addDirToGeometrySearchPaths(path);

modelFilename = 'Kukaiiwa7_Final.osim';
osimModel = Model(modelFilename);

% Get a reference to the ground object
ground = osimModel.getGround();

%% Visualize in the SimBody visualizer, will open in another window!
fprintf('\n');
prompt = 'Do you want to visualize the simulation? [y]es/[n]o: ';
str1 = input(prompt, 's');
if str1 == 'y'
    fprintf('Visualized in other window!\n');
    osimModel.setUseVisualizer(true);
elseif str1 == 'n'
    fprintf('No visualization!\n');
end


%% Load the OpenSense estimated joint angles

% load('IEKF_Trial4.mat')
load('IEKF.mat')

q_desired  = [IEKF.x_upds(1,:)' IEKF.x_upds(2,:)' IEKF.x_upds(3,:)' IEKF.x_upds(4,:)' IEKF.x_upds(5,:)' IEKF.x_upds(6,:)'];
lengthOfTrial = (length(IEKF.x_upds(1,:))-1)/100;
tSamples   = 0:0.01:lengthOfTrial;
SampleFreq = 100; % Hz
reportTimeInterval = 1/SampleFreq;

%% Prescribe motion:

% Get coordinate set from model, and count the number of coordinates
modelCoordSet = osimModel.getCoordinateSet();
nCoords = modelCoordSet.getSize()-1;

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

%% Finalize connections and initialize the System (checks model consistency)
% initSystem() does the following:   * Generate the system of equations
%                                    * Assemble to satisfy and constraints
%                                    * Returns the initial state
%
% If no intial state is specified, the state will just be set to 0.
% state = osimModel.initSystem();
state = osimModel.initSystem();

%% Visualize the simulation time in the visualizer
if str1 == 'y'
    sviz = osimModel.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
end

%% Simulation of the model and update states: Simulation Option B
dTime = reportTimeInterval; % Sample frequency of Xsens IMU
finalTime = lengthOfTrial; %5;
n = round(finalTime/dTime);

% Manager: A class that manages the execution of a simulation.
manager = Manager(osimModel);
% This must be called before calling Manager::integrate() 
manager.initialize(state);

%% Start simulation

for t = 1:n
    %______________________________________________________________________
    %Integrate to the new time: t = t + dTime
    state.setTime((t-1)*dTime); % Start at t = 0
    
%     fprintf(' Time prior integration step is set to : %6.4f\n ', state.getTime)

    % Integrate the Equations of Motion for the specified model, given the
    % current state (at which the integration will start) and a finalTime.
    % finalTime here is the new time step: dTime.
    state = manager.integrate(t*dTime); % Integrate the EoM to the new state

%     fprintf(' Time after integration step is        : %6.4f\n', state.getTime) 
end
