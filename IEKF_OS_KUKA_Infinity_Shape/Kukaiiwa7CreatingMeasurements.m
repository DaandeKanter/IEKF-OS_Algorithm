%% Script simulating the Kukaiiwa7_6Link model to create "measurements"
%% Clear Workspace
clear;
close all;
clc;
tic

%% Import OpenSim Libraries into Matlab and import the .osim Model
import org.opensim.modeling.*

% Import geometry files:
path='/Applications/OpenSim 4.1/OpenSim 4.1.app/Contents/Resources/OpenSim/Geometry';
ModelVisualizer.addDirToGeometrySearchPaths(path);

% Open Kuka iiwa 7 OpenSim model
modelFilename = 'Kukaiiwa7_Tracking.osim';
osimModel = Model(modelFilename);

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

%% Create an coordinate actuator for the Revolute0_1 joint
optimalForce = 1;      % Set the optimal force. The maximum torque produced by this actuator when fully activated.
                       % Ajay told me this is essentially a gain which multiplies the user-set value for tau in
                       % the component brain.PrescribedController()
                       
% Get the number of coordinates and a handle to the coordinate set
coordSet = osimModel.getCoordinateSet();

% Get a reference to the first coordinate
Rotation0_1 = coordSet.get(0);
   
% Create a coordinate actuator for the Revolute0_1 joint
Rotation0_1Actuator = CoordinateActuator( char(Rotation0_1.getName) );
Rotation0_1Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
Rotation0_1Actuator.setName( Rotation0_1.getName() ); % Set the actuator name 
Rotation0_1Actuator.setMaxControl(Inf)                % Set max controls
Rotation0_1Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set

osimModel.addForce(Rotation0_1Actuator);

%% Create an coordinate actuator for the Revolute1_2 joint
% Get a reference to the second coordinate
Rotation1_2 = coordSet.get(1);
   
% Create a coordinate actuator for the Revolute1_2 joint
Rotation1_2Actuator = CoordinateActuator( char(Rotation1_2.getName) );
Rotation1_2Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
Rotation1_2Actuator.setName( Rotation1_2.getName() ); % Set the actuator name 
Rotation1_2Actuator.setMaxControl(Inf)                % Set max controls
Rotation1_2Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set
osimModel.addForce(Rotation1_2Actuator);

%% Create an coordinate actuator for the Revolute2_3 joint
% Get a reference to the third coordinate
Rotation2_3 = coordSet.get(2);
   
% Create a coordinate actuator for the Revolute2_3 joint
Rotation2_3Actuator = CoordinateActuator( char(Rotation2_3.getName) );
Rotation2_3Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
Rotation2_3Actuator.setName( Rotation2_3.getName() ); % Set the actuator name 
Rotation2_3Actuator.setMaxControl(Inf)                % Set max controls
Rotation2_3Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set
osimModel.addForce(Rotation2_3Actuator);

%% Create an coordinate actuator for the Revolute3_4 joint
% Get a reference to the fourth coordinate
Rotation3_4 = coordSet.get(3);
   
% Create a coordinate actuator for the Revolute3_4 joint
Rotation3_4Actuator = CoordinateActuator( char(Rotation3_4.getName) );
Rotation3_4Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
Rotation3_4Actuator.setName( Rotation3_4.getName() ); % Set the actuator name 
Rotation3_4Actuator.setMaxControl(Inf)                % Set max controls
Rotation3_4Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set
osimModel.addForce(Rotation3_4Actuator);

%% Create an coordinate actuator for the Revolute4_5 joint
% Get a reference to the fifth coordinate
Rotation4_5 = coordSet.get(4);
   
% Create a coordinate actuator for the Revolute4_5 joint
Rotation4_5Actuator = CoordinateActuator( char(Rotation4_5.getName) );
Rotation4_5Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
Rotation4_5Actuator.setName( Rotation4_5.getName() ); % Set the actuator name 
Rotation4_5Actuator.setMaxControl(Inf)                % Set max controls
Rotation4_5Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set
osimModel.addForce(Rotation4_5Actuator);

%% Create an coordinate actuator for the Revolute5_6 joint
% Get a reference to the sixth coordinate
Rotation5_6 = coordSet.get(5);
   
% Create a coordinate actuator for the Revolute5_6 joint
Rotation5_6Actuator = CoordinateActuator( char(Rotation5_6.getName) );
Rotation5_6Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
Rotation5_6Actuator.setName( Rotation5_6.getName() ); % Set the actuator name 
Rotation5_6Actuator.setMaxControl(Inf)                % Set max controls
Rotation5_6Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set
osimModel.addForce(Rotation5_6Actuator);

%% Set the initial values of the coordinates and their speeds
%     [ q1 ] % Generalized coordinate 1       of the Rotation0_1
%     [ q2 ] % Generalized coordinate 2       of the Rotation1_2
%     [ q3 ] % Generalized coordinate 3       of the Rotation2_3
%     [ q4 ] % Generalized coordinate 4       of the Rotation3_4
%     [ q5 ] % Generalized coordinate 5       of the Rotation4_5
%     [ q6 ] % Generalized coordinate 6       of the Rotation5_6
% x = [ u1 ] % Generalized velocity 1         of the Rotation0_1
%     [ u2 ] % Generalized velocity 2         of the Rotation1_2
%     [ u3 ] % Generalized velocity 3         of the Rotation2_3
%     [ u4 ] % Generalized velocity 4         of the Rotation3_4
%     [ u5 ] % Generalized velocity 5         of the Rotation4_5
%     [ u6 ] % Generalized velocity 6         of the Rotation5_6
%     [tau1] % Control input / joint torque 1 of the Rotation0_1
%     [tau2] % Control input / joint torque 2 of the Rotation1_2
%     [tau3] % Control input / joint torque 3 of the Rotation2_3
%     [tau4] % Control input / joint torque 4 of the Rotation3_4
%     [tau5] % Control input / joint torque 5 of the Rotation4_5
%     [tau6] % Control input / joint torque 6 of the Rotation5_6

% Get a reference to the ground object
ground = osimModel.getGround();

% Define the acceleration of gravity
Gravity = -9.81;
osimModel.setGravity(Vec3(0, Gravity, 0));
GravityVec = Vec3(0, -Gravity, 0); % For later use, - - Gravity = + Gravity
%                                  % The IMU measures the reaction hence +

load('JointAngles_q_Desired_100Hz_5seconds.mat')


% Setting of initial conditions

% % Setting q1 and u1
Rotation0_1.setDefaultValue(q_desired(1,1));
angle1_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
Rotation0_1.setDefaultSpeedValue(angle1_rad_s);

% Setting q2 and u2
Rotation1_2.setDefaultValue(q_desired(1,2));
angle2_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
Rotation1_2.setDefaultSpeedValue(angle2_rad_s);

% Setting q3 and u3
Rotation2_3.setDefaultValue(q_desired(1,3));
angle3_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
Rotation2_3.setDefaultSpeedValue(angle3_rad_s);

% Setting q4 and u4
Rotation3_4.setDefaultValue(q_desired(1,4));
angle4_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
Rotation3_4.setDefaultSpeedValue(angle4_rad_s);

% Setting q5 and u5
Rotation4_5.setDefaultValue(q_desired(1,5));
angle5_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
Rotation4_5.setDefaultSpeedValue(angle5_rad_s);

% Setting q6 and u6
Rotation5_6.setDefaultValue(q_desired(1,6));
angle6_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
Rotation5_6.setDefaultSpeedValue(angle6_rad_s);


%% Create frames that will represent the "real" IMUs

% --------------------------------Xsens1---------------------------------
% Get a handle to the first link from the loaded OpenSim model
Link1    = osimModel.getBodySet.get('link_1');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 1.
Xsens1 = PhysicalOffsetFrame(); 
Xsens1.setName('FirstXsensIMU');
Xsens1.setParentFrame(Link1); % Select the body to which it is attached.
Xsens1.set_translation(Vec3(0, -0.1, 0.07));     % Original location
Xsens1.set_orientation(Vec3(1.870796326794897, 0, 1.570796326794897)); % Rotate
Link1.addComponent(Xsens1);

% ----------------------Intermediate frame for Xsens2----------------------
% Get a handle to the second link from the loaded OpenSim model
Link2    = osimModel.getBodySet.get('link_2');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 2.
Xsens2Intermediate = PhysicalOffsetFrame(); 
Xsens2Intermediate.setName('IntermediateFrameForXsens2');
Xsens2Intermediate.setParentFrame(Link2); % Select the body to which it is attached.
Xsens2Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
Xsens2Intermediate.set_orientation(Vec3(-1.570796326794897, 0, 3.141592653589793));   % Rotate
Link2.addComponent(Xsens2Intermediate);


% ---------------------------------Xsens2----------------------------------
% Frame modeled as a PhysicalOffsetFrame to IntermediateFrameForXsens2
Xsens2 = PhysicalOffsetFrame(); 
Xsens2.setName('SecondXsensIMU');
Xsens2.setParentFrame(Xsens2Intermediate); % Select the body to which it is attached.
Xsens2.set_translation(Vec3(0, 0.095, 0.12)); % Original location
Xsens2.set_orientation(Vec3(-1.270796326794897, 0, -1.570796326794897));   % Rotate 0.3 + -1.5708 = -1.2708
Link2.addComponent(Xsens2);

% ---------------------------------Xsens3----------------------------------
% Get a handle to the third link from the loaded OpenSim model
Link3    = osimModel.getBodySet.get('link_3');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 3.
Xsens3 = PhysicalOffsetFrame(); 
Xsens3.setName('ThirdXsensIMU');
Xsens3.setParentFrame(Link3); % Select the body to which it is attached.
Xsens3.set_translation(Vec3(0, 0.095, 0.09)); % Original location
Xsens3.set_orientation(Vec3(-1.970796326794897, 0, -1.570796326794897));   % Rotate =  -1.5708 + -0.4
Link3.addComponent(Xsens3);


% ----------------------Intermediate frame for Xsens4----------------------
% Get a handle to the fourth link from the loaded OpenSim model
Link4    = osimModel.getBodySet.get('link_4');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 4.
Xsens4Intermediate = PhysicalOffsetFrame(); 
Xsens4Intermediate.setName('IntermediateFrameForXsens4');
Xsens4Intermediate.setParentFrame(Link4); % Select the body to which it is attached.
Xsens4Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
Xsens4Intermediate.set_orientation(Vec3(-1.570796326794897, 0, 0));   % Rotate
Link4.addComponent(Xsens4Intermediate);

% ---------------------------------Xsens4----------------------------------

% Frame modeled as a PhysicalOffsetFrame to IntermediateFrameForXsens4
Xsens4 = PhysicalOffsetFrame(); 
Xsens4.setName('FourthXsensIMU');
Xsens4.setParentFrame(Xsens4Intermediate); % Select the body to which it is attached.
Xsens4.set_translation(Vec3(0, -0.09, 0.13)); % Original location
Xsens4.set_orientation(Vec3(1.270796326794897, 0, 1.570796326794897));   % Rotate 1.5708 - 0.3 = 1.2708
Link4.addComponent(Xsens4);

% ----------------------Intermediate frame for Xsens5----------------------
% Get a handle to the fifth link from the loaded OpenSim model
Link5    = osimModel.getBodySet.get('link_5');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 5.
Xsens5Intermediate = PhysicalOffsetFrame(); 
Xsens5Intermediate.setName('IntermediateFrameForXsens5');
Xsens5Intermediate.setParentFrame(Link5); % Select the body to which it is attached.
Xsens5Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
Xsens5Intermediate.set_orientation(Vec3(0, 0, 3.141592653589793));
Link5.addComponent(Xsens5Intermediate);

% ---------------------------------Xsens5----------------------------------
% Frame modeled as a PhysicalOffsetFrame to IntermediateFrameForXsens5
Xsens5 = PhysicalOffsetFrame(); 
Xsens5.setName('FifthXsensIMU');
Xsens5.setParentFrame(Xsens5Intermediate); % Select the body to which it is attached.
Xsens5.set_translation(Vec3(0, -0.095, 0.11)); % Original location
Xsens5.set_orientation(Vec3(1.870796326794897, 0, 1.570796326794897));   % Rotate = 1.570796326794897 + 0.3 = 1.87 
Link5.addComponent(Xsens5);

% ----------------------Intermediate frame for Xsens6----------------------
% Get a handle to the sixth link from the loaded OpenSim model
Link6    = osimModel.getBodySet.get('link_6');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 6.
Xsens6Intermediate = PhysicalOffsetFrame(); 
Xsens6Intermediate.setName('IntermediateFrameForXsens6');
Xsens6Intermediate.setParentFrame(Link6); % Select the body to which it is attached.
Xsens6Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
Xsens6Intermediate.set_orientation(Vec3(-1.570796326794897, 0, 3.141592653589793));   % Rotate
Link6.addComponent(Xsens6Intermediate);

% Frame modeled as a PhysicalOffsetFrame to IntermediateFrameForXsens6.
Xsens6 = PhysicalOffsetFrame(); 
Xsens6.setName('SixthXsensIMU');
Xsens6.setParentFrame(Xsens6Intermediate); % Select the body to which it is attached.
Xsens6.set_translation(Vec3(0, 0.12, 0.07)); % Original location
Xsens6.set_orientation(Vec3(-1.420796326794897, 0, -1.570796326794897));   % Rotate 0.15 + -1.5708 = -1.4208 
Link6.addComponent(Xsens6);


%% Create struct for saving actual values of the simulated system
Results = struct;

Results.q1   = NaN(1,0);
Results.u1   = NaN(1,0);

Results.q1(:,end+1)   = Rotation0_1.getDefaultValue;
Results.u1(:,end+1)   = Rotation0_1.getDefaultSpeedValue;


Results.q2   = NaN(1,0);
Results.u2   = NaN(1,0);

Results.q2(:,end+1)   = Rotation1_2.getDefaultValue;
Results.u2(:,end+1)   = Rotation1_2.getDefaultSpeedValue;


Results.q3   = NaN(1,0);
Results.u3   = NaN(1,0);

Results.q3(:,end+1)   = Rotation2_3.getDefaultValue;
Results.u3(:,end+1)   = Rotation2_3.getDefaultSpeedValue;


Results.q4   = NaN(1,0);
Results.u4   = NaN(1,0);

Results.q4(:,end+1)   = Rotation3_4.getDefaultValue;
Results.u4(:,end+1)   = Rotation3_4.getDefaultSpeedValue;


Results.q5   = NaN(1,0);
Results.u5   = NaN(1,0);

Results.q5(:,end+1)   = Rotation4_5.getDefaultValue;
Results.u5(:,end+1)   = Rotation4_5.getDefaultSpeedValue;


Results.q6   = NaN(1,0);
Results.u6   = NaN(1,0);

Results.q6(:,end+1)   = Rotation5_6.getDefaultValue;
Results.u6(:,end+1)   = Rotation5_6.getDefaultSpeedValue;


% T will store the time indices 
Results.T = [];
Results.T(end+1) = 0; % This is time t = 0

%% Create "real" IMU
% This rSensor should represent the actual sensor which will be placed on
% the users body
rSensor = struct; % r = real sensor => real attached IMUs.

% Initializing struct variables for the * angular velocity
%                                       * linear acceleration
% --------------------------------Xsens IMU1-------------------------------
rSensor.Xsens1_AngVel             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens1_LinAcc             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens1_Gravity            = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens1_AngVel(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens1_LinAcc(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens1_Gravity(:,end+1)   = zeros(3,1); % At t=0 no measurement

% --------------------------------Xsens IMU2-------------------------------
rSensor.Xsens2_AngVel             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens2_LinAcc             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens2_Gravity            = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens2_AngVel(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens2_LinAcc(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens2_Gravity(:,end+1)   = zeros(3,1); % At t=0 no measurement

% --------------------------------Xsens IMU3-------------------------------
rSensor.Xsens3_AngVel             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens3_LinAcc             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens3_Gravity            = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens3_AngVel(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens3_LinAcc(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens3_Gravity(:,end+1)   = zeros(3,1); % At t=0 no measurement

% --------------------------------Xsens IMU4-------------------------------
rSensor.Xsens4_AngVel             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens4_LinAcc             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens4_Gravity            = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens4_AngVel(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens4_LinAcc(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens4_Gravity(:,end+1)   = zeros(3,1); % At t=0 no measurement

% --------------------------------Xsens IMU5-------------------------------
rSensor.Xsens5_AngVel             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens5_LinAcc             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens5_Gravity            = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens5_AngVel(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens5_LinAcc(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens5_Gravity(:,end+1)   = zeros(3,1); % At t=0 no measurement

% --------------------------------Xsens IMU6-------------------------------
rSensor.Xsens6_AngVel             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens6_LinAcc             = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens6_Gravity            = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens6_AngVel(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens6_LinAcc(:,end+1)    = zeros(3,1); % At t=0 no measurement
rSensor.Xsens6_Gravity(:,end+1)   = zeros(3,1); % At t=0 no measurement

%  T will store the time indices 
rSensor.T = [];
rSensor.T(end+1) = 0; % This is time t = 0

%% Create a TableReporterVec3() to save quantities to a file after simulating.
% This table can report SimTK::Vec3s, and thus can be used for reporting
% 3 dimensional positions, velocities, accelerations, etc.


reporter = TableReporterVec3();
reporter.setName('reporter');

% Set time interval for the reporter
reportTimeInterval = 1/SampleFreq;
reporter.set_report_time_interval(reportTimeInterval);

% -------------- Reporting variables of attached Xsens IMUs ---------------

% --------------------------------IMU1---------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(Xsens1.getOutput('linear_acceleration'), 'Xsens1_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens1.getOutput('angular_velocity'), 'Xsens1_ang_vel');

% --------------------------------IMU2---------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(Xsens2.getOutput('linear_acceleration'), 'Xsens2_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens2.getOutput('angular_velocity'), 'Xsens2_ang_vel');

% --------------------------------IMU3---------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(Xsens3.getOutput('linear_acceleration'), 'Xsens3_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens3.getOutput('angular_velocity'), 'Xsens3_ang_vel');

% --------------------------------IMU4---------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(Xsens4.getOutput('linear_acceleration'), 'Xsens4_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens4.getOutput('angular_velocity'), 'Xsens4_ang_vel');

% --------------------------------IMU5---------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(Xsens5.getOutput('linear_acceleration'), 'Xsens5_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens5.getOutput('angular_velocity'), 'Xsens5_ang_vel');

% --------------------------------IMU6---------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(Xsens6.getOutput('linear_acceleration'), 'Xsens6_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens6.getOutput('angular_velocity'), 'Xsens6_ang_vel');

% Add the reporter to the model
osimModel.addComponent(reporter);


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

%% Save the .osim model to a file to visualize in OpenSim
modelFilenameForOpenSim = 'Kukaiiwa7_CreatingMeasurementsWithIntermediateFrame.osim';
osimModel.print(modelFilenameForOpenSim);
disp('Kukaiiwa7_CreatingMeasurementsWithIntermediateFrame.osim printed!');

%% Simulation of the model and update states: Simulation Option B
dTime = reportTimeInterval; % Sample frequency of Xsens IMU
finalTime = 5;
n = round(finalTime/dTime);

% Manager: A class that manages the execution of a simulation.
manager = Manager(osimModel);
% This must be called before calling Manager::integrate() 
manager.initialize(state);

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
    rSensor.T(end+1) = state.getTime;
    Results.T(end+1) = state.getTime;
    
    % Again realize the acceleration stage to be able to compute Qdot and Udot correctly!
    osimModel.realizeAcceleration(state);
    
    % -----------------------Save to struct: Results-----------------------
    % Save the actual values of state q to the struct: Results
    q     = state.getQ;
    q1    = q.get(0);
    q2    = q.get(1);
    q3    = q.get(2);
    q4    = q.get(3);
    q5    = q.get(4);
    q6    = q.get(5);
    Results.q1(:,end+1) = q1;
    Results.q2(:,end+1) = q2;
    Results.q3(:,end+1) = q3;
    Results.q4(:,end+1) = q4;
    Results.q5(:,end+1) = q5;
    Results.q6(:,end+1) = q6;
    
    % Save the actual values of state u to the struct: Results
    u     = state.getU;
    u1    = u.get(0);
    u2    = u.get(1);
    u3    = u.get(2);
    u4    = u.get(3);
    u5    = u.get(4);
    u6    = u.get(5);
    Results.u1(:,end+1) = u1;
    Results.u2(:,end+1) = u2;
    Results.u3(:,end+1) = u3;
    Results.u4(:,end+1) = u4;
    Results.u5(:,end+1) = u5;
    Results.u6(:,end+1) = u6;
    
    % -------------------- End Save to struct: Results --------------------

    % -----------------------Save to struct: rSensor-----------------------       
    
    % --------------------------------IMU1---------------------------------
    % Get the angular velocity of Xsens IMU1 expressed in the Ground frame.
    ground_Xsens1_angvel = Xsens1.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens1angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens1_angvel, Xsens1);
    local_Xsens1AngVel = [local_Xsens1angvel.get(0) local_Xsens1angvel.get(1) local_Xsens1angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens1_AngVel(:,end+1) = local_Xsens1AngVel;
    
    % Determining the gravity vector expressed in Xsens IMU 1 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_Xsens1_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,Xsens1);
    Xsens1_Gravity         = [local_Xsens1_Gravity.get(0) local_Xsens1_Gravity.get(1) local_Xsens1_Gravity.get(2)]';
    rSensor.Xsens1_Gravity(:,end+1) = Xsens1_Gravity;
    
    % Get the linear acceleration of Xsens IMU1 expressed in the Ground frame.
    ground_Xsens1linacc = Xsens1.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens1linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens1linacc, Xsens1);
    local_Xsens1LinAcc = [local_Xsens1linacc.get(0) local_Xsens1linacc.get(1) local_Xsens1linacc.get(2)]';
    rSensor.Xsens1_LinAcc(:,end+1) = local_Xsens1LinAcc + Xsens1_Gravity;

    % --------------------------------IMU2---------------------------------
    % Get the angular velocity of Xsens IMU2 expressed in the Ground frame.
    ground_Xsens2_angvel = Xsens2.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens2angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens2_angvel, Xsens2);
    local_Xsens2AngVel = [local_Xsens2angvel.get(0) local_Xsens2angvel.get(1) local_Xsens2angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens2_AngVel(:,end+1) = local_Xsens2AngVel;
    
    % Determining the gravity vector expressed in Xsens IMU 2 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_Xsens2_Gravity = ground.expressVectorInAnotherFrame(state,GravityVec,Xsens2);
    Xsens2_Gravity         = [local_Xsens2_Gravity.get(0) local_Xsens2_Gravity.get(1) local_Xsens2_Gravity.get(2)]';
    rSensor.Xsens2_Gravity(:,end+1) = Xsens2_Gravity;
    
    % Get the linear acceleration of Xsens IMU2 expressed in the Ground frame.
    ground_Xsens2linacc = Xsens2.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens2linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens2linacc, Xsens2);
    local_Xsens2LinAcc = [local_Xsens2linacc.get(0) local_Xsens2linacc.get(1) local_Xsens2linacc.get(2)]';
    rSensor.Xsens2_LinAcc(:,end+1) = local_Xsens2LinAcc + Xsens2_Gravity;
    
    % --------------------------------IMU3---------------------------------
    % Get the angular velocity of Xsens IMU3 expressed in the Ground frame.
    ground_Xsens3_angvel = Xsens3.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU3: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens3angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens3_angvel, Xsens3);
    local_Xsens3AngVel = [local_Xsens3angvel.get(0) local_Xsens3angvel.get(1) local_Xsens3angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens3_AngVel(:,end+1) = local_Xsens3AngVel;
    
    % Determining the gravity vector expressed in Xsens IMU 3 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_Xsens3_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,Xsens3);
    Xsens3_Gravity         = [local_Xsens3_Gravity.get(0) local_Xsens3_Gravity.get(1) local_Xsens3_Gravity.get(2)]';
    rSensor.Xsens3_Gravity(:,end+1) = Xsens3_Gravity;
    
    % Get the linear acceleration of Xsens IMU3 expressed in the Ground frame.
    ground_Xsens3linacc = Xsens3.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU3: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens3linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens3linacc, Xsens3);
    local_Xsens3LinAcc = [local_Xsens3linacc.get(0) local_Xsens3linacc.get(1) local_Xsens3linacc.get(2)]';
    rSensor.Xsens3_LinAcc(:,end+1) = local_Xsens3LinAcc + Xsens3_Gravity;
    
    % --------------------------------IMU4---------------------------------
    % Get the angular velocity of Xsens IMU4 expressed in the Ground frame.
    ground_Xsens4_angvel = Xsens4.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU4: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens4angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens4_angvel, Xsens4);
    local_Xsens4AngVel = [local_Xsens4angvel.get(0) local_Xsens4angvel.get(1) local_Xsens4angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens4_AngVel(:,end+1) = local_Xsens4AngVel;
    
    % Determining the gravity vector expressed in Xsens IMU 4 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_Xsens4_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,Xsens4);
    Xsens4_Gravity         = [local_Xsens4_Gravity.get(0) local_Xsens4_Gravity.get(1) local_Xsens4_Gravity.get(2)]';
    rSensor.Xsens4_Gravity(:,end+1) = Xsens4_Gravity;
    
    % Get the linear acceleration of Xsens IMU4 expressed in the Ground frame.
    ground_Xsens4linacc = Xsens4.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU4: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens4linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens4linacc, Xsens4);
    local_Xsens4LinAcc = [local_Xsens4linacc.get(0) local_Xsens4linacc.get(1) local_Xsens4linacc.get(2)]';
    rSensor.Xsens4_LinAcc(:,end+1) = local_Xsens4LinAcc + Xsens4_Gravity;
    
    % --------------------------------IMU5---------------------------------
    % Get the angular velocity of Xsens IMU5 expressed in the Ground frame.
    ground_Xsens5_angvel = Xsens5.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU5: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens5angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens5_angvel, Xsens5);
    local_Xsens5AngVel = [local_Xsens5angvel.get(0) local_Xsens5angvel.get(1) local_Xsens5angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens5_AngVel(:,end+1) = local_Xsens5AngVel;
    
    % Determining the gravity vector expressed in Xsens IMU 5 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_Xsens5_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,Xsens5);
    Xsens5_Gravity         = [local_Xsens5_Gravity.get(0) local_Xsens5_Gravity.get(1) local_Xsens5_Gravity.get(2)]';
    rSensor.Xsens5_Gravity(:,end+1) = Xsens5_Gravity;
    
    % Get the linear acceleration of Xsens IMU5 expressed in the Ground frame.
    ground_Xsens5linacc = Xsens5.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU5: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens5linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens5linacc, Xsens5);
    local_Xsens5LinAcc = [local_Xsens5linacc.get(0) local_Xsens5linacc.get(1) local_Xsens5linacc.get(2)]';
    rSensor.Xsens5_LinAcc(:,end+1) = local_Xsens5LinAcc + Xsens5_Gravity;

    % --------------------------------IMU6---------------------------------
    % Get the angular velocity of Xsens IMU6 expressed in the Ground frame.
    ground_Xsens6_angvel = Xsens6.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU6: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens6angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens6_angvel, Xsens6);
    local_Xsens6AngVel = [local_Xsens6angvel.get(0) local_Xsens6angvel.get(1) local_Xsens6angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens6_AngVel(:,end+1) = local_Xsens6AngVel;
    
    % Determining the gravity vector expressed in Xsens IMU 6 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_Xsens6_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,Xsens6);
    Xsens6_Gravity         = [local_Xsens6_Gravity.get(0) local_Xsens6_Gravity.get(1) local_Xsens6_Gravity.get(2)]';
    rSensor.Xsens6_Gravity(:,end+1) = Xsens6_Gravity;
    
    % Get the linear acceleration of Xsens IMU6 expressed in the Ground frame.
    ground_Xsens6linacc = Xsens6.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU6: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens6linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens6linacc, Xsens6);
    local_Xsens6LinAcc = [local_Xsens6linacc.get(0) local_Xsens6linacc.get(1) local_Xsens6linacc.get(2)]';
    rSensor.Xsens6_LinAcc(:,end+1) = local_Xsens6LinAcc + Xsens6_Gravity;    
    
    % -------------------- End Save to struct: rSensor --------------------
end

%% Plot the state values 'q' of the measurement system
figure,
plot(Results.T, Results.q1(1,:), Results.T, Results.q2(1,:), Results.T, Results.q3(1,:), Results.T, Results.q4(1,:), Results.T, Results.q5(1,:), Results.T, Results.q6(1,:))
title('State values q')
legend('q1','q2','q3','q4','q5','q6')

%% Plot the state values 'u' of the measurement system
figure,
plot(Results.T, Results.u1(1,:), Results.T, Results.u2(1,:), Results.T, Results.u3(1,:), Results.T, Results.u4(1,:), Results.T, Results.u5(1,:), Results.T, Results.u6(1,:))
title('State values u')
legend('u1','u2','u3','u4','u5','u6')

%% Add zero-mean white noise to the obtained measurements
load('Covariances.mat'); 

MU = zeros(3,1);
% IMU 1
CovGyro1 = Covariances.IMU1CovGyro;
CovAcc1  = Covariances.IMU1CovAcc;

% IMU 2
CovGyro2 = Covariances.IMU2CovGyro;
CovAcc2  = Covariances.IMU2CovAcc;

% IMU 3
CovGyro3 = Covariances.IMU1CovGyro; % Take the covariance of IMU 1 as only 2 IMUs were determined
CovAcc3  = Covariances.IMU1CovAcc;

% IMU 4
CovGyro4 = Covariances.IMU2CovGyro; % Take the covariance of IMU 2 as only 2 IMUs were determined
CovAcc4  = Covariances.IMU2CovAcc;

% IMU 5
CovGyro5 = Covariances.IMU1CovGyro; % Take the covariance of IMU 1 as only 2 IMUs were determined
CovAcc5  = Covariances.IMU1CovAcc;

% IMU 6
CovGyro6 = Covariances.IMU2CovGyro; % Take the covariance of IMU 2 as only 2 IMUs were determined
CovAcc6  = Covariances.IMU2CovAcc;

% --------------------------------Xsens IMU1-------------------------------
% Add zero-mean white noise to the angular velocity measurements
[numRowsGyroIMU1, numColsGyroIMU1] = size(rSensor.Xsens1_AngVel(1,:));
             
NoiseGyro1 = mvnrnd(MU,CovGyro1,numColsGyroIMU1);
NoiseGyro1 = NoiseGyro1'; %Transpose such that dimensions match to be able to add noise
% Create noisy signals below original measurements
rSensor.Xsens1_AngVel(4,:) = rSensor.Xsens1_AngVel(1,:) + NoiseGyro1(1,:); % Rotation around x-axis
rSensor.Xsens1_AngVel(5,:) = rSensor.Xsens1_AngVel(2,:) + NoiseGyro1(2,:); % Rotation around y-axis
rSensor.Xsens1_AngVel(6,:) = rSensor.Xsens1_AngVel(3,:) + NoiseGyro1(3,:); % Rotation around z-axis

% Add zero-mean white noise to the linear acceleration measurements
[numRowsAccIMU1, numColsAccIMU1] = size(rSensor.Xsens1_LinAcc(1,:));
NoiseAcc1 = mvnrnd(MU,CovAcc1,numColsAccIMU1);
NoiseAcc1 = NoiseAcc1'; %Transpose such that dimensions match to be able to add noise

% Create noisy signals below original measurements
rSensor.Xsens1_LinAcc(4,:) = rSensor.Xsens1_LinAcc(1,:) + NoiseAcc1(1,:); % x-axis
rSensor.Xsens1_LinAcc(5,:) = rSensor.Xsens1_LinAcc(2,:) + NoiseAcc1(2,:); % y-axis
rSensor.Xsens1_LinAcc(6,:) = rSensor.Xsens1_LinAcc(3,:) + NoiseAcc1(3,:); % z-axis


% --------------------------------Xsens IMU2-------------------------------
% Add zero-mean white noise to the angular velocity measurements
[numRowsGyroIMU2, numColsGyroIMU2] = size(rSensor.Xsens2_AngVel(1,:));
             
NoiseGyro2 = mvnrnd(MU,CovGyro2,numColsGyroIMU2);
NoiseGyro2 = NoiseGyro2'; %Transpose such that dimensions match to be able to add noise
% Create noisy signals below original measurements
rSensor.Xsens2_AngVel(4,:) = rSensor.Xsens2_AngVel(1,:) + NoiseGyro2(1,:); % Rotation around x-axis
rSensor.Xsens2_AngVel(5,:) = rSensor.Xsens2_AngVel(2,:) + NoiseGyro2(2,:); % Rotation around y-axis
rSensor.Xsens2_AngVel(6,:) = rSensor.Xsens2_AngVel(3,:) + NoiseGyro2(3,:); % Rotation around z-axis

% Add zero-mean white noise to the linear acceleration measurements
[numRowsAccIMU2, numColsAccIMU2] = size(rSensor.Xsens2_LinAcc(1,:));
NoiseAcc2 = mvnrnd(MU,CovAcc2,numColsAccIMU2);
NoiseAcc2 = NoiseAcc2'; %Transpose such that dimensions match to be able to add noise

% Create noisy signals below original measurements
rSensor.Xsens2_LinAcc(4,:) = rSensor.Xsens2_LinAcc(1,:) + NoiseAcc2(1,:); % x-axis
rSensor.Xsens2_LinAcc(5,:) = rSensor.Xsens2_LinAcc(2,:) + NoiseAcc2(2,:); % y-axis
rSensor.Xsens2_LinAcc(6,:) = rSensor.Xsens2_LinAcc(3,:) + NoiseAcc2(3,:); % z-axis

% --------------------------------Xsens IMU3-------------------------------
% Add zero-mean white noise to the angular velocity measurements
[numRowsGyroIMU3, numColsGyroIMU3] = size(rSensor.Xsens3_AngVel(1,:));
             
NoiseGyro3 = mvnrnd(MU,CovGyro3,numColsGyroIMU3);
NoiseGyro3 = NoiseGyro3'; %Transpose such that dimensions match to be able to add noise
% Create noisy signals below original measurements
rSensor.Xsens3_AngVel(4,:) = rSensor.Xsens3_AngVel(1,:) + NoiseGyro3(1,:); % Rotation around x-axis
rSensor.Xsens3_AngVel(5,:) = rSensor.Xsens3_AngVel(2,:) + NoiseGyro3(2,:); % Rotation around y-axis
rSensor.Xsens3_AngVel(6,:) = rSensor.Xsens3_AngVel(3,:) + NoiseGyro3(3,:); % Rotation around z-axis

% Add zero-mean white noise to the linear acceleration measurements
[numRowsAccIMU3, numColsAccIMU3] = size(rSensor.Xsens3_LinAcc(1,:));
NoiseAcc3 = mvnrnd(MU,CovAcc3,numColsAccIMU3);
NoiseAcc3 = NoiseAcc3'; %Transpose such that dimensions match to be able to add noise

% Create noisy signals below original measurements
rSensor.Xsens3_LinAcc(4,:) = rSensor.Xsens3_LinAcc(1,:) + NoiseAcc3(1,:); % x-axis
rSensor.Xsens3_LinAcc(5,:) = rSensor.Xsens3_LinAcc(2,:) + NoiseAcc3(2,:); % y-axis
rSensor.Xsens3_LinAcc(6,:) = rSensor.Xsens3_LinAcc(3,:) + NoiseAcc3(3,:); % z-axis

% --------------------------------Xsens IMU4-------------------------------
% Add zero-mean white noise to the angular velocity measurements
[numRowsGyroIMU4, numColsGyroIMU4] = size(rSensor.Xsens4_AngVel(1,:));
             
NoiseGyro4 = mvnrnd(MU,CovGyro4,numColsGyroIMU4);
NoiseGyro4 = NoiseGyro4'; %Transpose such that dimensions match to be able to add noise
% Create noisy signals below original measurements
rSensor.Xsens4_AngVel(4,:) = rSensor.Xsens4_AngVel(1,:) + NoiseGyro4(1,:); % Rotation around x-axis
rSensor.Xsens4_AngVel(5,:) = rSensor.Xsens4_AngVel(2,:) + NoiseGyro4(2,:); % Rotation around y-axis
rSensor.Xsens4_AngVel(6,:) = rSensor.Xsens4_AngVel(3,:) + NoiseGyro4(3,:); % Rotation around z-axis

% Add zero-mean white noise to the linear acceleration measurements
[numRowsAccIMU4, numColsAccIMU4] = size(rSensor.Xsens4_LinAcc(1,:));
NoiseAcc4 = mvnrnd(MU,CovAcc4,numColsAccIMU4);
NoiseAcc4 = NoiseAcc4'; %Transpose such that dimensions match to be able to add noise

% Create noisy signals below original measurements
rSensor.Xsens4_LinAcc(4,:) = rSensor.Xsens4_LinAcc(1,:) + NoiseAcc4(1,:); % x-axis
rSensor.Xsens4_LinAcc(5,:) = rSensor.Xsens4_LinAcc(2,:) + NoiseAcc4(2,:); % y-axis
rSensor.Xsens4_LinAcc(6,:) = rSensor.Xsens4_LinAcc(3,:) + NoiseAcc4(3,:); % z-axis

% --------------------------------Xsens IMU5-------------------------------
% Add zero-mean white noise to the angular velocity measurements
[numRowsGyroIMU5, numColsGyroIMU5] = size(rSensor.Xsens5_AngVel(1,:));
             
NoiseGyro5 = mvnrnd(MU,CovGyro5,numColsGyroIMU5);
NoiseGyro5 = NoiseGyro5'; %Transpose such that dimensions match to be able to add noise
% Create noisy signals below original measurements
rSensor.Xsens5_AngVel(4,:) = rSensor.Xsens5_AngVel(1,:) + NoiseGyro5(1,:); % Rotation around x-axis
rSensor.Xsens5_AngVel(5,:) = rSensor.Xsens5_AngVel(2,:) + NoiseGyro5(2,:); % Rotation around y-axis
rSensor.Xsens5_AngVel(6,:) = rSensor.Xsens5_AngVel(3,:) + NoiseGyro5(3,:); % Rotation around z-axis

% Add zero-mean white noise to the linear acceleration measurements
[numRowsAccIMU5, numColsAccIMU5] = size(rSensor.Xsens5_LinAcc(1,:));
NoiseAcc5 = mvnrnd(MU,CovAcc5,numColsAccIMU5);
NoiseAcc5 = NoiseAcc5'; %Transpose such that dimensions match to be able to add noise

% Create noisy signals below original measurements
rSensor.Xsens5_LinAcc(4,:) = rSensor.Xsens5_LinAcc(1,:) + NoiseAcc5(1,:); % x-axis
rSensor.Xsens5_LinAcc(5,:) = rSensor.Xsens5_LinAcc(2,:) + NoiseAcc5(2,:); % y-axis
rSensor.Xsens5_LinAcc(6,:) = rSensor.Xsens5_LinAcc(3,:) + NoiseAcc5(3,:); % z-axis


% --------------------------------Xsens IMU6-------------------------------
% Add zero-mean white noise to the angular velocity measurements
[numRowsGyroIMU6, numColsGyroIMU6] = size(rSensor.Xsens6_AngVel(1,:));
             
NoiseGyro6 = mvnrnd(MU,CovGyro6,numColsGyroIMU6);
NoiseGyro6 = NoiseGyro6'; %Transpose such that dimensions match to be able to add noise
% Create noisy signals below original measurements
rSensor.Xsens6_AngVel(4,:) = rSensor.Xsens6_AngVel(1,:) + NoiseGyro6(1,:); % Rotation around x-axis
rSensor.Xsens6_AngVel(5,:) = rSensor.Xsens6_AngVel(2,:) + NoiseGyro6(2,:); % Rotation around y-axis
rSensor.Xsens6_AngVel(6,:) = rSensor.Xsens6_AngVel(3,:) + NoiseGyro6(3,:); % Rotation around z-axis

% Add zero-mean white noise to the linear acceleration measurements
[numRowsAccIMU6, numColsAccIMU6] = size(rSensor.Xsens6_LinAcc(1,:));
NoiseAcc6 = mvnrnd(MU,CovAcc6,numColsAccIMU6);
NoiseAcc6 = NoiseAcc6'; %Transpose such that dimensions match to be able to add noise

% Create noisy signals below original measurements
rSensor.Xsens6_LinAcc(4,:) = rSensor.Xsens6_LinAcc(1,:) + NoiseAcc6(1,:); % x-axis
rSensor.Xsens6_LinAcc(5,:) = rSensor.Xsens6_LinAcc(2,:) + NoiseAcc6(2,:); % y-axis
rSensor.Xsens6_LinAcc(6,:) = rSensor.Xsens6_LinAcc(3,:) + NoiseAcc6(3,:); % z-axis

%% Save measured variables in struct rSensor to .mat file
% To save the variables in the struct rSensor.
save('Measurements_Kuka_iiwa_7.mat', 'rSensor')
% When loaded using: load('Measurements_Kuka_iiwa_7.mat'), they are loaded as a struct

%% Save state values in struct Results to .mat file
% To save the variables in the struct Results.
save('States_Kuka_iiwa_7.mat', 'Results')
% When loaded using: load('States_Kuka_iiwa_7.mat'), they are loaded as a struct

%% Stop the timer
toc