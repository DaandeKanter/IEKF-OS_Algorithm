%% The Movement Reconstruction Framework based on raw IMU data and OpenSim.
%__________________________________________________________________________
% Script created by Daan de Kanter for MSc Thesis project.                
% Project is supervised by Dr. M. Kok, Delft University of Technology     
%                          Dr. Ir. A. Seth, Delft University of Technology
%__________________________________________________________________________

% Clear Workspace
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

%% Do you want to write the IEKF outputs to the command window? Used for checking
fprintf('\n');
prompt = 'Do you want to write the IEKF outputs to the command window? [y]es/[n]o: ';
str2 = input(prompt, 's');
if str2 == 'y'
    fprintf('IEKF outputs written to the command window!\n');
elseif str2 == 'n'
    fprintf('No IEKF outputs written to the command window!\n');
end

%% Create an coordinate actuator for the Revolute0_1 joint
% Get the number of coordinates and a handle to the coordinate set
coordSet = osimModel.getCoordinateSet();

% Get a reference to the first coordinate
Rotation0_1 = coordSet.get(0);

optimalForce = 1;      % Set the optimal force. The maximum torque produced by this actuator when fully activated.
                       % Ajay told me this is essentially a gain which
                       % multiplies the user-set value for tau in the
                       % component brain.PrescribedController()
   
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
% Get a reference to the second coordinate
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
% Get a reference to the second coordinate
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
% Get a reference to the fourth coordinate
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

% Load the initial joint angles
load('q_init.mat')
PerturbValue = deg2rad(5); % Added to the initial states to see if the IEKF-OS converges

% -------------------------- Rotation0_1 Actuator -------------------------
% Add a controller that specifies the excitation of Rotation0_1Actuator
brain1 = PrescribedController();
brain1.setName('Brain1')
brain1.addActuator(Rotation0_1Actuator);

% Initial conditions for link1 for the state
angle1_rad   = q_init(1,1); % Define generalized coordinate in [rad]
angle1_rad_s = deg2rad(0);  % Define generalized velocity in [rad/s]
tau1         = 0;           % Define joint torque in N*m. 

% Setting of initial conditions
% Setting q and u
Rotation0_1.setDefaultValue(angle1_rad);
Rotation0_1.setDefaultSpeedValue(angle1_rad_s);
% Setting tau
torque1Gen = Constant(tau1); % Controller: (brain) requires input as a function: Constant
% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain1.prescribeControlForActuator('Rotation0_1', torque1Gen);
osimModel.addController(brain1);


% -------------------------- Rotation1_2 Actuator -------------------------
% Add a controller that specifies the excitation of Rotation1_2Actuator
brain2 = PrescribedController();
brain2.setName('Brain2')
brain2.addActuator(Rotation1_2Actuator);

% Initial conditions for link2 for the state
angle2_rad   = q_init(1,2)+PerturbValue; % Define generalized coordinate in [rad]
angle2_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
tau2         = 0;          % Define joint torque in N*m.

% Setting of initial conditions
% Setting q and u
Rotation1_2.setDefaultValue(angle2_rad);
Rotation1_2.setDefaultSpeedValue(angle2_rad_s);
% Setting tau
torque2Gen = Constant(tau2); % Controller: (brain) requires input as a function: Constant
% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain2.prescribeControlForActuator('Rotation1_2', torque2Gen);
osimModel.addController(brain2);

% -------------------------- Rotation2_3 Actuator -------------------------
% Add a controller that specifies the excitation of Rotation2_3Actuator
brain3 = PrescribedController();
brain3.setName('Brain3')
brain3.addActuator(Rotation2_3Actuator);

% Initial conditions for link3 for the state
angle3_rad   = q_init(1,3)+PerturbValue; % Define generalized coordinate in [rad]
angle3_rad_s = deg2rad(0);  % Define generalized velocity in [rad/s]
tau3         = 0;           % Define joint torque in N*m.

% Setting of initial conditions
% Setting q and u
Rotation2_3.setDefaultValue(angle3_rad);
Rotation2_3.setDefaultSpeedValue(angle3_rad_s);
% Setting tau
torque3Gen = Constant(tau3); % Controller: (brain) requires input as a function: Constant

% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain3.prescribeControlForActuator('Rotation2_3', torque3Gen);
osimModel.addController(brain3);

% -------------------------- Rotation3_4 Actuator -------------------------
% Add a controller that specifies the excitation of Rotation3_4Actuator
brain4 = PrescribedController();
brain4.setName('Brain4')
brain4.addActuator(Rotation3_4Actuator);

% Initial conditions for link4 for the state
angle4_rad   = q_init(1,4)+PerturbValue; % Define generalized coordinate in [rad]
angle4_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
tau4         = 0;          % Define joint torque in N*m.

% Setting of initial conditions
% Setting q and u
Rotation3_4.setDefaultValue(angle4_rad);
Rotation3_4.setDefaultSpeedValue(angle4_rad_s);
% Setting tau
torque4Gen = Constant(tau4); % Controller: (brain) requires input as a function: Constant

% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain4.prescribeControlForActuator('Rotation3_4', torque4Gen);
osimModel.addController(brain4);


% -------------------------- Rotation4_5 Actuator -------------------------
% Add a controller that specifies the excitation of Rotation4_5Actuator
brain5 = PrescribedController();
brain5.setName('Brain5')
brain5.addActuator(Rotation4_5Actuator);

% Initial conditions for link5 for the state
angle5_rad   = q_init(1,5)+PerturbValue; % Define generalized coordinate in [rad]
angle5_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
tau5         = 0;          % Define joint torque in N*m.

% Setting of initial conditions
% Setting q and u
Rotation4_5.setDefaultValue(angle5_rad);
Rotation4_5.setDefaultSpeedValue(angle5_rad_s);
% Setting tau
torque5Gen = Constant(tau5); % Controller: (brain) requires input as a function: Constant

% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain5.prescribeControlForActuator('Rotation4_5', torque5Gen);
osimModel.addController(brain5);

% -------------------------- Rotation5_6 Actuator -------------------------
% Add a controller that specifies the excitation of Rotation5_6Actuator
brain6 = PrescribedController();
brain6.setName('Brain6')
brain6.addActuator(Rotation5_6Actuator);

% Initial conditions for link6 for the state
angle6_rad   = q_init(1,6)+PerturbValue; % Define generalized coordinate in [rad]
angle6_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
tau6         = 0;          % Define joint torque in N*m.

% Setting of initial conditions
% Setting q and u
Rotation5_6.setDefaultValue(angle6_rad);
Rotation5_6.setDefaultSpeedValue(angle6_rad_s);
% Setting tau
torque6Gen = Constant(tau6); % Controller: (brain) requires input as a function: Constant

% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain6.prescribeControlForActuator('Rotation5_6', torque6Gen);
osimModel.addController(brain6);

% Set initial values of generalized coordinates (q), speeds (u) and control input (tau)
q_init   = [Rotation0_1.getDefaultValue;
            Rotation1_2.getDefaultValue;
            Rotation2_3.getDefaultValue;
            Rotation3_4.getDefaultValue;
            Rotation4_5.getDefaultValue;
            Rotation5_6.getDefaultValue];
u_init   = [Rotation0_1.getDefaultSpeedValue;
            Rotation1_2.getDefaultSpeedValue;
            Rotation2_3.getDefaultSpeedValue;
            Rotation3_4.getDefaultSpeedValue;
            Rotation4_5.getDefaultSpeedValue;
            Rotation5_6.getDefaultSpeedValue];
tau_init = [torque1Gen.getValue;
            torque2Gen.getValue;
            torque3Gen.getValue;
            torque4Gen.getValue;
            torque5Gen.getValue;
            torque6Gen.getValue];
     
% Load the determined covariance matrices
load('Covariances.mat'); 

% IMU 1
CovGyro1 = Covariances.IMU1CovGyro;
CovAcc1  = Covariances.IMU1CovAcc;

% IMU 2
CovGyro2 = Covariances.IMU2CovGyro;
CovAcc2  = Covariances.IMU2CovAcc;

% IMU 3
CovGyro3 = Covariances.IMU1CovGyro; % The covariances of IMU1 are taken as the covariances of only 2 IMUs have been computed
CovAcc3  = Covariances.IMU1CovAcc;

% IMU 4
CovGyro4 = Covariances.IMU2CovGyro; % The covariances of IMU2 are taken as the covariances of only 2 IMUs have been computed
CovAcc4  = Covariances.IMU2CovAcc;

% IMU 5
CovGyro5 = Covariances.IMU1CovGyro; % The covariances of IMU1 are taken as the covariances of only 2 IMUs have been computed
CovAcc5  = Covariances.IMU1CovAcc;

% IMU 6
CovGyro6 = Covariances.IMU2CovGyro; % The covariances of IMU2 are taken as the covariances of only 2 IMUs have been computed
CovAcc6  = Covariances.IMU2CovAcc;


%% Create frames that will represent "virtual" IMUs
% Get a handle to the first link from the loaded OpenSim model
Link1    = osimModel.getBodySet.get('link_1');

% ----------------------------------IMU1-----------------------------------
% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 1.
IMU1 = PhysicalOffsetFrame(); 
IMU1.setName('FirstIMU');
IMU1.setParentFrame(Link1); % Select the body to which it is attached.
IMU1.set_translation(Vec3(0, -0.1, 0.07)); % Translate
IMU1.set_orientation(Vec3(1.870796326794897, 0, 1.570796326794897));           % Rotate
Link1.addComponent(IMU1);

% -----------------------Intermediate frame for IMU2-----------------------
% Get a handle to the second link from the loaded OpenSim model
Link2    = osimModel.getBodySet.get('link_2');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 2.
IMU2Intermediate = PhysicalOffsetFrame(); 
IMU2Intermediate.setName('IntermediateFrameForIMU2');
IMU2Intermediate.setParentFrame(Link2); % Select the body to which it is attached.
IMU2Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
IMU2Intermediate.set_orientation(Vec3(-1.570796326794897, 0, 3.141592653589793));   % Rotate
Link2.addComponent(IMU2Intermediate);

% ----------------------------------IMU2-----------------------------------
% Frame modeled as a PhysicalOffsetFrame to IntermediateFrameForIMU2.
IMU2 = PhysicalOffsetFrame(); 
IMU2.setName('SecondIMU');
IMU2.setParentFrame(IMU2Intermediate); % Select the body to which it is attached.
IMU2.set_translation(Vec3(0, 0.095, 0.12)); % Translate
IMU2.set_orientation(Vec3(-1.270796326794897, 0, -1.570796326794897)); % Rotate 0.3 + -1.5708 = -1.2708
Link2.addComponent(IMU2);


% ----------------------------------IMU3-----------------------------------
% Get a handle to the third link from the loaded OpenSim model
Link3    = osimModel.getBodySet.get('link_3');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 3.
IMU3 = PhysicalOffsetFrame(); 
IMU3.setName('ThirdIMU');
IMU3.setParentFrame(Link3); % Select the body to which it is attached.
IMU3.set_translation(Vec3(0, 0.095, 0.09)); % Translate
IMU3.set_orientation(Vec3(-1.970796326794897, 0, -1.570796326794897));   % Rotate =  -1.5708 + -0.4
Link3.addComponent(IMU3);

% -----------------------Intermediate frame for IMU4-----------------------
% Get a handle to the fourth link from the loaded OpenSim model
Link4    = osimModel.getBodySet.get('link_4');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 4.
IMU4Intermediate = PhysicalOffsetFrame(); 
IMU4Intermediate.setName('IntermediateFrameForIMU4');
IMU4Intermediate.setParentFrame(Link4); % Select the body to which it is attached.
IMU4Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
IMU4Intermediate.set_orientation(Vec3(-1.570796326794897, 0, 0));   % Rotate
Link4.addComponent(IMU4Intermediate);

% ----------------------------------IMU4-----------------------------------
% Frame modeled as a PhysicalOffsetFrame to IntermediateFrameForIMU4.
IMU4 = PhysicalOffsetFrame(); 
IMU4.setName('FourthIMU');
IMU4.setParentFrame(IMU4Intermediate); % Select the body to which it is attached.
IMU4.set_translation(Vec3(0, -0.09, 0.13)); % Translate
IMU4.set_orientation(Vec3(1.270796326794897, 0, 1.570796326794897));  % Rotate 1.5708 - 0.3 = 1.2708
Link4.addComponent(IMU4);

% -----------------------Intermediate frame for IMU5-----------------------
% Get a handle to the fifth link from the loaded OpenSim model
Link5    = osimModel.getBodySet.get('link_5');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 5.
IMU5Intermediate = PhysicalOffsetFrame(); 
IMU5Intermediate.setName('IntermediateFrameForIMU5');
IMU5Intermediate.setParentFrame(Link5); % Select the body to which it is attached.
IMU5Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
IMU5Intermediate.set_orientation(Vec3(0, 0, 3.141592653589793));
Link5.addComponent(IMU5Intermediate);

% ----------------------------------IMU5-----------------------------------
% Frame modeled as a PhysicalOffsetFrame.
IMU5 = PhysicalOffsetFrame(); 
IMU5.setName('FifthIMU');
IMU5.setParentFrame(IMU5Intermediate); % Select the body to which it is attached.
IMU5.set_translation(Vec3(0, -0.095, 0.11)); % Translate
IMU5.set_orientation(Vec3(1.870796326794897, 0, 1.570796326794897));   % Rotate = -0.3 + -1.570796326794897
Link5.addComponent(IMU5);

% ----------------------Intermediate frame for IMU6----------------------
% Get a handle to the sixth link from the loaded OpenSim model
Link6    = osimModel.getBodySet.get('link_6');

% Frame modeled as a PhysicalOffsetFrame to joint center frame of link 6.
IMU6Intermediate = PhysicalOffsetFrame(); 
IMU6Intermediate.setName('IntermediateFrameForIMU6');
IMU6Intermediate.setParentFrame(Link6); % Select the body to which it is attached.
IMU6Intermediate.set_translation(Vec3(0, 0, 0)); % Original location
IMU6Intermediate.set_orientation(Vec3(-1.570796326794897, 0, 3.141592653589793));   % Rotate
Link6.addComponent(IMU6Intermediate);

% ----------------------------------IMU6-----------------------------------
% Frame modeled as a PhysicalOffsetFrame to IntermediateFrameForIMU6.
IMU6 = PhysicalOffsetFrame(); 
IMU6.setName('SixthIMU');
IMU6.setParentFrame(IMU6Intermediate); % Select the body to which it is attached.
IMU6.set_translation(Vec3(0, 0.12, 0.07)); % Translate
IMU6.set_orientation(Vec3(-1.420796326794897, 0, -1.570796326794897));   % Rotate 0.15 + -1.5708 = -1.4208 
Link6.addComponent(IMU6);

%% Initialize the Iterated Extended Kalman Filter: IEKF
[IEKF, vSensor] = IEKF_init(q_init, u_init, tau_init,...
                            CovGyro1, CovAcc1,...
                            CovGyro2, CovAcc2,...
                            CovGyro3, CovAcc3,...
                            CovGyro4, CovAcc4,...
                            CovGyro5, CovAcc5,...
                            CovGyro6, CovAcc6);

%% Create a TableReporterVec3() to save quantities to a file during simulating.
% This table can report SimTK::Vec3s, and thus can be used for reporting
% 3 dimensional positions, velocities, accelerations, etc.

SamplingFrequency = 100;  % Sampling frequency of Xsens MTw IMU
reportTimeInterval = 1/SamplingFrequency; 

%-------------------Reporting variables of attached IMU1-------------------
reporter = TableReporterVec3();
reporter.setName('reporter');

% Set time interval for the reporter
reporter.set_report_time_interval(reportTimeInterval);

% --------------------------------- IMU1 ----------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(IMU1.getOutput('linear_acceleration'), 'IMU1_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(IMU1.getOutput('angular_velocity'), 'IMU1_ang_vel');

% --------------------------------- IMU2 ----------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(IMU2.getOutput('linear_acceleration'), 'IMU2_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(IMU2.getOutput('angular_velocity'), 'IMU2_ang_vel');

% --------------------------------- IMU3 ----------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(IMU3.getOutput('linear_acceleration'), 'IMU3_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(IMU3.getOutput('angular_velocity'), 'IMU3_ang_vel');

% --------------------------------- IMU4 ----------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(IMU4.getOutput('linear_acceleration'), 'IMU4_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(IMU4.getOutput('angular_velocity'), 'IMU4_ang_vel');

% --------------------------------- IMU5 ----------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(IMU5.getOutput('linear_acceleration'), 'IMU5_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(IMU5.getOutput('angular_velocity'), 'IMU5_ang_vel');

% --------------------------------- IMU6 ----------------------------------
% Report the linear acceleration of the origin of the IMU.
reporter.addToReport(IMU6.getOutput('linear_acceleration'), 'IMU6_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(IMU6.getOutput('angular_velocity'), 'IMU6_ang_vel');

% Add the reporter to the model
osimModel.addComponent(reporter);

%% Initialize the System (checks model consistency)
% initSystem() does the following:   * Generate the system of equations
%                                    * Assemble to satisfy and constraints
%                                    * Returns the initial state
% If no intial state is specified, the state will just be set to 0.
state = osimModel.initSystem();

%% Visualize the simulation time in the visualizer
if str1 == 'y'
    sviz = osimModel.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
end

%% Save the .osim model to a file to visualize in OpenSim
modelFilenameForOpenSim = 'Kukaiiwa7_IEKF.osim';
osimModel.print(modelFilenameForOpenSim);
fprintf('\n');
disp('Kukaiiwa7_IEKF.osim printed!');

%% Load state values from KukaLink6CreatingMeasurements.m
fprintf('\n');
disp('Loading the actual state data:')
load('States_Kuka_iiwa_7.mat') % Actual state values loaded
disp('Done!')
fprintf('\n');

%% Load measurements from KukaLink6CreatingMeasurements.m
disp('Loading the measurement data:')
load('Measurements_Kuka_iiwa_7.mat') % Measurements loaded
disp('Done!')
fprintf('\n');

%% Start the timer
tic

%% Simulation of the model and update states
% ________________________________________________________________________
% |Manager outside of function due to otherwise MATLAB internal error.   |
% |Hence, prediction step split in 3 parts: Part 1 : Set prior states    |
% |                                         Manager: Integrating EoM     |
% |                                         Part 2 : Get predicted states|
% ------------------------------------------------------------------------
    
IEKF.dTime = reportTimeInterval; % Sample frequency of Xsens IMU
IEKF.finalTime = 5;
IEKF.n = round(IEKF.finalTime/IEKF.dTime);
IEKF.NoI = 3; % Number of Iterations (NoI) of the IEKF update step

% Manager: A class that manages the execution of a simulation.
manager = Manager(osimModel);
% This must be called before calling Manager::integrate() 
manager.initialize(state);

for t = 1:IEKF.n
    
    %______________________________________________________________________
    % IEKF Step 1 - Part 1
    % Getting the prior updated states and setting current states accordingly
    IEKF = IEKF_predict_step_part_1(IEKF, osimModel, state, torque1Gen, torque2Gen, torque3Gen, torque4Gen, torque5Gen, torque6Gen, t, str2);
    
    % Set the control torque / joint torque tau, back to its original value
    torque1Gen.setValue(IEKF.tau_prev(1,1));
    if str2 == 'y'
        disp('Joint torque tau1 should be set to its previous IEKF updated value: ');
        disp(torque1Gen.getValue)
    end
    
    torque2Gen.setValue(IEKF.tau_prev(2,1));
    if str2 == 'y'
        disp('Joint torque tau2 should be set to its previous IEKF updated value: ');
        disp(torque2Gen.getValue)
    end

    torque3Gen.setValue(IEKF.tau_prev(3,1));
    if str2 == 'y'
        disp('Joint torque tau3 should be set to its previous IEKF updated value: ');
        disp(torque3Gen.getValue)
    end
    
    torque4Gen.setValue(IEKF.tau_prev(4,1));
    if str2 == 'y'
        disp('Joint torque tau4 should be set to its previous IEKF updated value: ');
        disp(torque4Gen.getValue)
    end
    
    torque5Gen.setValue(IEKF.tau_prev(5,1));
    if str2 == 'y'
        disp('Joint torque tau5 should be set to its previous IEKF updated value: ');
        disp(torque5Gen.getValue)
    end
    
    torque6Gen.setValue(IEKF.tau_prev(6,1));
    if str2 == 'y'
        disp('Joint torque tau6 should be set to its previous IEKF updated value: ');
        disp(torque6Gen.getValue)
    end
    
    osimModel.realizeAcceleration(state); % As priorly, different values have been realized, so realize again.
    
    %______________________________________________________________________
    % IEKF Step 1 - Manager
    % Integrate to the new time: t = t + dTime
    state.setTime((t-1)*IEKF.dTime); % Start at t = 0
    
    if str2 == 'y'
        fprintf(' Time prior integration step is set to : %6.4f\n ', state.getTime)
        fprintf(' ------------------------------------------------------------------------\n')
        fprintf('  |                          Integrating forwards                        |\n')
        fprintf('  ------------------------------------------------------------------------\n')
    end
    
    % Integrate the Equations of Motion for the specified model, given the
    % current state (at which the integration will start) and a finalTime.
    % finalTime here is the new time step: dTime.
    state = manager.integrate(t*IEKF.dTime); % Integrate the EoM to the new state
    
    if str2 == 'y'
        fprintf(' Time after integration step is        : %6.4f\n', state.getTime)
    end
    
    %______________________________________________________________________
    % IEKF Step 1 - Part 2
    % Getting the predicted states from OpenSim
    IEKF = IEKF_predict_step_part_2(IEKF, osimModel, state, t, str2);

    %______________________________________________________________________
    % IEKF Step 2
    % Implement the Measurement Update Step of the Kalman filter
    for i = 1:IEKF.NoI % Number of iterations of the IEKF update step
    [IEKF, vSensor] = IEKF_update_step(IEKF, vSensor, rSensor, IMU1, IMU2, IMU3, IMU4, IMU5, IMU6, osimModel, state, t, torque1Gen, torque2Gen, torque3Gen, torque4Gen, torque5Gen, torque6Gen, ground, GravityVec, str2);
    end
    
    %______________________________________________________________________
    % IEKF Step 3
    % Setting the state and state-covariance matrix accordingly to
    % iterated updated values (Equations 4-25a, 4-25b in Literature Survey)
    % x_update = x_iter
    IEKF.x_upds(:,end) = IEKF.x_updsIter(:,end);
    if str2 == 'y'
        disp('x_update')
        disp(IEKF.x_updsIter(:,end))
    end
    
    % Updating the state covariance matrix according to the updated Kalman
    % gain and obtained updated measurement Jacobian H
    % P_update = ( I - K*H ) * P_prev
    P_prev = IEKF.P_upds(:,:,end);
    P_update = ( eye(IEKF.Dx) - ( IEKF.K_current(:,:,end) * IEKF.H_current(:,:,end) ) ) * P_prev;
    if str2 == 'y'
        disp('P_update')
        disp(P_update(:,:,end))
    end
    
    IEKF.P_upds(:,:,end) = P_update;
    
    % Now that the IEKF estimated states have been determined, set the
    % model accordingly:
    % Setting the state to its updated values
    osimModel.getCoordinateSet().get(0).setValue(state, IEKF.x_upds(1,end));       % Set generalized coordinate 1
    osimModel.getCoordinateSet().get(1).setValue(state, IEKF.x_upds(2,end));       % Set generalized coordinate 2
    osimModel.getCoordinateSet().get(2).setValue(state, IEKF.x_upds(3,end));       % Set generalized coordinate 3
    osimModel.getCoordinateSet().get(3).setValue(state, IEKF.x_upds(4,end));       % Set generalized coordinate 4
    osimModel.getCoordinateSet().get(4).setValue(state, IEKF.x_upds(5,end));       % Set generalized coordinate 5
    osimModel.getCoordinateSet().get(5).setValue(state, IEKF.x_upds(6,end));       % Set generalized coordinate 6
    osimModel.getCoordinateSet().get(0).setSpeedValue(state, IEKF.x_upds(7,end));  % Set generalized velocity 1
    osimModel.getCoordinateSet().get(1).setSpeedValue(state, IEKF.x_upds(8,end));  % Set generalized velocity 2
    osimModel.getCoordinateSet().get(2).setSpeedValue(state, IEKF.x_upds(9,end));  % Set generalized velocity 3
    osimModel.getCoordinateSet().get(3).setSpeedValue(state, IEKF.x_upds(10,end)); % Set generalized velocity 4
    osimModel.getCoordinateSet().get(4).setSpeedValue(state, IEKF.x_upds(11,end)); % Set generalized velocity 5
    osimModel.getCoordinateSet().get(5).setSpeedValue(state, IEKF.x_upds(12,end)); % Set generalized velocity 6
    torque1Gen.setValue(IEKF.x_upds(13,end))                                       % Set the joint torque 1
    torque2Gen.setValue(IEKF.x_upds(14,end))                                       % Set the joint torque 2
    torque3Gen.setValue(IEKF.x_upds(15,end))                                       % Set the joint torque 3
    torque4Gen.setValue(IEKF.x_upds(16,end))                                       % Set the joint torque 4
    torque5Gen.setValue(IEKF.x_upds(17,end))                                       % Set the joint torque 5
    torque6Gen.setValue(IEKF.x_upds(18,end))                                       % Set the joint torque 6
        
    % Checking if indeed the corect values have been set:
    q1IEKFUpdated_set   = osimModel.getCoordinateSet().get(0).getValue(state);      % Get generalized coordinate 1
    q2IEKFUpdated_set   = osimModel.getCoordinateSet().get(1).getValue(state);      % Get generalized coordinate 2
    q3IEKFUpdated_set   = osimModel.getCoordinateSet().get(2).getValue(state);      % Get generalized coordinate 3
    q4IEKFUpdated_set   = osimModel.getCoordinateSet().get(3).getValue(state);      % Get generalized coordinate 4
    q5IEKFUpdated_set   = osimModel.getCoordinateSet().get(4).getValue(state);      % Get generalized coordinate 5
    q6IEKFUpdated_set   = osimModel.getCoordinateSet().get(5).getValue(state);      % Get generalized coordinate 6
    u1IEKFUpdated_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(state); % Get generalized velocity 1
    u2IEKFUpdated_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(state); % Get generalized velocity 2
    u3IEKFUpdated_set   = osimModel.getCoordinateSet().get(2).getSpeedValue(state); % Get generalized velocity 3
    u4IEKFUpdated_set   = osimModel.getCoordinateSet().get(3).getSpeedValue(state); % Get generalized velocity 4
    u5IEKFUpdated_set   = osimModel.getCoordinateSet().get(4).getSpeedValue(state); % Get generalized velocity 5
    u6IEKFUpdated_set   = osimModel.getCoordinateSet().get(5).getSpeedValue(state); % Get generalized velocity 6
    tau1IEKFUpdated_set = torque1Gen.getValue;                                      % Get the joint torque 1
    tau2IEKFUpdated_set = torque2Gen.getValue;                                      % Get the joint torque 2
    tau3IEKFUpdated_set = torque3Gen.getValue;                                      % Get the joint torque 3
    tau4IEKFUpdated_set = torque4Gen.getValue;                                      % Get the joint torque 4
    tau5IEKFUpdated_set = torque5Gen.getValue;                                      % Get the joint torque 5
    tau6IEKFUpdated_set = torque6Gen.getValue;                                      % Get the joint torque 6
    
    if str2 == 'y'
        fprintf('Check: Value of   q1 should be set to the IEKF iterated updated value of q1  : %16.15f\n', q1IEKFUpdated_set)
        fprintf('Check: Value of   u1 should be set to the IEKF iterated updated value of u1  : %16.15f\n', u1IEKFUpdated_set)
        fprintf('Check: Value of tau1 should be set to the IEKF iterated updated value of tau1: %16.15f\n', tau1IEKFUpdated_set)
        fprintf('Check: Value of   q2 should be set to the IEKF iterated updated value of q2  : %16.15f\n', q2IEKFUpdated_set)
        fprintf('Check: Value of   u2 should be set to the IEKF iterated updated value of u2  : %16.15f\n', u2IEKFUpdated_set)
        fprintf('Check: Value of tau2 should be set to the IEKF iterated updated value of tau2: %16.15f\n', tau2IEKFUpdated_set)
        fprintf('Check: Value of   q3 should be set to the IEKF iterated updated value of q3  : %16.15f\n', q3IEKFUpdated_set)
        fprintf('Check: Value of   u3 should be set to the IEKF iterated updated value of u3  : %16.15f\n', u3IEKFUpdated_set)
        fprintf('Check: Value of tau3 should be set to the IEKF iterated updated value of tau3: %16.15f\n', tau3IEKFUpdated_set)
        fprintf('Check: Value of   q4 should be set to the IEKF iterated updated value of q4  : %16.15f\n', q4IEKFUpdated_set)
        fprintf('Check: Value of   u4 should be set to the IEKF iterated updated value of u4  : %16.15f\n', u4IEKFUpdated_set)
        fprintf('Check: Value of tau4 should be set to the IEKF iterated updated value of tau4: %16.15f\n', tau4IEKFUpdated_set)
        fprintf('Check: Value of   q5 should be set to the IEKF iterated updated value of q5  : %16.15f\n', q5IEKFUpdated_set)
        fprintf('Check: Value of   u5 should be set to the IEKF iterated updated value of u5  : %16.15f\n', u5IEKFUpdated_set)
        fprintf('Check: Value of tau5 should be set to the IEKF iterated updated value of tau5: %16.15f\n', tau5IEKFUpdated_set)
        fprintf('Check: Value of   q6 should be set to the IEKF iterated updated value of q6  : %16.15f\n', q6IEKFUpdated_set)
        fprintf('Check: Value of   u6 should be set to the IEKF iterated updated value of u6  : %16.15f\n', u6IEKFUpdated_set)
        fprintf('Check: Value of tau6 should be set to the IEKF iterated updated value of tau6: %16.15f\n', tau6IEKFUpdated_set)
    end
    
    % Again realize the acceleration stage for the last iteration
    osimModel.realizeAcceleration(state);    
    if str2 == 'y'
        fprintf(' ---------------------------------------------------------\n')
        fprintf(' |                End iteration: %d                        |\n ', t)                   
        fprintf('---------------------------------------------------------\n ')
    end
    
end

%% Stop the timer
toc

%% Calculate the RMSD values for the states
RMSD = RMSDcalculate(IEKF, Results);

%% Plotting results of created measurements and the IEKF estimated states
NoI  = IEKF.NoI;

close all

figure(1)
subplot(3,2,1)
plot(Results.T(1,:), Results.q1(1,:), 'LineWidth', 4)        % Actual generalized coordinate q1
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(1,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate q1
ylabel('Joint angle [$rad$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit1 = title('Actual and IEKF-OS estimated joint angle $q_1$');
set(tit1,'Interpreter','latex','FontSize',26);
leg1 = legend('Actual $q_1$',...
              'IEKF-OS $\hat{q}_1$');
set(leg1,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,2)
plot(Results.T(1,:), Results.u1(1,:), 'LineWidth', 4)        % Actual generalized coordinate u1
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(7,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate u1
ylabel('Joint angular velocity [$rad/s$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit2 = title('Actual and IEKF-OS estimated joint angular velocity $u_1$');
set(tit2,'Interpreter','latex','FontSize',26);
leg2 = legend('Actual $u_1$',...
              'IEKF-OS $\hat{u}_1$');
set(leg2,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,3)
plot(Results.T(1,:), Results.q2(1,:), 'LineWidth', 4)        % Actual generalized coordinate q2
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(2,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate q2
ylabel('Joint angle [$rad$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit3 = title('Actual and IEKF-OS estimated joint angle $q_2$');
set(tit3,'Interpreter','latex','FontSize',26);
leg3 = legend('Actual $q_2$',...
              'IEKF-OS $\hat{q}_2$');
set(leg3,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,4)
plot(Results.T(1,:), Results.u2(1,:), 'LineWidth', 4)        % Actual generalized coordinate u2
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(8,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate u2
ylabel('Joint angular velocity [$rad/s$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit4 = title('Actual and IEKF-OS estimated joint angular velocity $u_2$');
set(tit4,'Interpreter','latex','FontSize',26);
leg4 = legend('Actual $u_2$',...
              'IEKF-OS $\hat{u}_2$');
set(leg4,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,5)
plot(Results.T(1,:), Results.q3(1,:), 'LineWidth', 4)        % Actual generalized coordinate q3
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(3,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate q3
ylabel('Joint angle [$rad$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit5 = title('Actual and IEKF-OS estimated joint angle $q_3$');
set(tit5,'Interpreter','latex','FontSize',26);
leg5 = legend('Actual $q_3$',...
              'IEKF-OS $\hat{q}_3$');
set(leg5,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,6)
plot(Results.T(1,:), Results.u3(1,:), 'LineWidth', 4)        % Actual generalized coordinate u3
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(9,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate u3
ylabel('Joint angular velocity [$rad/s$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit6 = title('Actual and IEKF-OS estimated joint angular velocity $u_3$');
set(tit6,'Interpreter','latex','FontSize',26);
leg6 = legend('Actual $u_3$',...
              'IEKF-OS $\hat{u}_3$');
set(leg6,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

figure(2)
subplot(3,2,1)
plot(Results.T(1,:), Results.q4(1,:), 'LineWidth', 4)        % Actual generalized coordinate q4
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(4,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate q4
ylabel('Joint angle [$rad$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit7 = title('Actual and IEKF-OS estimated joint angle $q_4$');
set(tit7,'Interpreter','latex','FontSize',26);
leg7 = legend('Actual $q_4$',...
              'IEKF-OS $\hat{q}_4$');
set(leg7,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,2)
plot(Results.T(1,:), Results.u4(1,:), 'LineWidth', 4)        % Actual generalized coordinate u4
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(10,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate u4
ylabel('Joint angular velocity [$rad/s$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit8 = title('Actual and IEKF-OS estimated joint angular velocity $u_4$');
set(tit8,'Interpreter','latex','FontSize',26);
leg8 = legend('Actual $u_4$',...
              'IEKF-OS $\hat{u}_4$');
set(leg8,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,3)
plot(Results.T(1,:), Results.q5(1,:), 'LineWidth', 4)        % Actual generalized coordinate q5
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(5,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate q5
ylabel('Joint angle [$rad$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit9 = title('Actual and IEKF-OS estimated joint angle $q_5$');
set(tit9,'Interpreter','latex','FontSize',26);
leg9 = legend('Actual $q_5$',...
              'IEKF-OS $\hat{q}_5$');
set(leg9,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,4)
plot(Results.T(1,:), Results.u5(1,:), 'LineWidth', 4)        % Actual generalized coordinate u5
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(11,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate u5
ylabel('Joint angular velocity [$rad/s$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit10 = title('Actual and IEKF-OS estimated joint angular velocity $u_5$');
set(tit10,'Interpreter','latex','FontSize',26);
leg10 = legend('Actual $u_5$',...
              'IEKF-OS $\hat{u}_5$');
set(leg10,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,5)
plot(Results.T(1,:), Results.q6(1,:), 'LineWidth', 4)        % Actual generalized coordinate q6
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(6,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate q6
ylabel('Joint angle [$rad$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit11 = title('Actual and IEKF-OS estimated joint angle $q_6$');
set(tit11,'Interpreter','latex','FontSize',26);
leg11 = legend('Actual $q_6$',...
              'IEKF-OS $\hat{q}_6$');
set(leg11,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(3,2,6)
plot(Results.T(1,:), Results.u6(1,:), 'LineWidth', 4)        % Actual generalized coordinate u6
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(12,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated generalized coordinate u6
ylabel('Joint angular velocity [$rad/s$]','Interpreter','latex','FontSize',26)
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
tit12 = title('Actual and IEKF-OS estimated joint angular velocity $u_6$');
set(tit12,'Interpreter','latex','FontSize',26);
leg12 = legend('Actual $u_6$',...
              'IEKF-OS $\hat{u}_6$');
set(leg12,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');