%% Clear Workspace
clear;
close all;
clc;

%% Import OpenSim Libraries into Matlab and import the .osim Model
import org.opensim.modeling.*
modelFilename = 'ModelDoublePendulumTwoHingeJoints.osim';
osimModel = Model(modelFilename);

%% Create an coordinate actuator for the UpperPendulumToGround joint
% Get the number of coordinates and a handle to the coordinate set
coordSet = osimModel.getCoordinateSet();

% Get a reference to the first coordinate
coordinate1 = coordSet.get(0);

optimalForce = 1;      % Set the optimal force. The maximum torque produced by this actuator when fully activated.
                       % Ajay told me this is essentially a gain which
                       % multiplies the user-set value for tau in the
                       % component brain.PrescribedController():
   
% Create a coordinate actuator for the joint connecting the pendulum to the ground.    
joint1Actuator = CoordinateActuator( char(coordinate1.getName) );
joint1Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
joint1Actuator.setName( coordinate1.getName() ); % Set the actuator name 
joint1Actuator.setMaxControl(Inf)                % Set max controls
joint1Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set
osimModel.addForce(joint1Actuator);

%% Create an coordinate actuator for the LowerPendulumToUpperPendulum joint
% Get a reference to the second coordinate
coordinate2 = coordSet.get(1);
   
% Create a coordinate actuator for the joint connecting both pendulums.    
joint2Actuator = CoordinateActuator( char(coordinate2.getName) );
joint2Actuator.setOptimalForce(optimalForce);    % Set the optimal force for that coordinate
joint2Actuator.setName( coordinate2.getName() ); % Set the actuator name 
joint2Actuator.setMaxControl(Inf)                % Set max controls
joint2Actuator.setMinControl(-Inf)               % Set min controls

% Append the new actuator onto the empty force set
osimModel.addForce(joint2Actuator);

%% Set the initial values of the coordinates and their speed
%     [ q1 ]  Generalized coordinate 1       of the Upper Pendulum
%     [ q2 ]  Generalized coordinate 2       of the Lower Pendulum
% x = [ u1 ]  Generalized velocity 1         of the Upper Pendulum
%     [ u2 ]  Generalized velocity 2         of the Lower Pendulum
%     [tau1]  Control input / joint torque 1 of the Upper Pendulum
%     [tau2]  Control input / joint torque 2 of the Lower Pendulum

% Get a reference to the ground object
ground = osimModel.getGround();

% Define the acceleration of gravity
Gravity = -9.80665;
osimModel.setGravity(Vec3(0, Gravity, 0));

% ---------------------------- Upper pendulum ----------------------------
% Add a controller that specifies the excitation of joint1Actuator
brain1 = PrescribedController();
brain1.setName('Brain1')
brain1.addActuator(joint1Actuator);

% Initial conditions for the Upper Pendulum for the state
angle1_rad   = deg2rad(0); % Define generalized coordinate in [rad]
angle1_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
tau1StartVal = 0;          % Define the start joint torque value in N*m.
tau1         = 8;          % Define joint torque in N*m.

% Setting of initial conditions
coordinate1.setDefaultValue(angle1_rad);	    % Setting q1
coordinate1.setDefaultSpeedValue(angle1_rad_s); % Setting u1

% Controller: (brain) requires input as a function: Constant
torque1Gen = Constant(tau1);                    % Setting tau1
% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain1.prescribeControlForActuator('UpperPendulumToGround_coord_0', StepFunction(0.5, 1.5, tau1StartVal, tau1));
osimModel.addController(brain1);

% ---------------------------- Lower pendulum ----------------------------
% Add a controller that specifies the excitation of joint2Actuator
brain2 = PrescribedController();
brain2.setName('Brain2')
brain2.addActuator(joint2Actuator);

% Initial conditions for the Lower Pendulum for the state
angle2_rad   = deg2rad(0); % Define generalized coordinate in [rad]
angle2_rad_s = deg2rad(0); % Define generalized velocity in [rad/s]
tau2StartVal = 0;          % Define the start joint torque value in N*m.
tau2         = 3;          % Define joint torque in N*m. 

% Setting of initial conditions
coordinate2.setDefaultValue(angle2_rad);	    % Setting q
coordinate2.setDefaultSpeedValue(angle2_rad_s); % Setting u

% Controller: (brain) requires input as a function: Constant
torque2Gen = Constant(tau2);                    % Setting tau
% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain2.prescribeControlForActuator('LowerPendulumToUpperPendulum_coord_0', StepFunction(2, 3, tau2StartVal, tau2));
osimModel.addController(brain2);

%% Create frames that will represent the "real" IMUs
% Get a handle to the Upper Pendulum and Lower Pendulum from the loaded OpenSim model
UpperPendulum = osimModel.getBodySet.get('UpperPendulum');
LowerPendulum = osimModel.getBodySet.get('LowerPendulum');

% -------------------------------Xsens IMU1--------------------------------
% Frame modeled as a PhysicalOffsetFrame.
Xsens1 = PhysicalOffsetFrame(); % real IMU 1
Xsens1.setName('FirstXsensIMU');
Xsens1.setParentFrame(UpperPendulum);  % Select the body to which it is attached.
Xsens1.set_translation(Vec3(0, 0, 0)); % Translate
Xsens1.set_orientation(Vec3(deg2rad(0), deg2rad(0), deg2rad(0))); % Rotate
UpperPendulum.addComponent(Xsens1);

% -------------------------------Xsens IMU2--------------------------------
% Frame modeled as a PhysicalOffsetFrame.
Xsens2 = PhysicalOffsetFrame(); % real IMU 2
Xsens2.setName('SecondXsensIMU');
Xsens2.setParentFrame(LowerPendulum); % Select the body to which it is attached.
Xsens2.set_translation(Vec3(0, 0, 0)); % Translate
Xsens2.set_orientation(Vec3(deg2rad(0), deg2rad(0), deg2rad(0))); % Rotat
LowerPendulum.addComponent(Xsens2);

%% Create struct for saving actual values of the simulated system
Results = struct;

% Define the state variables that need to be logged for validation later on
Results.q1   = NaN(1,0);
Results.q2   = NaN(1,0);
Results.u1   = NaN(1,0);
Results.u2   = NaN(1,0);
Results.tau1 = NaN(1,0);
Results.tau2 = NaN(1,0);

Results.q1(:,end+1)   = coordinate1.getDefaultValue;
Results.q2(:,end+1)   = coordinate2.getDefaultValue;
Results.u1(:,end+1)   = coordinate1.getDefaultSpeedValue;
Results.u2(:,end+1)   = coordinate2.getDefaultSpeedValue;
Results.tau1(:,end+1) = tau1StartVal;
Results.tau2(:,end+1) = tau2StartVal;

% T will store the time indices 
Results.T = [];
Results.T(end+1) = 0; % This is time t = 0

%% Create "real" IMU
% This rSensor should represent the actual sensor which will be placed on
% the users body
rSensor = struct; % r = real sensor => real attached IMUs.

% --------------------------------Xsens IMU1---------------------------------
rSensor.Xsens1_AngVel           = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens1_LinAcc           = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens1_AngVel(:,end+1)  = zeros(3,1); % At t=0 no measurement
rSensor.Xsens1_LinAcc(:,end+1)  = zeros(3,1); % At t=0 no measurement

% --------------------------------Xsens IMU2---------------------------------
rSensor.Xsens2_AngVel           = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens2_LinAcc           = NaN(3,0);   % Spatial movement, hence 3 DoFs
rSensor.Xsens2_AngVel(:,end+1)  = zeros(3,1); % At t=0 no measurement
rSensor.Xsens2_LinAcc(:,end+1)  = zeros(3,1); % At t=0 no measurement

%  T will store the time indices 
rSensor.T = [];
rSensor.T(end+1) = 0; % This is time t = 0

%% Create a TableReporterVec3() to save quantities to a file after simulating.
% This table can report SimTK::Vec3s, and thus can be used for reporting
% 3 dimensional positions, velocities, accelerations, etc.

reporter = TableReporterVec3();
reporter.setName('reporter');

% Set time interval for the reporter
SamplingFrequency = 100;  % Sampling frequency of Xsens MTw IMU
reportTimeInterval = 1/SamplingFrequency;
reporter.set_report_time_interval(reportTimeInterval);

% -------------- Reporting variables of attached Xsens IMUs ---------------

% --------------------------------- IMU1 ----------------------------------
% Report the linear acceleration of the origin of the Xsens IMU 1.
reporter.addToReport(Xsens1.getOutput('linear_acceleration'), 'Xsens1_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens1.getOutput('angular_velocity'), 'Xsens1_ang_vel');

% --------------------------------- IMU2 ----------------------------------
% Report the linear acceleration of the origin of the Xsens IMU 2.
reporter.addToReport(Xsens2.getOutput('linear_acceleration'), 'Xsens2_lin_acc');
% Report the angular velocity of the origin of the IMU.
reporter.addToReport(Xsens2.getOutput('angular_velocity'), 'Xsens2_ang_vel');

% Add the reporter to the model
osimModel.addComponent(reporter);

%% Reporter to save the applied joint torque at each time instant
% Add reporter for first applied joint torque
TauReporter = TableReporter();
TauReporter.setName('TauReporter');
TauReporter.set_report_time_interval(reportTimeInterval);
TauReporter.addToReport(joint1Actuator.getOutput('actuation'), 'Torque1');
TauReporter.addToReport(joint2Actuator.getOutput('actuation'), 'Torque2');
osimModel.addComponent(TauReporter);

%% Finalize connections and initialize the System (checks model consistency)
% initSystem() does the following:   * Generate the system of equations
%                                    * Assemble to satisfy and constraints
%                                    * Returns the initial state
%
% If no intial state is specified, the state will just be set to 0.
state = osimModel.initSystem();

%% Save the .osim model to a file to visualize in OpenSim
modelFilenameForOpenSim = 'DoublePendulumTwoHingeJointsCreatingMeasurements.osim';
osimModel.print(modelFilenameForOpenSim);
disp('DoublePendulumTwoHingeJointsCreatingMeasurements.osim printed!');

%% Simulation the model and update states: Simulation Option B
dTime = reportTimeInterval; % Sample frequency of Xsens IMU
finalTime = 10;
n = round(finalTime/dTime);

% Manager: A class that manages the execution of a simulation.
manager = Manager(osimModel);
% This must be called before calling Manager::integrate() 
manager.initialize(state);

for t = 1:n

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
    
    Results.q1(:,end+1) = q1;
    Results.q2(:,end+1) = q2;
    
    % Save the actual values of state u to the struct: Results
    u     = state.getU;
    u1    = u.get(0);
    u2    = u.get(1);
    
    Results.u1(:,end+1) = u1;
    Results.u2(:,end+1) = u2;
    
    % Save the actual values of state tau to the struct: Results
    tau1 = joint1Actuator.getActuation(state);
    tau2 = joint2Actuator.getActuation(state);
    
    Results.tau1(:,end+1)  = tau1;
    Results.tau2(:,end+1)  = tau2;
    % -------------------- End Save to struct: Results --------------------
  
    
    % -----------------------Save to struct: rSensor-----------------------
    
    % --------------------------------Xsens IMU1---------------------------------

    % IN XSENS1 FRAME
    % Get the angular velocity of Xsens IMU1 expressed in the Ground frame.
    ground_Xsens1_angvel = Xsens1.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens1angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens1_angvel, Xsens1);
    local_Xsens1AngVel = [local_Xsens1angvel.get(0) local_Xsens1angvel.get(1) local_Xsens1angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens1_AngVel(:,end+1) = local_Xsens1AngVel;

    % IN XSENS1 FRAME
    % Get the linear acceleration of Xsens IMU1 expressed in the Ground frame.
    ground_Xsens1linacc = Xsens1.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens1linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens1linacc, Xsens1);
    local_Xsens1LinAcc = [local_Xsens1linacc.get(0) local_Xsens1linacc.get(1) local_Xsens1linacc.get(2)]';
    rSensor.Xsens1_LinAcc(:,end+1) = local_Xsens1LinAcc;
   

    
    % --------------------------------Xsens IMU2---------------------------------
    
    % IN XSENS2 FRAME
    % Get the angular velocity of Xsens IMU2 expressed in the Ground frame.
    ground_Xsens2angvel = Xsens2.getAngularVelocityInGround(state);                     % Get the values
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens2angvel = ground.expressVectorInAnotherFrame(state, ground_Xsens2angvel, Xsens2);
    local_Xsens2AngVel = [local_Xsens2angvel.get(0) local_Xsens2angvel.get(1) local_Xsens2angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    rSensor.Xsens2_AngVel(:,end+1) = local_Xsens2AngVel;
    
    % IN XSENS2 FRAME
    % Get the linear acceleration of Xsens IMU2 expressed in the Ground frame.
    ground_Xsens2linacc = Xsens2.getLinearAccelerationInGround(state);
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_Xsens2linacc = ground.expressVectorInAnotherFrame(state, ground_Xsens2linacc, Xsens2);
    local_Xsens2LinAcc = [local_Xsens2linacc.get(0) local_Xsens2linacc.get(1) local_Xsens2linacc.get(2)]';
    rSensor.Xsens2_LinAcc(:,end+1) = local_Xsens2LinAcc;
   
end

%% Plot the state values 'q' of the measurement system
figure,
plot(Results.T, Results.q1(1,:), Results.T, Results.q2(1,:))
title('State values q')
legend('q1','q2')

%% Plot the state values 'u' of the measurement system
figure,
plot(Results.T, Results.u1(1,:), Results.T, Results.u2(1,:))
title('State values u')
legend('u1','u2')

%% Plot the state values 'tau' of the measurement system
figure,
plot(Results.T, Results.tau1(1,:), Results.T, Results.tau2(1,:))
title('State values tau')
legend('tau1','tau2')


%% Add zero-mean white noise to the obtained measurements
load('Covariances.mat'); 

MU = zeros(3,1);
% IMU 1
CovGyro1 = Covariances.IMU1CovGyro;
CovAcc1  = Covariances.IMU1CovAcc;

% IMU 2
CovGyro2 = Covariances.IMU2CovGyro;
CovAcc2  = Covariances.IMU2CovAcc;

% --------------------------------Xsens IMU1---------------------------------
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


% --------------------------------Xsens IMU2---------------------------------
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

%% Save measured variables in struct rSensor to .mat file
% To save the virtual measurements in the struct rSensor.
save('MeasurementsDoublePendulumTwoHingeJointsForThesis.mat', 'rSensor')
disp('Done')

% To save the actual state variables for later comparison in the struct Results. 
save('StatesDoublePendulumTwoHingeJointsForThesis.mat', 'Results')
disp('Done')