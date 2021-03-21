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
modelFilename = 'ModelDoublePendulumTwoHingeJoints.osim';
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

osimModel.addForce(joint2Actuator);

%% Set the initial values of the coordinates and their speeds
%     [ q1 ]  Generalized coordinate 1       of the Upper Pendulum ( (X,Y) planar rotation )
%     [ q2 ]  Generalized coordinate 2       of the Lower Pendulum ( (Z,Y) planar rotation )
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
angle1_rad   = deg2rad(10);  % Define generalized coordinate in [rad]
angle1_rad_s = deg2rad(0);   % Define generalized velocity in [rad/s]
tau1         = 0;            % Define joint torque in N/m. 

% Setting of initial conditions
% Setting q
inittheta1 = osimModel.getCoordinateSet().get(0);
inittheta1.setDefaultValue(angle1_rad);
% Setting u
initdottheta1 = osimModel.getCoordinateSet().get(0);
initdottheta1.setDefaultSpeedValue(angle1_rad_s);
% Setting tau
torque1Gen = Constant(tau1); % Controller: (brain) requires input as a function: Constant
% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain1.prescribeControlForActuator('UpperPendulumToGround_coord_0', torque1Gen);
osimModel.addController(brain1);

% ---------------------------- Lower pendulum ----------------------------
% Add a controller that specifies the excitation of joint2Actuator
brain2 = PrescribedController();
brain2.setName('Brain2')
brain2.addActuator(joint2Actuator);

% Initial conditions for the Lower Pendulum for the state
angle2_rad   = deg2rad(10); % Define generalized coordinate in [rad]
angle2_rad_s = deg2rad(0);  % Define generalized velocity in [rad/s]
tau2         = 0;           % Define joint torque in N/m. 

% Setting of initial conditions
% Setting q
inittheta2 = osimModel.getCoordinateSet().get(1);
inittheta2.setDefaultValue(angle2_rad);
% Setting u
initdottheta2 = osimModel.getCoordinateSet().get(1);
initdottheta2.setDefaultSpeedValue(angle2_rad_s);
% Setting tau
torque2Gen = Constant(tau2); % Controller: (brain) requires input as a function: Constant
% Syntax of prescribeControlForActuator('name of joint', Controlfunction)
brain2.prescribeControlForActuator('LowerPendulumToUpperPendulum_coord_0', torque2Gen);
osimModel.addController(brain2);


% Set initial values of generalized coordinates (q), speeds (u) and control input (tau)
q_init   = [inittheta1.getDefaultValue;
            inittheta2.getDefaultValue];
u_init   = [initdottheta1.getDefaultSpeedValue;
            initdottheta2.getDefaultSpeedValue];
tau_init = [torque1Gen.getValue;
            torque2Gen.getValue];

% Load the determined covariance matrices
load('Covariances.mat'); 

% IMU 1
CovGyro1 = Covariances.IMU1CovGyro;
CovAcc1  = Covariances.IMU1CovAcc;

% IMU 2
CovGyro2 = Covariances.IMU2CovGyro;
CovAcc2  = Covariances.IMU2CovAcc;

%% Create frames that will represent "virtual" IMUs
% Get a handle to the Pendulum from the loaded OpenSim model
UpperPendulum = osimModel.getBodySet.get('UpperPendulum');
LowerPendulum = osimModel.getBodySet.get('LowerPendulum');

% -------------------------------vFrame IMU1-------------------------------
% Frame modeled as a PhysicalOffsetFrame.
IMU1 = PhysicalOffsetFrame(); 
IMU1.setName('FirstIMU');
IMU1.setParentFrame(UpperPendulum); % Select the body to which it is attached.
IMU1.set_translation(Vec3(0,0,0)); % Translate
IMU1.set_orientation(Vec3(deg2rad(0), deg2rad(0), deg2rad(0))); % Perturb the rotation of the sensor in the update step using R^{BX}Perturb
UpperPendulum.addComponent(IMU1);

% -------------------------------vFrame IMU2-------------------------------
% Frame modeled as a PhysicalOffsetFrame.
IMU2 = PhysicalOffsetFrame(); 
IMU2.setName('SecondIMU');
IMU2.setParentFrame(LowerPendulum); % Select the body to which it is attached.
IMU2.set_translation(Vec3(0,0,0)); % Translate
IMU2.set_orientation(Vec3(deg2rad(0), deg2rad(0), deg2rad(0))); % Perturb the rotation of the sensor in the update step using R^{BX}Perturb
LowerPendulum.addComponent(IMU2);

%% Initialize the Iterated Extended Kalman Filter: IEKF
[IEKF, vSensor] = IEKF_init(q_init, u_init, tau_init, CovGyro1, CovAcc1, CovGyro2, CovAcc2);

%% Create a TableReporterVec3() to save quantities to a file after simulating.
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
    sviz.setShowSimTime(true)
end

%% Save the .osim model to a file to visualize in OpenSim
modelFilenameForOpenSim = 'OpenSimModelDoublePendulumTwoHingeJoints.osim'; 
osimModel.print(modelFilenameForOpenSim);
fprintf('\n');
disp('OpenSimModelDoublePendulumTwoHingeJoints.osim printed!');

%% Load state values from DoublePendulumTwoHingeJointsCreatingMeasurements.m
fprintf('\n');
disp('Loading the actual state data:')
load('StatesDoublePendulumTwoHingeJointsForThesis.mat') % Measurements created
disp('Done!')
fprintf('\n');

%% Load measurements from DoublePendulumTwoHingeJointsCreatingMeasurements.m
fprintf('\n');
disp('Loading the measurement data:')
load('MeasurementsDoublePendulumTwoHingeJointsForThesis.mat') % Measurements created
disp('Done!')
fprintf('\n');
disp('Running the IEKF!')

%% Start the timer
tic

%% Simulation the model and update states
% ________________________________________________________________________
% |Manager outside of function due to otherwise MATLAB internal error.   |
% |Hence, prediction step split in 3 parts: Part 1 : Set prior states    |
% |                                         Manager: Integrating EoM     |
% |                                         Part 2 : Get predicted states|
% ------------------------------------------------------------------------
    
IEKF.dTime = reportTimeInterval;% =10^-2 = Sample frequency of Xsens IMU
IEKF.finalTime = 10;
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
    IEKF = IEKF_predict_step_part_1(IEKF, osimModel, state, torque1Gen, torque2Gen, t, str2);

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
        [IEKF, vSensor] = IEKF_update_step(IEKF, vSensor, rSensor, IMU1, IMU2, osimModel, state, t, torque1Gen, torque2Gen, ground, str2);
%         [IEKF, vSensor] = IEKF_update_stepTransInKalmanLocal(IEKF, vSensor, rSensor, IMU1, IMU2, osimModel, state, t, torque1Gen, torque2Gen, ground, str2);
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
    
    % Now that the IEKF estimated states have been determined, set the model accordingly:
    % Setting the state to its updated values
    osimModel.getCoordinateSet().get(0).setValue(state, IEKF.x_upds(1,end));      % Set generalized coordinate 1
    osimModel.getCoordinateSet().get(1).setValue(state, IEKF.x_upds(2,end));      % Set generalized coordinate 2
    osimModel.getCoordinateSet().get(0).setSpeedValue(state, IEKF.x_upds(3,end)); % Set generalized velocity 1
    osimModel.getCoordinateSet().get(1).setSpeedValue(state, IEKF.x_upds(4,end)); % Set generalized velocity 2
    torque1Gen.setValue(IEKF.x_upds(5,end))                                       % Set the joint torque 1
    torque2Gen.setValue(IEKF.x_upds(6,end))                                       % Set the joint torque 2
        
    % Checking if indeed the corect values have been set:
    q1IEKFUpdated_set   = osimModel.getCoordinateSet().get(0).getValue(state);      % Get generalized coordinate 1
    q2IEKFUpdated_set   = osimModel.getCoordinateSet().get(1).getValue(state);      % Get generalized coordinate 2
    u1IEKFUpdated_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(state); % Get generalized velocity 1
    u2IEKFUpdated_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(state); % Get generalized velocity 2
    tau1IEKFUpdated_set = torque1Gen.getValue;                                      % Get the joint torque 1
    tau2IEKFUpdated_set = torque2Gen.getValue;                                      % Get the joint torque 2
    
    if str2 == 'y'
        fprintf('Check: Value of   q1 should be set to the IEKF iterated updated value of q1  : %16.15f\n', q1IEKFUpdated_set)
        fprintf('Check: Value of   u1 should be set to the IEKF iterated updated value of u1  : %16.15f\n', u1IEKFUpdated_set)
        fprintf('Check: Value of tau1 should be set to the IEKF iterated updated value of tau1: %16.15f\n', tau1IEKFUpdated_set)
        fprintf('Check: Value of   q2 should be set to the IEKF iterated updated value of q2  : %16.15f\n', q2IEKFUpdated_set)
        fprintf('Check: Value of   u2 should be set to the IEKF iterated updated value of u2  : %16.15f\n', u2IEKFUpdated_set)
        fprintf('Check: Value of tau2 should be set to the IEKF iterated updated value of tau2: %16.15f\n', tau2IEKFUpdated_set)
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

%% Plotting results of created measurements and these IEKF estimations
% ----------------- Plot 1 ------------------
NoI = IEKF.NoI;
tau1 = IEKF.stdJointTorque1;
tau2 = IEKF.stdJointTorque2;

close all

%% Plotting the states

% Plotting the state x = [q; u; tau] -> Actual x vs IEKF estimated x
%------------------------------UpperPendulum-------------------------------
figure(1)
subplot(2,2,1)
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


subplot(2,2,2)
plot(Results.T(1,:), Results.u1(1,:), 'LineWidth', 4)  % Actual joint angular velocity u1              
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(3,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated joint angular velocity u1
tit2 = title('Actual and IEKF-OS estimated joint angular velocity $u_1$');
set(tit2,'Interpreter','latex','FontSize',26);
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
ylabel('Joint angular velocity [$rad/s$]','Interpreter','latex','FontSize',26)
leg2 = legend('Actual $u_1$',...
               'IEKF-OS $\hat{u}_1$');
set(leg2,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

subplot(2,2,3)
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

subplot(2,2,4)
plot(Results.T(1,:), Results.u2(1,:), 'LineWidth', 4)  % Actual joint angular velocity u2             
hold on
plot(IEKF.T(1,:),    IEKF.x_upds(4,:),'o--g','MarkerIndices',1:50:length(IEKF.x_upds(1,:)),'MarkerSize',16,...
    'MarkerEdgeColor','g', 'LineWidth', 2) % IEKF estimated joint angular velocity u2
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

figure(2)
plot(Results.T(1,:), Results.tau1(1,:),...            % Actual joint torque tau1
     Results.T(1,:), Results.tau2(1,:),...            % Actual joint torque tau2
     IEKF.T(1,:),    IEKF.x_upds(5,:),...             % IEKF estimated joint torque tau1
     IEKF.T(1,:),    IEKF.x_upds(6,:),'LineWidth', 2) % IEKF estimated joint torque tau2
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
ylabel('Joint torque [$N\cdot m$]','Interpreter','latex','FontSize',26)
tit5 = title('Actual and IEKF-OS estimated joint torques');
set(tit5,'Interpreter','latex','FontSize',26);
leg5 = legend('Actual $\tau_1$',...
               'Actual $\tau_2$',...
               'IEKF-OS $\hat{\tau}_1$',...
               'IEKF-OS $\hat{\tau}_2$');
set(leg5,'Location','northwest','Interpreter','latex','FontSize',20);
grid on;
set(gca, 'FontSize',24)
set(gcf,'color','w');

%% Plotting actual measurements of IMU1 and IMU2
% %-----------------------------------IMU1-----------------------------------
figure(3)
% Plotting actual measurements of gyroscope1.
plot(rSensor.T(1,2:end), rSensor.Xsens1_AngVel(4,2:end),...
     rSensor.T(1,2:end), rSensor.Xsens1_AngVel(5,2:end),...
     rSensor.T(1,2:end), rSensor.Xsens1_AngVel(6,2:end),'LineWidth',2) % IMU1 gyroscope measured angular velocity with noise
tit6 = title('Actual IMU1 gyroscope measurements');
set(tit6,'Interpreter','latex','FontSize',26);
leg6 = legend('$y_{\omega,x}$ with noise',...
              '$y_{\omega,y}$ with noise',...
              '$y_{\omega,z}$ with noise');
set(leg6,'Location','northwest','Interpreter','latex','FontSize',20);
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
ylabel('$y_{Gyro} \ [rad/s]$','Interpreter','latex','FontSize',26)
grid on;
set(gca, 'FontSize',24) 

figure(4)
% Plotting actual measurements of accelerometer1.
plot(rSensor.T(1,2:end), rSensor.Xsens1_LinAcc(4,2:end),...             % IMU1 accelerometer measured x linear acceleration with noise
     rSensor.T(1,2:end), rSensor.Xsens1_LinAcc(5,2:end),...             % IMU1 accelerometer measured y linear acceleration with noise
     rSensor.T(1,2:end), rSensor.Xsens1_LinAcc(6,2:end), 'LineWidth',2) % IMU1 accelerometer measured z linear acceleration with noise
tit7 = title('Actual IMU1 accelerometer measurements');
set(tit7,'Interpreter','latex','FontSize',26);
leg7 = legend('$y_{a,x,1}$ with noise',...
              '$y_{a,y,1}$ with noise',...
              '$y_{a,z,1}$ with noise');
set(leg7,'Location','northwest','Interpreter','latex','FontSize',20);
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
ylabel('$y_{Acc} \ [m/s^2]$','Interpreter','latex','FontSize',26)
grid on;
set(gca, 'FontSize',24) 

%-----------------------------------IMU2-----------------------------------
figure(5)
% Plotting actual measurements of gyroscope2.
plot(rSensor.T(1,2:end), rSensor.Xsens2_AngVel(4,2:end),...
     rSensor.T(1,2:end), rSensor.Xsens2_AngVel(5,2:end),...
     rSensor.T(1,2:end), rSensor.Xsens2_AngVel(6,2:end),'LineWidth',2) % IMU1 gyroscope measured angular velocity with noise
tit8 = title('Actual IMU2 gyroscope measurements');
set(tit8,'Interpreter','latex','FontSize',26);
leg8 = legend('$y_{\omega,x}$ with noise',...
              '$y_{\omega,y}$ with noise',...
              '$y_{\omega,z}$ with noise');
set(leg8,'Location','northwest','Interpreter','latex','FontSize',20);
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
ylabel('$y_{Gyro} \ [rad/s]$','Interpreter','latex','FontSize',26)
grid on;
set(gca, 'FontSize',24) 

figure(6)
% Plotting actual measurements of accelerometer2.
plot(rSensor.T(1,2:end), rSensor.Xsens2_LinAcc(4,2:end),...             % IMU2 accelerometer measured x linear acceleration with noise
     rSensor.T(1,2:end), rSensor.Xsens2_LinAcc(5,2:end),...             % IMU2 accelerometer measured y linear acceleration with noise
     rSensor.T(1,2:end), rSensor.Xsens2_LinAcc(6,2:end), 'LineWidth',2) % IMU2 accelerometer measured z linear acceleration with noise
tit9 = title('Actual IMU2 accelerometer measurements');
set(tit9,'Interpreter','latex','FontSize',26);
leg9 = legend('$y_{a,x,2}$ with noise',...
              '$y_{a,y,2}$ with noise',...
              '$y_{a,z,2}$ with noise');
set(leg9,'Location','northwest','Interpreter','latex','FontSize',20);
xlabel('Time $[s]$','Interpreter','latex','FontSize',26)
ylabel('$y_{Acc} \ [m/s^2]$','Interpreter','latex','FontSize',26)
grid on;
set(gca, 'FontSize',24)