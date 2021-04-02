%% Iterated Extended Kalman Filter script

function [IEKF, vSensor] = IEKF_init(q_init, u_init, tau_init, CovGyro1, CovAcc1, CovGyro2, CovAcc2, CovGyro3, CovAcc3, CovGyro4, CovAcc4, CovGyro5, CovAcc5, CovGyro6, CovAcc6)
    
    %-----------------------------State Vector-----------------------------
    %     [ q ] % Generalized coordinate
    % x = [ u ] % Generalized velocity
    %     [tau] % Control input / joint torque
    
    % Create state vector and state covariance matrix
    x_init = [  q_init;           % Initial state
                u_init;
              tau_init]; 
    P_init = eye(length(x_init)); % Initial state covariance
                                  % Should be set to the scale,
                                  % correspondingto their respective units,
                                  % in which you have the uncertainties. 
                                  % Here rad, rad/s and N/m so set all to 1
                                  % -> uncertain.
                                  
    %--------------------------------Model---------------------------------
    % x_t = f_{t-1} (x_{t-1}, w_{t-1}), where noise w_{t-1} ~ N(0, Q_{t-1})
    % y_t = h_t(x_t,e_t), where noise e_t ~ N(0, R_t)
    
    %       [y1_omega_x]
    %       [y1_omega_y]
    %       [y1_omega_z]
    %       [y1_acc_x]
    % y_t = [y1_acc_y]
    %       [y1_acc_z]
    %       [y2_omega_x]
    %       [y2_omega_y]
    %       [y2_omega_z]
    %       [y2_acc_x]
    %       [y2_acc_y]
    %       [y2_acc_z]
    %       [y3_omega_x]
    %       [y3_omega_y]
    %       [y3_omega_z]
    %       [y3_acc_x]
    %       [y3_acc_y]
    %       [y3_acc_z]
    %       [y4_omega_x]
    %       [y4_omega_y]
    %       [y4_omega_z]
    %       [y4_acc_x]
    %       [y4_acc_y]
    %       [y4_acc_z]
    %       [y5_omega_x]
    %       [y5_omega_y]
    %       [y5_omega_z]
    %       [y5_acc_x]
    %       [y5_acc_y]
    %       [y5_acc_z]
    %       [y6_omega_x]
    %       [y6_omega_y]
    %       [y6_omega_z]
    %       [y6_acc_x]
    %       [y6_acc_y]
    %       [y6_acc_z]

    %-----------------------------IEKF Struct------------------------------
    IEKF = struct;
    
    % Check that initial state makes sense:
    Dx = numel(x_init);
    assert(all(size(x_init) == [Dx, 1])); 
    assert(all(size(P_init) == [Dx, Dx]));
    
    % Set length of state vector:
    IEKF.Dx = Dx;
    
    % Set initial IEKF struct state vector:
    IEKF.q_init   = q_init;
    IEKF.u_init   = u_init;
    IEKF.tau_init = tau_init;
    
    %  Array which will store the predicted, iterated and updated states x
    %  and predicted and updated state covariance P for each time step:
    %  x_preds(:,t)      = predicted x at time t
    %  P_preds(:,:,t)    = predicted P at time t
    %  x_updsIter(:,t)   = iterated  x at time t  
    %  x_upds(:,t)       = updated   x at time t
    %  P_upds(:,:,t)     = updated   P at time t    
    % The matrices are initialized with 0 length, since initially no time
    % has passed, but will be extended in the predict and update steps.
    IEKF.x_preds    = NaN(Dx, 0);
    IEKF.P_preds    = NaN(Dx, Dx, 0);
    IEKF.x_upds     = NaN(Dx, 0);
    IEKF.P_upds     = NaN(Dx, Dx, 0);
    IEKF.x_updsIter = NaN(Dx, 0);
    
    % Initialize the state transition Jacobians
    IEKF.F_prev    = NaN(Dx, Dx, 0); % State transition model Jacobian with respect to state
    IEKF.L_prev    = NaN(Dx, Dx, 0); % State transition model Jacobian with respect to noise
    
    % std of 1 works better with Q*150 than std of 0.1 with Q*100
    IEKF.stdJointTorque1 = 1;    % Standard deviation for the random walk model of the joint torque tau1
    IEKF.stdJointTorque2 = 1;    % Standard deviation for the random walk model of the joint torque tau2
    IEKF.stdJointTorque3 = 1;    % Standard deviation for the random walk model of the joint torque tau3
    IEKF.stdJointTorque4 = 1;    % Standard deviation for the random walk model of the joint torque tau4
    IEKF.stdJointTorque5 = 1;    % Standard deviation for the random walk model of the joint torque tau5
    IEKF.stdJointTorque6 = 1;    % Standard deviation for the random walk model of the joint torque tau6
    
    noise_var_gen_coor1       = 0;                        % Variance of the generalized coordinate q1 process noise
    noise_var_gen_coor2       = 0;                        % Variance of the generalized coordinate q2 process noise
    noise_var_gen_coor3       = 0;                        % Variance of the generalized coordinate q3 process noise
    noise_var_gen_coor4       = 0;                        % Variance of the generalized coordinate q4 process noise
    noise_var_gen_coor5       = 0;                        % Variance of the generalized coordinate q5 process noise
    noise_var_gen_coor6       = 0;                        % Variance of the generalized coordinate q6 process noise
    noise_var_gen_vel1        = 0;                        % Variance of the generalized velocity u1 process noise
    noise_var_gen_vel2        = 0;                        % Variance of the generalized velocity u2 process noise
    noise_var_gen_vel3        = 0;                        % Variance of the generalized velocity u3 process noise
    noise_var_gen_vel4        = 0;                        % Variance of the generalized velocity u4 process noise
    noise_var_gen_vel5        = 0;                        % Variance of the generalized velocity u5 process noise
    noise_var_gen_vel6        = 0;                        % Variance of the generalized velocity u6 process noise
    noise_var_joint_torque1   = (IEKF.stdJointTorque1)^2; % Variance of the joint torque tau1 process noise
    noise_var_joint_torque2   = (IEKF.stdJointTorque2)^2; % Variance of the joint torque tau2 process noise
    noise_var_joint_torque3   = (IEKF.stdJointTorque3)^2; % Variance of the joint torque tau3 process noise
    noise_var_joint_torque4   = (IEKF.stdJointTorque4)^2; % Variance of the joint torque tau4 process noise
    noise_var_joint_torque5   = (IEKF.stdJointTorque5)^2; % Variance of the joint torque tau5 process noise
    noise_var_joint_torque6   = (IEKF.stdJointTorque6)^2; % Variance of the joint torque tau6 process noise
    
    CovarianceGyro1 = CovGyro1 * 150;
    CovarianceAcc1  = CovAcc1  * 150;
    CovarianceGyro2 = CovGyro2 * 150;
    CovarianceAcc2  = CovAcc2  * 150;
    CovarianceGyro3 = CovGyro3 * 150;
    CovarianceAcc3  = CovAcc3  * 150;
    CovarianceGyro4 = CovGyro4 * 150;
    CovarianceAcc4  = CovAcc4  * 150;
    CovarianceGyro5 = CovGyro5 * 150;
    CovarianceAcc5  = CovAcc5  * 150;
    CovarianceGyro6 = CovGyro6 * 150;
    CovarianceAcc6  = CovAcc6  * 150;
    
    IEKF.Q = diag([noise_var_gen_coor1, noise_var_gen_coor2, noise_var_gen_coor3, noise_var_gen_coor4, noise_var_gen_coor5,  noise_var_gen_coor6, noise_var_gen_vel1, noise_var_gen_vel2, noise_var_gen_vel3, noise_var_gen_vel4, noise_var_gen_vel5, noise_var_gen_vel6, noise_var_joint_torque1, noise_var_joint_torque2, noise_var_joint_torque3, noise_var_joint_torque4, noise_var_joint_torque5, noise_var_joint_torque6]); 
    IEKF.R = blkdiag(CovarianceGyro1, CovarianceAcc1, CovarianceGyro2, CovarianceAcc2, CovarianceGyro3, CovarianceAcc3, CovarianceGyro4, CovarianceAcc4, CovarianceGyro5, CovarianceAcc5, CovarianceGyro6, CovarianceAcc6);

    % Set the dimensionality of the output vector
    [numRowsY, ~] = size(IEKF.R);
    Dy = numRowsY;
    IEKF.Dy = Dy;
    
    IEKF.H_current    = NaN(Dy, Dx, 0); % Measurement model Jacobian with respect to state
    IEKF.M_current    = NaN(Dy, Dy, 0); % Measurement model Jacobian with respect to noise
    IEKF.S_current    = NaN(Dy, Dy, 0); % Innovation covariance matrix
    IEKF.K_current    = NaN(Dx, Dy, 0); % Kalman gain
    IEKF.e_current    = NaN(Dy, 0);     % Innovation / residual
    IEKF.h_current    = NaN(Dy, 0);     % Predicted measurement from OpenSim
    IEKF.hat_y_localF = NaN(Dy, 0);     % Predicted linearized measurement from OpenSim in local virtual IMU Frame
    
    % T will store the time indices corresponding to the predicted and updated x, P.
    IEKF.T = [];
    
    % Initialize as zero matrices as at the initial condition at time t = 0 no measurement is recorded hence zero.
    % Initialize the measurement model Jacobians:
    IEKF.H_current(:,:,end+1) = zeros(Dy, Dx); % Measurement model Jacobian with respect to state
    IEKF.M_current(:,:,end+1) = zeros(Dy, Dy); % Measurement model Jacobian with respect to noise
    
    IEKF.S_current(:,:,end+1)  = zeros(Dy, Dy); % Innovation covariance set to zeros as at t = 0 no measurement has taken place to be able to compute it
    IEKF.K_current(:,:,end+1)  = zeros(Dx, Dy); % Kalman gain at t=0 there is no measurement hence no gain to be able to be calculated
    IEKF.e_current(:,end+1)    = zeros(Dy, 1);  % No measurement obtained hence difference between predicted measurement and measurement = 0
    IEKF.h_current(:,end+1)    = zeros(Dy, 1);  % No simulation has taken place, hence no predicted measurement could be performed = 0
    IEKF.hat_y_localF(:,end+1) = zeros(Dy, 1);  % No simulation has taken place, hence no predicted measurement could be performed = 0

    % Set the initial prediction equal to the initialized state at t = 0
    IEKF.x_preds(:,end+1)   = x_init;
    IEKF.P_preds(:,:,end+1) = P_init;
    % Also set the IEKF updated values for the new time step equal to the initialized state at t = 0
    IEKF.x_upds(:,end+1) = x_init;
    IEKF.P_upds(:,:,end+1) = P_init;
    % Also set the IEKF iterated update values equal to the initialized state at t = 0
    IEKF.x_updsIter(:,end+1) = x_init;
    % Also set the time
    IEKF.T(end+1) = 0; % This is time t = 0
 
    %--------------------------Virtual IMU Struct--------------------------
    vSensor = struct; % v = virtual sensor => virtual attached IMUs.
    
    % IMU 1
	vSensor.IMU1_AngVel           = NaN(3,0);               % Spatial movement, hence 3 DoFs	
    vSensor.IMU1_LinAcc           = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU1_Gravity          = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU1_AngVel(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU1_LinAcc(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU1_Gravity(:,end+1) = zeros(3,1);             % At t=0 no measurement
    
    % IMU 2
	vSensor.IMU2_AngVel           = NaN(3,0);               % Spatial movement, hence 3 DoFs	
    vSensor.IMU2_LinAcc           = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU2_Gravity          = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU2_AngVel(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU2_LinAcc(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU2_Gravity(:,end+1) = zeros(3,1);             % At t=0 no measurement
    
    % IMU 3
	vSensor.IMU3_AngVel           = NaN(3,0);               % Spatial movement, hence 3 DoFs	
    vSensor.IMU3_LinAcc           = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU3_Gravity          = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU3_AngVel(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU3_LinAcc(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU3_Gravity(:,end+1) = zeros(3,1);             % At t=0 no measurement
    
    % IMU 4
	vSensor.IMU4_AngVel           = NaN(3,0);               % Spatial movement, hence 3 DoFs	
    vSensor.IMU4_LinAcc           = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU4_Gravity          = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU4_AngVel(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU4_LinAcc(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU4_Gravity(:,end+1) = zeros(3,1);             % At t=0 no measurement
    
    % IMU 5
	vSensor.IMU5_AngVel           = NaN(3,0);               % Spatial movement, hence 3 DoFs	
    vSensor.IMU5_LinAcc           = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU5_Gravity          = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU5_AngVel(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU5_LinAcc(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU5_Gravity(:,end+1) = zeros(3,1);             % At t=0 no measurement
    
    % IMU 6
	vSensor.IMU6_AngVel           = NaN(3,0);               % Spatial movement, hence 3 DoFs	
    vSensor.IMU6_LinAcc           = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU6_Gravity          = NaN(3,0);               % Spatial movement, hence 3 DoFs
    vSensor.IMU6_AngVel(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU6_LinAcc(:,end+1)  = zeros(3,1);             % At t=0 no measurement
    vSensor.IMU6_Gravity(:,end+1) = zeros(3,1);             % At t=0 no measurement
    
    %  T will store the time indices 
    vSensor.T = [];
    vSensor.T(end+1) = 0; % This is time t = 0
end