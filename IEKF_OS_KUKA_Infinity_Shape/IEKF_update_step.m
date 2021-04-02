function [IEKF, vSensor] = IEKF_update_step(IEKF, vSensor, rSensor, IMU1, IMU2, IMU3, IMU4, IMU5, IMU6, osimModel, state, t, torque1Gen, torque2Gen, torque3Gen, torque4Gen, torque5Gen, torque6Gen, ground, GravityVec, str2)

    import org.opensim.modeling.*;  % To import all OpenSim/Simbody functionality
    
    % _____________________________________________________________________
    % Get the last (updated) state and state covariance matrix from the
    % previous time step:
    x_prev = IEKF.x_upds(:,end);
    x_iter = IEKF.x_updsIter(:,end);
    P_prev = IEKF.P_upds(:,:,end);

    % Setting the state to its updated values
    osimModel.getCoordinateSet().get(0).setValue(state, x_iter(1,1));
    osimModel.getCoordinateSet().get(1).setValue(state, x_iter(2,1));
    osimModel.getCoordinateSet().get(2).setValue(state, x_iter(3,1));
    osimModel.getCoordinateSet().get(3).setValue(state, x_iter(4,1));
    osimModel.getCoordinateSet().get(4).setValue(state, x_iter(5,1));
    osimModel.getCoordinateSet().get(5).setValue(state, x_iter(6,1));
    osimModel.getCoordinateSet().get(0).setSpeedValue(state, x_iter(7,1));
    osimModel.getCoordinateSet().get(1).setSpeedValue(state, x_iter(8,1));
    osimModel.getCoordinateSet().get(2).setSpeedValue(state, x_iter(9,1));
    osimModel.getCoordinateSet().get(3).setSpeedValue(state, x_iter(10,1));
    osimModel.getCoordinateSet().get(4).setSpeedValue(state, x_iter(11,1));
    osimModel.getCoordinateSet().get(5).setSpeedValue(state, x_iter(12,1));
    torque1Gen.setValue(x_iter(13,1));
    torque2Gen.setValue(x_iter(14,1));
    torque3Gen.setValue(x_iter(15,1));
    torque4Gen.setValue(x_iter(16,1));
    torque5Gen.setValue(x_iter(17,1));
    torque6Gen.setValue(x_iter(18,1));
    
    % Checking if indeed the corect values have been set:
    q1Updated_set   = osimModel.getCoordinateSet().get(0).getValue(state);      % Set generalized coordinate 1
    q2Updated_set   = osimModel.getCoordinateSet().get(1).getValue(state);      % Set generalized coordinate 2
    q3Updated_set   = osimModel.getCoordinateSet().get(2).getValue(state);      % Set generalized coordinate 3
    q4Updated_set   = osimModel.getCoordinateSet().get(3).getValue(state);      % Set generalized coordinate 4
    q5Updated_set   = osimModel.getCoordinateSet().get(4).getValue(state);      % Set generalized coordinate 5
    q6Updated_set   = osimModel.getCoordinateSet().get(5).getValue(state);      % Set generalized coordinate 6
    u1Updated_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(state); % Set generalized velocity 1
    u2Updated_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(state); % Set generalized velocity 2
    u3Updated_set   = osimModel.getCoordinateSet().get(2).getSpeedValue(state); % Set generalized velocity 3
    u4Updated_set   = osimModel.getCoordinateSet().get(3).getSpeedValue(state); % Set generalized velocity 4
    u5Updated_set   = osimModel.getCoordinateSet().get(4).getSpeedValue(state); % Set generalized velocity 5
    u6Updated_set   = osimModel.getCoordinateSet().get(5).getSpeedValue(state); % Set generalized velocity 6
    tau1Updated_set = torque1Gen.getValue;                                      % Set joint torque 1
    tau2Updated_set = torque2Gen.getValue;                                      % Set joint torque 2
    tau3Updated_set = torque3Gen.getValue;                                      % Set joint torque 3
    tau4Updated_set = torque4Gen.getValue;                                      % Set joint torque 4
    tau5Updated_set = torque5Gen.getValue;                                      % Set joint torque 5
    tau6Updated_set = torque6Gen.getValue;                                      % Set joint torque 6
    
    if str2 == 'y'
        fprintf('Check: Value of   q1 should be set to the updated value of q1  : %16.15f\n', q1Updated_set)
        fprintf('Check: Value of   q2 should be set to the updated value of q2  : %16.15f\n', q2Updated_set)
        fprintf('Check: Value of   q3 should be set to the updated value of q3  : %16.15f\n', q3Updated_set)
        fprintf('Check: Value of   q4 should be set to the updated value of q4  : %16.15f\n', q4Updated_set)
        fprintf('Check: Value of   q5 should be set to the updated value of q5  : %16.15f\n', q5Updated_set)
        fprintf('Check: Value of   q6 should be set to the updated value of q6  : %16.15f\n', q6Updated_set)
        fprintf('Check: Value of   u1 should be set to the updated value of u1  : %16.15f\n', u1Updated_set)
        fprintf('Check: Value of   u2 should be set to the updated value of u2  : %16.15f\n', u2Updated_set)
        fprintf('Check: Value of   u3 should be set to the updated value of u3  : %16.15f\n', u3Updated_set)
        fprintf('Check: Value of   u4 should be set to the updated value of u4  : %16.15f\n', u4Updated_set)
        fprintf('Check: Value of   u5 should be set to the updated value of u5  : %16.15f\n', u5Updated_set)
        fprintf('Check: Value of   u6 should be set to the updated value of u6  : %16.15f\n', u6Updated_set)
        fprintf('Check: Value of tau1 should be set to the updated value of tau1: %16.15f\n', tau1Updated_set)
        fprintf('Check: Value of tau2 should be set to the updated value of tau2: %16.15f\n', tau2Updated_set)
        fprintf('Check: Value of tau3 should be set to the updated value of tau3: %16.15f\n', tau3Updated_set)
        fprintf('Check: Value of tau4 should be set to the updated value of tau4: %16.15f\n', tau4Updated_set)
        fprintf('Check: Value of tau5 should be set to the updated value of tau5: %16.15f\n', tau5Updated_set)
        fprintf('Check: Value of tau6 should be set to the updated value of tau6: %16.15f\n', tau6Updated_set)
    end
    
    % -----------------Reporting variables of attached IMU-----------------
    % h_t(x_t, e_t) = [omega_s] % where s denotes the s-th sensor.
    %                 [  a_s  ] % where a_s = [a_s_x]
    %                                         [a_s_y]
    
    % Again realize the acceleration stage to be able to compute Qdot and Udot correctly!
    osimModel.realizeAcceleration(state);
    
    % --------------------------------IMU1---------------------------------
    % Get the angular velocity of the IMU1.
    ground_IMU1_angvel = IMU1.getAngularVelocityInGround(state); % Get the values
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU1_angvel = ground.expressVectorInAnotherFrame(state, ground_IMU1_angvel, IMU1);
    local_IMU1_AngVel = [local_IMU1_angvel.get(0) local_IMU1_angvel.get(1) local_IMU1_angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU1_AngVel(:,end+1) = local_IMU1_AngVel; % Store values in struct: "vSensor"
    
    % Determining the gravity vector expressed in IMU 1 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_IMU1_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,IMU1);
    IMU1_Gravity         = [local_IMU1_Gravity.get(0) local_IMU1_Gravity.get(1) local_IMU1_Gravity.get(2)]';
    vSensor.IMU1_Gravity(:,end+1) = IMU1_Gravity;
    
    % Get the linear acceleration of the IMU1.
    ground_IMU1_linacc = IMU1.getLinearAccelerationInGround(state); % Get the values
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU1_linacc = ground.expressVectorInAnotherFrame(state, ground_IMU1_linacc, IMU1);
    local_IMU1_LinAcc = [local_IMU1_linacc.get(0) local_IMU1_linacc.get(1) local_IMU1_linacc.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU1_LinAcc(:,end+1) = local_IMU1_LinAcc; % Store values in struct: "vSensor"
    
    % --------------------------------IMU2---------------------------------
    % Get the angular velocity of the IMU2.
    ground_IMU2_angvel = IMU2.getAngularVelocityInGround(state); % Get the values
    % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU2_angvel = ground.expressVectorInAnotherFrame(state, ground_IMU2_angvel, IMU2);
    local_IMU2_AngVel = [local_IMU2_angvel.get(0) local_IMU2_angvel.get(1) local_IMU2_angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU2_AngVel(:,end+1) = local_IMU2_AngVel; % Store values in struct: "vSensor"

    % Determining the gravity vector expressed in IMU 2 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_IMU2_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,IMU2);
    IMU2_Gravity         = [local_IMU2_Gravity.get(0) local_IMU2_Gravity.get(1) local_IMU2_Gravity.get(2)]';
    vSensor.IMU2_Gravity(:,end+1) = IMU2_Gravity;    
    
    % Get the linear acceleration of the IMU2.
    ground_IMU2_linacc = IMU2.getLinearAccelerationInGround(state); % Get the values
    % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU2_linacc = ground.expressVectorInAnotherFrame(state, ground_IMU2_linacc, IMU2);
    local_IMU2_LinAcc = [local_IMU2_linacc.get(0) local_IMU2_linacc.get(1) local_IMU2_linacc.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU2_LinAcc(:,end+1) = local_IMU2_LinAcc; % Store values in struct: "vSensor"
    
    % --------------------------------IMU3---------------------------------
    % Get the angular velocity of the IMU3.
    ground_IMU3_angvel = IMU3.getAngularVelocityInGround(state); % Get the values
    % Re-express in local sensor frame of IMU3: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU3_angvel = ground.expressVectorInAnotherFrame(state, ground_IMU3_angvel, IMU3);
    local_IMU3_AngVel = [local_IMU3_angvel.get(0) local_IMU3_angvel.get(1) local_IMU3_angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU3_AngVel(:,end+1) = local_IMU3_AngVel; % Store values in struct: "vSensor"

    % Determining the gravity vector expressed in IMU 3 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_IMU3_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,IMU3);
    IMU3_Gravity         = [local_IMU3_Gravity.get(0) local_IMU3_Gravity.get(1) local_IMU3_Gravity.get(2)]';
    vSensor.IMU3_Gravity(:,end+1) = IMU3_Gravity;    
    
    % Get the linear acceleration of the IMU3.
    ground_IMU3_linacc = IMU3.getLinearAccelerationInGround(state); % Get the values
    % Re-express in local sensor frame of IMU3: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU3_linacc = ground.expressVectorInAnotherFrame(state, ground_IMU3_linacc, IMU3);
    local_IMU3_LinAcc = [local_IMU3_linacc.get(0) local_IMU3_linacc.get(1) local_IMU3_linacc.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU3_LinAcc(:,end+1) = local_IMU3_LinAcc; % Store values in struct: "vSensor"
    
    % --------------------------------IMU4---------------------------------
    % Get the angular velocity of the IMU4.
    ground_IMU4_angvel = IMU4.getAngularVelocityInGround(state); % Get the values
    % Re-express in local sensor frame of IMU4: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU4_angvel = ground.expressVectorInAnotherFrame(state, ground_IMU4_angvel, IMU4);
    local_IMU4_AngVel = [local_IMU4_angvel.get(0) local_IMU4_angvel.get(1) local_IMU4_angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU4_AngVel(:,end+1) = local_IMU4_AngVel; % Store values in struct: "vSensor"

    % Determining the gravity vector expressed in IMU 4 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_IMU4_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,IMU4);
    IMU4_Gravity         = [local_IMU4_Gravity.get(0) local_IMU4_Gravity.get(1) local_IMU4_Gravity.get(2)]';
    vSensor.IMU4_Gravity(:,end+1) = IMU4_Gravity;      
    
    % Get the linear acceleration of the IMU4.
    ground_IMU4_linacc = IMU4.getLinearAccelerationInGround(state); % Get the values
    % Re-express in local sensor frame of IMU4: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU4_linacc = ground.expressVectorInAnotherFrame(state, ground_IMU4_linacc, IMU4);
    local_IMU4_LinAcc = [local_IMU4_linacc.get(0) local_IMU4_linacc.get(1) local_IMU4_linacc.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU4_LinAcc(:,end+1) = local_IMU4_LinAcc; % Store values in struct: "vSensor"
    
    % --------------------------------IMU5---------------------------------
    % Get the angular velocity of the IMU5.
    ground_IMU5_angvel = IMU5.getAngularVelocityInGround(state); % Get the values
    % Re-express in local sensor frame of IMU5: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU5_angvel = ground.expressVectorInAnotherFrame(state, ground_IMU5_angvel, IMU5);
    local_IMU5_AngVel = [local_IMU5_angvel.get(0) local_IMU5_angvel.get(1) local_IMU5_angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU5_AngVel(:,end+1) = local_IMU5_AngVel; % Store values in struct: "vSensor"
    
    % Determining the gravity vector expressed in IMU 5 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_IMU5_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,IMU5);
    IMU5_Gravity         = [local_IMU5_Gravity.get(0) local_IMU5_Gravity.get(1) local_IMU5_Gravity.get(2)]';
    vSensor.IMU5_Gravity(:,end+1) = IMU5_Gravity; 
    
    % Get the linear acceleration of the IMU5.
    ground_IMU5_linacc = IMU5.getLinearAccelerationInGround(state); % Get the values
    % Re-express in local sensor frame of IMU5: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU5_linacc = ground.expressVectorInAnotherFrame(state, ground_IMU5_linacc, IMU5);
    local_IMU5_LinAcc = [local_IMU5_linacc.get(0) local_IMU5_linacc.get(1) local_IMU5_linacc.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU5_LinAcc(:,end+1) = local_IMU5_LinAcc; % Store values in struct: "vSensor"
    
    % --------------------------------IMU6---------------------------------
    % Get the angular velocity of the IMU6.
    ground_IMU6_angvel = IMU6.getAngularVelocityInGround(state); % Get the values
    % Re-express in local sensor frame of IMU6: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU6_angvel = ground.expressVectorInAnotherFrame(state, ground_IMU6_angvel, IMU6);
    local_IMU6_AngVel = [local_IMU6_angvel.get(0) local_IMU6_angvel.get(1) local_IMU6_angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU6_AngVel(:,end+1) = local_IMU6_AngVel; % Store values in struct: "vSensor"

    % Determining the gravity vector expressed in IMU 6 frame
    % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
    local_IMU6_Gravity   = ground.expressVectorInAnotherFrame(state,GravityVec,IMU6);
    IMU6_Gravity         = [local_IMU6_Gravity.get(0) local_IMU6_Gravity.get(1) local_IMU6_Gravity.get(2)]';
    vSensor.IMU6_Gravity(:,end+1) = IMU6_Gravity;     
    
    % Get the linear acceleration of the IMU6.
    ground_IMU6_linacc = IMU6.getLinearAccelerationInGround(state); % Get the values
    % Re-express in local sensor frame of IMU6: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU6_linacc = ground.expressVectorInAnotherFrame(state, ground_IMU6_linacc, IMU6);
    local_IMU6_LinAcc = [local_IMU6_linacc.get(0) local_IMU6_linacc.get(1) local_IMU6_linacc.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU6_LinAcc(:,end+1) = local_IMU6_LinAcc; % Store values in struct: "vSensor"    
    
    % Virtual IMU measurements are stacked as follows:
    %                               [omega_x_IMU1]
    %                               [omega_y_IMU1]
    %                               [omega_z_IMU1]
    %                               [  a_x_IMU1  ]
    %                               [  a_y_IMU1  ]
    %                               [  a_z_IMU1  ]
    %                               [omega_x_IMU2]
    %                               [omega_y_IMU2]
    %                               [omega_z_IMU2]
    %                               [  a_x_IMU2  ]
    %                               [  a_y_IMU2  ]
    %                               [  a_z_IMU2  ]
    %                               [omega_x_IMU3]
    %                               [omega_y_IMU3]
    %                               [omega_z_IMU3]
    %                               [  a_x_IMU3  ]
    %                               [  a_y_IMU3  ]
    %                               [  a_z_IMU3  ]
    %                               [omega_x_IMU4]
    %                               [omega_y_IMU4]
    %                               [omega_z_IMU4]
    %                               [  a_x_IMU4  ]
    %                               [  a_y_IMU4  ]
    %                               [  a_z_IMU4  ]
    %                               [omega_x_IMU5]
    %                               [omega_y_IMU5]
    %                               [omega_z_IMU5]
    %                               [  a_x_IMU5  ]
    %                               [  a_y_IMU5  ]
    %                               [  a_z_IMU5  ]
    %                               [omega_x_IMU6]
    %                               [omega_y_IMU6]
    %                               [omega_z_IMU6]
    %                               [  a_x_IMU6  ]
    %                               [  a_y_IMU6  ]
    %                               [  a_z_IMU6  ]
    
    
    % First six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU1 = local_IMU1_AngVel(1,1);
    angvel_y_IMU1 = local_IMU1_AngVel(2,1);
    angvel_z_IMU1 = local_IMU1_AngVel(3,1);
    
    linacc_x_IMU1 = local_IMU1_LinAcc(1,1) + IMU1_Gravity(1,1);
    linacc_y_IMU1 = local_IMU1_LinAcc(2,1) + IMU1_Gravity(2,1);
    linacc_z_IMU1 = local_IMU1_LinAcc(3,1) + IMU1_Gravity(3,1);
    
    % Second six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU2 = local_IMU2_AngVel(1,1);
    angvel_y_IMU2 = local_IMU2_AngVel(2,1);
    angvel_z_IMU2 = local_IMU2_AngVel(3,1);

    linacc_x_IMU2 = local_IMU2_LinAcc(1,1) + IMU2_Gravity(1,1);
    linacc_y_IMU2 = local_IMU2_LinAcc(2,1) + IMU2_Gravity(2,1);
    linacc_z_IMU2 = local_IMU2_LinAcc(3,1) + IMU2_Gravity(3,1);
    
    % Third six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU3 = local_IMU3_AngVel(1,1);
    angvel_y_IMU3 = local_IMU3_AngVel(2,1);
    angvel_z_IMU3 = local_IMU3_AngVel(3,1);
    
    linacc_x_IMU3 = local_IMU3_LinAcc(1,1) + IMU3_Gravity(1,1);
    linacc_y_IMU3 = local_IMU3_LinAcc(2,1) + IMU3_Gravity(2,1);
    linacc_z_IMU3 = local_IMU3_LinAcc(3,1) + IMU3_Gravity(3,1);
    
    % Fourth six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU4 = local_IMU4_AngVel(1,1);
    angvel_y_IMU4 = local_IMU4_AngVel(2,1);
    angvel_z_IMU4 = local_IMU4_AngVel(3,1);
    
    linacc_x_IMU4 = local_IMU4_LinAcc(1,1) + IMU4_Gravity(1,1);
    linacc_y_IMU4 = local_IMU4_LinAcc(2,1) + IMU4_Gravity(2,1);
    linacc_z_IMU4 = local_IMU4_LinAcc(3,1) + IMU4_Gravity(3,1);
    
    % Fifth six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU5 = local_IMU5_AngVel(1,1);
    angvel_y_IMU5 = local_IMU5_AngVel(2,1);
    angvel_z_IMU5 = local_IMU5_AngVel(3,1);
    
    linacc_x_IMU5 = local_IMU5_LinAcc(1,1) + IMU5_Gravity(1,1);
    linacc_y_IMU5 = local_IMU5_LinAcc(2,1) + IMU5_Gravity(2,1);
    linacc_z_IMU5 = local_IMU5_LinAcc(3,1) + IMU5_Gravity(3,1);
    
    % Last six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU6 = local_IMU6_AngVel(1,1);
    angvel_y_IMU6 = local_IMU6_AngVel(2,1);
    angvel_z_IMU6 = local_IMU6_AngVel(3,1);
    
    linacc_x_IMU6 = local_IMU6_LinAcc(1,1) + IMU6_Gravity(1,1);
    linacc_y_IMU6 = local_IMU6_LinAcc(2,1) + IMU6_Gravity(2,1);
    linacc_z_IMU6 = local_IMU6_LinAcc(3,1) + IMU6_Gravity(3,1);
    
    if str2 == 'y'
        fprintf('  ------------------------------------------------------------------------\n')
        fprintf('  |                         Determining H_current:                       |\n')
        fprintf('  ------------------------------------------------------------------------\n')
    end
    
    % Setting the operation point for the pertubation method of H_current estimation
    OP = x_iter;
    if str2 == 'y'
        disp('Operating point = ')
        disp(OP)
    end
    
    EvaluateOP   = [angvel_x_IMU1; % Evaluate function prior to pertubing
                    angvel_y_IMU1;
                    angvel_z_IMU1;
                    linacc_x_IMU1;
                    linacc_y_IMU1;
                    linacc_z_IMU1;
                    angvel_x_IMU2;
                    angvel_y_IMU2;
                    angvel_z_IMU2;
                    linacc_x_IMU2;
                    linacc_y_IMU2;
                    linacc_z_IMU2;
                    angvel_x_IMU3;
                    angvel_y_IMU3;
                    angvel_z_IMU3;
                    linacc_x_IMU3;
                    linacc_y_IMU3;
                    linacc_z_IMU3;
                    angvel_x_IMU4;
                    angvel_y_IMU4;
                    angvel_z_IMU4;
                    linacc_x_IMU4;
                    linacc_y_IMU4;
                    linacc_z_IMU4;
                    angvel_x_IMU5;
                    angvel_y_IMU5;
                    angvel_z_IMU5;
                    linacc_x_IMU5;
                    linacc_y_IMU5;
                    linacc_z_IMU5;
                    angvel_x_IMU6;
                    angvel_y_IMU6;
                    angvel_z_IMU6;
                    linacc_x_IMU6;
                    linacc_y_IMU6;
                    linacc_z_IMU6];
    if str2 == 'y'            
        disp('Evaluate IMU measurement output h(x_iter, 0) = ')
        disp(EvaluateOP)
    end

    [numRows1, ~] = size(EvaluateOP); % Give length of Evaluated Operation Point vector
    [numRows2, ~] = size(OP);         % Give length of Operation Point vector            

    h = 1.e-8;                        % Pertubation step
    
    % Declaring the pertubation operation point for the pertubation method of H_current estimation
    OPperturb = x_iter;
    
    % Allocate memory for the Jacobian H_current
    J = NaN(numRows1, numRows2);
    
    for i = 1:numRows2
        if str2 == 'y'
            fprintf('Perturbing state %u yielding OPperturb = \n', i)
        end
        OPperturb(i)=OPperturb(i)+h;
        if str2 == 'y'
            disp(OPperturb)
        end
        
        % First, create a new !copy! of the source object, not a reference
        % to the original object: ( we don't want to change the actual
        % state in the simulation of the pendulum!!!
        stateToBePerturbed = State(state);

        % Setting the statePerturbed to its perturbed values
        osimModel.getCoordinateSet().get(0).setValue(stateToBePerturbed, OPperturb(1,1));       % Set generalized coordinate 1
        osimModel.getCoordinateSet().get(1).setValue(stateToBePerturbed, OPperturb(2,1));       % Set generalized coordinate 2
        osimModel.getCoordinateSet().get(2).setValue(stateToBePerturbed, OPperturb(3,1));       % Set generalized coordinate 3
        osimModel.getCoordinateSet().get(3).setValue(stateToBePerturbed, OPperturb(4,1));       % Set generalized coordinate 4
        osimModel.getCoordinateSet().get(4).setValue(stateToBePerturbed, OPperturb(5,1));       % Set generalized coordinate 5
        osimModel.getCoordinateSet().get(5).setValue(stateToBePerturbed, OPperturb(6,1));       % Set generalized coordinate 6
        
        osimModel.getCoordinateSet().get(0).setSpeedValue(stateToBePerturbed, OPperturb(7,1));  % Set generalized velocity 1
        osimModel.getCoordinateSet().get(1).setSpeedValue(stateToBePerturbed, OPperturb(8,1));  % Set generalized velocity 2
        osimModel.getCoordinateSet().get(2).setSpeedValue(stateToBePerturbed, OPperturb(9,1));  % Set generalized velocity 3
        osimModel.getCoordinateSet().get(3).setSpeedValue(stateToBePerturbed, OPperturb(10,1)); % Set generalized velocity 4
        osimModel.getCoordinateSet().get(4).setSpeedValue(stateToBePerturbed, OPperturb(11,1)); % Set generalized velocity 5
        osimModel.getCoordinateSet().get(5).setSpeedValue(stateToBePerturbed, OPperturb(12,1)); % Set generalized velocity 6
        
        torque1Gen.setValue(OPperturb(13,1));                                                   % Set joint torque 1
        torque2Gen.setValue(OPperturb(14,1));                                                   % Set joint torque 2
        torque3Gen.setValue(OPperturb(15,1));                                                   % Set joint torque 3
        torque4Gen.setValue(OPperturb(16,1));                                                   % Set joint torque 4
        torque5Gen.setValue(OPperturb(17,1));                                                   % Set joint torque 5
        torque6Gen.setValue(OPperturb(18,1));                                                   % Set joint torque 6

        % Checking if indeed the corect values have been set:
        q1Perturbed_set   = osimModel.getCoordinateSet().get(0).getValue(stateToBePerturbed);      % Set generalized coordinate 1
        q2Perturbed_set   = osimModel.getCoordinateSet().get(1).getValue(stateToBePerturbed);      % Set generalized coordinate 2
        q3Perturbed_set   = osimModel.getCoordinateSet().get(2).getValue(stateToBePerturbed);      % Set generalized coordinate 3
        q4Perturbed_set   = osimModel.getCoordinateSet().get(3).getValue(stateToBePerturbed);      % Set generalized coordinate 4
        q5Perturbed_set   = osimModel.getCoordinateSet().get(4).getValue(stateToBePerturbed);      % Set generalized coordinate 5
        q6Perturbed_set   = osimModel.getCoordinateSet().get(5).getValue(stateToBePerturbed);      % Set generalized coordinate 6
        
        u1Perturbed_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(stateToBePerturbed); % Set generalized velocity 1
        u2Perturbed_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(stateToBePerturbed); % Set generalized velocity 2
        u3Perturbed_set   = osimModel.getCoordinateSet().get(2).getSpeedValue(stateToBePerturbed); % Set generalized velocity 3
        u4Perturbed_set   = osimModel.getCoordinateSet().get(3).getSpeedValue(stateToBePerturbed); % Set generalized velocity 4
        u5Perturbed_set   = osimModel.getCoordinateSet().get(4).getSpeedValue(stateToBePerturbed); % Set generalized velocity 5
        u6Perturbed_set   = osimModel.getCoordinateSet().get(5).getSpeedValue(stateToBePerturbed); % Set generalized velocity 6
        
        tau1Perturbed_set = torque1Gen.getValue;                                                   % Set joint torque 1
        tau2Perturbed_set = torque2Gen.getValue;                                                   % Set joint torque 2
        tau3Perturbed_set = torque3Gen.getValue;                                                   % Set joint torque 3
        tau4Perturbed_set = torque4Gen.getValue;                                                   % Set joint torque 4
        tau5Perturbed_set = torque5Gen.getValue;                                                   % Set joint torque 5
        tau6Perturbed_set = torque6Gen.getValue;                                                   % Set joint torque 6
        
        if str2 == 'y'
            fprintf('Check: Value of   q1 should be set to the perturbed value of   q1: %16.15f\n', q1Perturbed_set)
            fprintf('Check: Value of   q2 should be set to the perturbed value of   q2: %16.15f\n', q2Perturbed_set)
            fprintf('Check: Value of   q3 should be set to the perturbed value of   q3: %16.15f\n', q3Perturbed_set)
            fprintf('Check: Value of   q4 should be set to the perturbed value of   q4: %16.15f\n', q4Perturbed_set)
            fprintf('Check: Value of   q5 should be set to the perturbed value of   q5: %16.15f\n', q5Perturbed_set)
            fprintf('Check: Value of   q6 should be set to the perturbed value of   q6: %16.15f\n', q6Perturbed_set)
            fprintf('Check: Value of   u1 should be set to the perturbed value of   u1: %16.15f\n', u1Perturbed_set)
            fprintf('Check: Value of   u2 should be set to the perturbed value of   u2: %16.15f\n', u2Perturbed_set)
            fprintf('Check: Value of   u3 should be set to the perturbed value of   u3: %16.15f\n', u3Perturbed_set)
            fprintf('Check: Value of   u4 should be set to the perturbed value of   u4: %16.15f\n', u4Perturbed_set)
            fprintf('Check: Value of   u5 should be set to the perturbed value of   u5: %16.15f\n', u5Perturbed_set)
            fprintf('Check: Value of   u6 should be set to the perturbed value of   u6: %16.15f\n', u6Perturbed_set)
            fprintf('Check: Value of tau1 should be set to the perturbed value of tau1: %16.15f\n', tau1Perturbed_set)
            fprintf('Check: Value of tau2 should be set to the perturbed value of tau2: %16.15f\n', tau2Perturbed_set)
            fprintf('Check: Value of tau3 should be set to the perturbed value of tau3: %16.15f\n', tau3Perturbed_set)
            fprintf('Check: Value of tau4 should be set to the perturbed value of tau4: %16.15f\n', tau4Perturbed_set)
            fprintf('Check: Value of tau5 should be set to the perturbed value of tau5: %16.15f\n', tau5Perturbed_set)
            fprintf('Check: Value of tau6 should be set to the perturbed value of tau6: %16.15f\n', tau6Perturbed_set)
        end
        
        % Again realize the acceleration stage to be able to compute Qdot and Udot correctly!
        osimModel.realizeAcceleration(stateToBePerturbed);
        
        %-------------------------------IMU1-------------------------------
        % Get the angular velocity of IMU1.
        ground_angvelIMU1Perturbed = IMU1.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU1Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU1Perturbed, IMU1);
        angvelIMU1PerturbedValues = [local_angvelIMU1Perturbed.get(0) local_angvelIMU1Perturbed.get(1) local_angvelIMU1Perturbed.get(2)]';

        % Determining the gravity vector expressed in IMU 1 frame
        % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
        local_IMU1_Gravity_Perturbed   = ground.expressVectorInAnotherFrame(stateToBePerturbed,GravityVec,IMU1);
        IMU1_Perturbed_Gravity         = [local_IMU1_Gravity_Perturbed.get(0) local_IMU1_Gravity_Perturbed.get(1) local_IMU1_Gravity_Perturbed.get(2)]';
        
        % Get the linear acceleration of IMU1.
        ground_linaccIMU1Perturbed = IMU1.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU1Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU1Perturbed, IMU1);
        linaccIMU1PerturbedValues = [local_linaccIMU1Perturbed.get(0) local_linaccIMU1Perturbed.get(1) local_linaccIMU1Perturbed.get(2)]';

        % First set of outputs in EvaluateOP for determining H_current below.
        angvel_x1Perturbed = angvelIMU1PerturbedValues(1,1);
        angvel_y1Perturbed = angvelIMU1PerturbedValues(2,1);
        angvel_z1Perturbed = angvelIMU1PerturbedValues(3,1);
        
        linacc_x1Perturbed = linaccIMU1PerturbedValues(1,1) + IMU1_Perturbed_Gravity(1,1);
        linacc_y1Perturbed = linaccIMU1PerturbedValues(2,1) + IMU1_Perturbed_Gravity(2,1);
        linacc_z1Perturbed = linaccIMU1PerturbedValues(3,1) + IMU1_Perturbed_Gravity(3,1);
        
        %-------------------------------IMU2-------------------------------
        % Get the angular velocity of IMU2.
        ground_angvelIMU2Perturbed = IMU2.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU2Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU2Perturbed, IMU2);
        angvelIMU2PerturbedValues = [local_angvelIMU2Perturbed.get(0) local_angvelIMU2Perturbed.get(1) local_angvelIMU2Perturbed.get(2)]';

        % Determining the gravity vector expressed in IMU 2 frame
        % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
        local_IMU2_Gravity_Perturbed   = ground.expressVectorInAnotherFrame(stateToBePerturbed,GravityVec,IMU2);
        IMU2_Perturbed_Gravity         = [local_IMU2_Gravity_Perturbed.get(0) local_IMU2_Gravity_Perturbed.get(1) local_IMU2_Gravity_Perturbed.get(2)]';
        
        % Get the linear acceleration of IMU2.
        ground_linaccIMU2Perturbed = IMU2.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU2Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU2Perturbed, IMU2);
        linaccIMU2PerturbedValues = [local_linaccIMU2Perturbed.get(0) local_linaccIMU2Perturbed.get(1) local_linaccIMU2Perturbed.get(2)]';

        % Second set of outputs in EvaluateOP for determining H_current below.
        angvel_x2Perturbed = angvelIMU2PerturbedValues(1,1);
        angvel_y2Perturbed = angvelIMU2PerturbedValues(2,1);
        angvel_z2Perturbed = angvelIMU2PerturbedValues(3,1);

        linacc_x2Perturbed = linaccIMU2PerturbedValues(1,1) + IMU2_Perturbed_Gravity(1,1);
        linacc_y2Perturbed = linaccIMU2PerturbedValues(2,1) + IMU2_Perturbed_Gravity(2,1);
        linacc_z2Perturbed = linaccIMU2PerturbedValues(3,1) + IMU2_Perturbed_Gravity(3,1);
        
        %-------------------------------IMU3-------------------------------
        % Get the angular velocity of IMU3.
        ground_angvelIMU3Perturbed = IMU3.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU3: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU3Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU3Perturbed, IMU3);
        angvelIMU3PerturbedValues = [local_angvelIMU3Perturbed.get(0) local_angvelIMU3Perturbed.get(1) local_angvelIMU3Perturbed.get(2)]';

        % Determining the gravity vector expressed in IMU 3 frame
        % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
        local_IMU3_Gravity_Perturbed   = ground.expressVectorInAnotherFrame(stateToBePerturbed,GravityVec,IMU3);
        IMU3_Perturbed_Gravity         = [local_IMU3_Gravity_Perturbed.get(0) local_IMU3_Gravity_Perturbed.get(1) local_IMU3_Gravity_Perturbed.get(2)]';
        
        % Get the linear acceleration of IMU3.
        ground_linaccIMU3Perturbed = IMU3.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU3: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU3Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU3Perturbed, IMU3);
        linaccIMU3PerturbedValues = [local_linaccIMU3Perturbed.get(0) local_linaccIMU3Perturbed.get(1) local_linaccIMU3Perturbed.get(2)]';

        % Second set of outputs in EvaluateOP for determining H_current below.
        angvel_x3Perturbed = angvelIMU3PerturbedValues(1,1);
        angvel_y3Perturbed = angvelIMU3PerturbedValues(2,1);
        angvel_z3Perturbed = angvelIMU3PerturbedValues(3,1);

        linacc_x3Perturbed = linaccIMU3PerturbedValues(1,1) + IMU3_Perturbed_Gravity(1,1);
        linacc_y3Perturbed = linaccIMU3PerturbedValues(2,1) + IMU3_Perturbed_Gravity(2,1);
        linacc_z3Perturbed = linaccIMU3PerturbedValues(3,1) + IMU3_Perturbed_Gravity(3,1);
        
        %-------------------------------IMU4-------------------------------
        % Get the angular velocity of IMU4.
        ground_angvelIMU4Perturbed = IMU4.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU4: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU4Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU4Perturbed, IMU4);
        angvelIMU4PerturbedValues = [local_angvelIMU4Perturbed.get(0) local_angvelIMU4Perturbed.get(1) local_angvelIMU4Perturbed.get(2)]';

        % Determining the gravity vector expressed in IMU 4 frame
        % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
        local_IMU4_Gravity_Perturbed   = ground.expressVectorInAnotherFrame(stateToBePerturbed,GravityVec,IMU4);
        IMU4_Perturbed_Gravity         = [local_IMU4_Gravity_Perturbed.get(0) local_IMU4_Gravity_Perturbed.get(1) local_IMU4_Gravity_Perturbed.get(2)]';
        
        % Get the linear acceleration of IMU4.
        ground_linaccIMU4Perturbed = IMU4.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU4: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU4Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU4Perturbed, IMU4);
        linaccIMU4PerturbedValues = [local_linaccIMU4Perturbed.get(0) local_linaccIMU4Perturbed.get(1) local_linaccIMU4Perturbed.get(2)]';

        % Second set of outputs in EvaluateOP for determining H_current below.
        angvel_x4Perturbed = angvelIMU4PerturbedValues(1,1);
        angvel_y4Perturbed = angvelIMU4PerturbedValues(2,1);
        angvel_z4Perturbed = angvelIMU4PerturbedValues(3,1);

        linacc_x4Perturbed = linaccIMU4PerturbedValues(1,1) + IMU4_Perturbed_Gravity(1,1);
        linacc_y4Perturbed = linaccIMU4PerturbedValues(2,1) + IMU4_Perturbed_Gravity(2,1);
        linacc_z4Perturbed = linaccIMU4PerturbedValues(3,1) + IMU4_Perturbed_Gravity(3,1);
        
        %-------------------------------IMU5-------------------------------
        % Get the angular velocity of IMU5.
        ground_angvelIMU5Perturbed = IMU5.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU5: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU5Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU5Perturbed, IMU5);
        angvelIMU5PerturbedValues = [local_angvelIMU5Perturbed.get(0) local_angvelIMU5Perturbed.get(1) local_angvelIMU5Perturbed.get(2)]';

        % Determining the gravity vector expressed in IMU 5 frame
        % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
        local_IMU5_Gravity_Perturbed   = ground.expressVectorInAnotherFrame(stateToBePerturbed,GravityVec,IMU5);
        IMU5_Perturbed_Gravity         = [local_IMU5_Gravity_Perturbed.get(0) local_IMU5_Gravity_Perturbed.get(1) local_IMU5_Gravity_Perturbed.get(2)]';
        
        % Get the linear acceleration of IMU5.
        ground_linaccIMU5Perturbed = IMU5.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU5: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU5Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU5Perturbed, IMU5);
        linaccIMU5PerturbedValues = [local_linaccIMU5Perturbed.get(0) local_linaccIMU5Perturbed.get(1) local_linaccIMU5Perturbed.get(2)]';

        % Second set of outputs in EvaluateOP for determining H_current below.
        angvel_x5Perturbed = angvelIMU5PerturbedValues(1,1);
        angvel_y5Perturbed = angvelIMU5PerturbedValues(2,1);
        angvel_z5Perturbed = angvelIMU5PerturbedValues(3,1);

        linacc_x5Perturbed = linaccIMU5PerturbedValues(1,1) + IMU5_Perturbed_Gravity(1,1);
        linacc_y5Perturbed = linaccIMU5PerturbedValues(2,1) + IMU5_Perturbed_Gravity(2,1);
        linacc_z5Perturbed = linaccIMU5PerturbedValues(3,1) + IMU5_Perturbed_Gravity(3,1);
        
        %-------------------------------IMU6-------------------------------
        % Get the angular velocity of IMU6.
        ground_angvelIMU6Perturbed = IMU6.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU6: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU6Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU6Perturbed, IMU6);
        angvelIMU6PerturbedValues = [local_angvelIMU6Perturbed.get(0) local_angvelIMU6Perturbed.get(1) local_angvelIMU6Perturbed.get(2)]';

        % Determining the gravity vector expressed in IMU 6 frame
        % The gravity vector GravityVec in ground frame is equal to [0 9.81 0]
        local_IMU6_Gravity_Perturbed   = ground.expressVectorInAnotherFrame(stateToBePerturbed,GravityVec,IMU6);
        IMU6_Perturbed_Gravity         = [local_IMU6_Gravity_Perturbed.get(0) local_IMU6_Gravity_Perturbed.get(1) local_IMU6_Gravity_Perturbed.get(2)]';
        
        % Get the linear acceleration of IMU6.
        ground_linaccIMU6Perturbed = IMU6.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU6: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU6Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU6Perturbed, IMU6);
        linaccIMU6PerturbedValues = [local_linaccIMU6Perturbed.get(0) local_linaccIMU6Perturbed.get(1) local_linaccIMU6Perturbed.get(2)]';

        % Second set of outputs in EvaluateOP for determining H_current below.
        angvel_x6Perturbed = angvelIMU6PerturbedValues(1,1);
        angvel_y6Perturbed = angvelIMU6PerturbedValues(2,1);
        angvel_z6Perturbed = angvelIMU6PerturbedValues(3,1);

        linacc_x6Perturbed = linaccIMU6PerturbedValues(1,1) + IMU6_Perturbed_Gravity(1,1);
        linacc_y6Perturbed = linaccIMU6PerturbedValues(2,1) + IMU6_Perturbed_Gravity(2,1);
        linacc_z6Perturbed = linaccIMU6PerturbedValues(3,1) + IMU6_Perturbed_Gravity(3,1);
        
        EvaluateOPperturb = [angvel_x1Perturbed; % Evaluate function after to pertubing
                             angvel_y1Perturbed;
                             angvel_z1Perturbed;
                             linacc_x1Perturbed;
                             linacc_y1Perturbed;
                             linacc_z1Perturbed;
                             angvel_x2Perturbed;
                             angvel_y2Perturbed;
                             angvel_z2Perturbed;
                             linacc_x2Perturbed;
                             linacc_y2Perturbed;
                             linacc_z2Perturbed;
                             angvel_x3Perturbed;
                             angvel_y3Perturbed;
                             angvel_z3Perturbed;
                             linacc_x3Perturbed;
                             linacc_y3Perturbed;
                             linacc_z3Perturbed;
                             angvel_x4Perturbed;
                             angvel_y4Perturbed;
                             angvel_z4Perturbed;
                             linacc_x4Perturbed;
                             linacc_y4Perturbed;
                             linacc_z4Perturbed;
                             angvel_x5Perturbed;
                             angvel_y5Perturbed;
                             angvel_z5Perturbed;
                             linacc_x5Perturbed;
                             linacc_y5Perturbed;
                             linacc_z5Perturbed;
                             angvel_x6Perturbed;
                             angvel_y6Perturbed;
                             angvel_z6Perturbed;
                             linacc_x6Perturbed;
                             linacc_y6Perturbed;
                             linacc_z6Perturbed];
        if str2 == 'y'
            fprintf('Evaluation of function prior pertubing: Angular velocity    IMU1 in x = %16.15f\n', EvaluateOP(1,1))
            fprintf('                                      : Angular velocity    IMU1 in y = %16.15f\n', EvaluateOP(2,1))
            fprintf('                                      : Angular velocity    IMU1 in z = %16.15f\n', EvaluateOP(3,1))
            fprintf('                                      : Linear acceleration IMU1 in x = %16.15f\n', EvaluateOP(4,1))
            fprintf('                                      : Linear acceleration IMU1 in y = %16.15f\n', EvaluateOP(5,1))
            fprintf('                                      : Linear acceleration IMU1 in z = %16.15f\n', EvaluateOP(6,1))
            fprintf('                                      : Angular velocity    IMU2 in x = %16.15f\n', EvaluateOP(7,1))
            fprintf('                                      : Angular velocity    IMU2 in y = %16.15f\n', EvaluateOP(8,1))
            fprintf('                                      : Angular velocity    IMU2 in z = %16.15f\n', EvaluateOP(9,1))
            fprintf('                                      : Linear acceleration IMU2 in x = %16.15f\n', EvaluateOP(10,1))
            fprintf('                                      : Linear acceleration IMU2 in y = %16.15f\n', EvaluateOP(11,1))
            fprintf('                                      : Linear acceleration IMU2 in z = %16.15f\n', EvaluateOP(12,1))
            fprintf('                                      : Angular velocity    IMU3 in x = %16.15f\n', EvaluateOP(13,1))
            fprintf('                                      : Angular velocity    IMU3 in y = %16.15f\n', EvaluateOP(14,1))
            fprintf('                                      : Angular velocity    IMU3 in z = %16.15f\n', EvaluateOP(15,1))
            fprintf('                                      : Linear acceleration IMU3 in x = %16.15f\n', EvaluateOP(16,1))
            fprintf('                                      : Linear acceleration IMU3 in y = %16.15f\n', EvaluateOP(17,1))
            fprintf('                                      : Linear acceleration IMU3 in z = %16.15f\n', EvaluateOP(18,1))
            fprintf('                                      : Angular velocity    IMU4 in x = %16.15f\n', EvaluateOP(19,1))
            fprintf('                                      : Angular velocity    IMU4 in y = %16.15f\n', EvaluateOP(20,1))
            fprintf('                                      : Angular velocity    IMU4 in z = %16.15f\n', EvaluateOP(21,1))
            fprintf('                                      : Linear acceleration IMU4 in x = %16.15f\n', EvaluateOP(22,1))
            fprintf('                                      : Linear acceleration IMU4 in y = %16.15f\n', EvaluateOP(23,1))
            fprintf('                                      : Linear acceleration IMU4 in z = %16.15f\n', EvaluateOP(24,1))
            fprintf('                                      : Angular velocity    IMU5 in x = %16.15f\n', EvaluateOP(25,1))
            fprintf('                                      : Angular velocity    IMU5 in y = %16.15f\n', EvaluateOP(26,1))
            fprintf('                                      : Angular velocity    IMU5 in z = %16.15f\n', EvaluateOP(27,1))
            fprintf('                                      : Linear acceleration IMU5 in x = %16.15f\n', EvaluateOP(28,1))
            fprintf('                                      : Linear acceleration IMU5 in y = %16.15f\n', EvaluateOP(29,1))
            fprintf('                                      : Linear acceleration IMU5 in z = %16.15f\n', EvaluateOP(30,1))
            fprintf('                                      : Angular velocity    IMU6 in x = %16.15f\n', EvaluateOP(31,1))
            fprintf('                                      : Angular velocity    IMU6 in y = %16.15f\n', EvaluateOP(32,1))
            fprintf('                                      : Angular velocity    IMU6 in z = %16.15f\n', EvaluateOP(33,1))
            fprintf('                                      : Linear acceleration IMU6 in x = %16.15f\n', EvaluateOP(34,1))
            fprintf('                                      : Linear acceleration IMU6 in y = %16.15f\n', EvaluateOP(35,1))
            fprintf('                                      : Linear acceleration IMU6 in z = %16.15f\n', EvaluateOP(36,1))
                            
            fprintf('Evaluation of function after pertubing: Angular velocity    IMU1 in x = %16.15f\n', EvaluateOPperturb(1,1))
            fprintf('                                      : Angular velocity    IMU1 in y = %16.15f\n', EvaluateOPperturb(2,1))
            fprintf('                                      : Angular velocity    IMU1 in z = %16.15f\n', EvaluateOPperturb(3,1))
            fprintf('                                      : Linear acceleration IMU1 in x = %16.15f\n', EvaluateOPperturb(4,1))
            fprintf('                                      : Linear acceleration IMU1 in y = %16.15f\n', EvaluateOPperturb(5,1))
            fprintf('                                      : Linear acceleration IMU1 in z = %16.15f\n', EvaluateOPperturb(6,1))
            fprintf('                                      : Angular velocity    IMU2 in x = %16.15f\n', EvaluateOPperturb(7,1))
            fprintf('                                      : Angular velocity    IMU2 in y = %16.15f\n', EvaluateOPperturb(8,1))
            fprintf('                                      : Angular velocity    IMU2 in z = %16.15f\n', EvaluateOPperturb(9,1))
            fprintf('                                      : Linear acceleration IMU2 in x = %16.15f\n', EvaluateOPperturb(10,1))
            fprintf('                                      : Linear acceleration IMU2 in y = %16.15f\n', EvaluateOPperturb(11,1))
            fprintf('                                      : Linear acceleration IMU2 in z = %16.15f\n', EvaluateOPperturb(12,1))
            fprintf('                                      : Angular velocity    IMU3 in x = %16.15f\n', EvaluateOPperturb(13,1))
            fprintf('                                      : Angular velocity    IMU3 in y = %16.15f\n', EvaluateOPperturb(14,1))
            fprintf('                                      : Angular velocity    IMU3 in z = %16.15f\n', EvaluateOPperturb(15,1))
            fprintf('                                      : Linear acceleration IMU3 in x = %16.15f\n', EvaluateOPperturb(16,1))
            fprintf('                                      : Linear acceleration IMU3 in y = %16.15f\n', EvaluateOPperturb(17,1))
            fprintf('                                      : Linear acceleration IMU3 in z = %16.15f\n', EvaluateOPperturb(18,1))
            fprintf('                                      : Angular velocity    IMU4 in x = %16.15f\n', EvaluateOPperturb(19,1))
            fprintf('                                      : Angular velocity    IMU4 in y = %16.15f\n', EvaluateOPperturb(20,1))
            fprintf('                                      : Angular velocity    IMU4 in z = %16.15f\n', EvaluateOPperturb(21,1))
            fprintf('                                      : Linear acceleration IMU4 in x = %16.15f\n', EvaluateOPperturb(22,1))
            fprintf('                                      : Linear acceleration IMU4 in y = %16.15f\n', EvaluateOPperturb(23,1))
            fprintf('                                      : Linear acceleration IMU4 in z = %16.15f\n', EvaluateOPperturb(24,1))
            fprintf('                                      : Angular velocity    IMU5 in x = %16.15f\n', EvaluateOPperturb(25,1))
            fprintf('                                      : Angular velocity    IMU5 in y = %16.15f\n', EvaluateOPperturb(26,1))
            fprintf('                                      : Angular velocity    IMU5 in z = %16.15f\n', EvaluateOPperturb(27,1))
            fprintf('                                      : Linear acceleration IMU5 in x = %16.15f\n', EvaluateOPperturb(28,1))
            fprintf('                                      : Linear acceleration IMU5 in y = %16.15f\n', EvaluateOPperturb(29,1))
            fprintf('                                      : Linear acceleration IMU5 in z = %16.15f\n', EvaluateOPperturb(30,1))
            fprintf('                                      : Angular velocity    IMU6 in x = %16.15f\n', EvaluateOPperturb(31,1))
            fprintf('                                      : Angular velocity    IMU6 in y = %16.15f\n', EvaluateOPperturb(32,1))
            fprintf('                                      : Angular velocity    IMU6 in z = %16.15f\n', EvaluateOPperturb(33,1))
            fprintf('                                      : Linear acceleration IMU6 in x = %16.15f\n', EvaluateOPperturb(34,1))
            fprintf('                                      : Linear acceleration IMU6 in y = %16.15f\n', EvaluateOPperturb(35,1))
            fprintf('                                      : Linear acceleration IMU6 in z = %16.15f\n', EvaluateOPperturb(36,1))
        end
        
        % The elements of the Jacobian are computed as follows: J = ( (Evaluated Perturbed Operating Point) - (Evaluated Operating Point) ) / h
        J(1,i)  = (round(angvel_x1Perturbed,15)-round(angvel_x_IMU1,15))/h;
        J(2,i)  = (round(angvel_y1Perturbed,15)-round(angvel_y_IMU1,15))/h;
        J(3,i)  = (round(angvel_z1Perturbed,15)-round(angvel_z_IMU1,15))/h;
        J(4,i)  = (round(linacc_x1Perturbed,15)-round(linacc_x_IMU1,15))/h;
        J(5,i)  = (round(linacc_y1Perturbed,15)-round(linacc_y_IMU1,15))/h;
        J(6,i)  = (round(linacc_z1Perturbed,15)-round(linacc_z_IMU1,15))/h;
        J(7,i)  = (round(angvel_x2Perturbed,15)-round(angvel_x_IMU2,15))/h;
        J(8,i)  = (round(angvel_y2Perturbed,15)-round(angvel_y_IMU2,15))/h;
        J(9,i)  = (round(angvel_z2Perturbed,15)-round(angvel_z_IMU2,15))/h;
        J(10,i) = (round(linacc_x2Perturbed,15)-round(linacc_x_IMU2,15))/h;
        J(11,i) = (round(linacc_y2Perturbed,15)-round(linacc_y_IMU2,15))/h;
        J(12,i) = (round(linacc_z2Perturbed,15)-round(linacc_z_IMU2,15))/h;
        J(13,i) = (round(angvel_x3Perturbed,15)-round(angvel_x_IMU3,15))/h;
        J(14,i) = (round(angvel_y3Perturbed,15)-round(angvel_y_IMU3,15))/h;
        J(15,i) = (round(angvel_z3Perturbed,15)-round(angvel_z_IMU3,15))/h;
        J(16,i) = (round(linacc_x3Perturbed,15)-round(linacc_x_IMU3,15))/h;
        J(17,i) = (round(linacc_y3Perturbed,15)-round(linacc_y_IMU3,15))/h;
        J(18,i) = (round(linacc_z3Perturbed,15)-round(linacc_z_IMU3,15))/h;
        J(19,i) = (round(angvel_x4Perturbed,15)-round(angvel_x_IMU4,15))/h;
        J(20,i) = (round(angvel_y4Perturbed,15)-round(angvel_y_IMU4,15))/h;
        J(21,i) = (round(angvel_z4Perturbed,15)-round(angvel_z_IMU4,15))/h;
        J(22,i) = (round(linacc_x4Perturbed,15)-round(linacc_x_IMU4,15))/h;
        J(23,i) = (round(linacc_y4Perturbed,15)-round(linacc_y_IMU4,15))/h;
        J(24,i) = (round(linacc_z4Perturbed,15)-round(linacc_z_IMU4,15))/h;
        J(25,i) = (round(angvel_x5Perturbed,15)-round(angvel_x_IMU5,15))/h;
        J(26,i) = (round(angvel_y5Perturbed,15)-round(angvel_y_IMU5,15))/h;
        J(27,i) = (round(angvel_z5Perturbed,15)-round(angvel_z_IMU5,15))/h;
        J(28,i) = (round(linacc_x5Perturbed,15)-round(linacc_x_IMU5,15))/h;
        J(29,i) = (round(linacc_y5Perturbed,15)-round(linacc_y_IMU5,15))/h;
        J(30,i) = (round(linacc_z5Perturbed,15)-round(linacc_z_IMU5,15))/h;
        J(31,i) = (round(angvel_x6Perturbed,15)-round(angvel_x_IMU6,15))/h;
        J(32,i) = (round(angvel_y6Perturbed,15)-round(angvel_y_IMU6,15))/h;
        J(33,i) = (round(angvel_z6Perturbed,15)-round(angvel_z_IMU6,15))/h;
        J(34,i) = (round(linacc_x6Perturbed,15)-round(linacc_x_IMU6,15))/h;
        J(35,i) = (round(linacc_y6Perturbed,15)-round(linacc_y_IMU6,15))/h;
        J(36,i) = (round(linacc_z6Perturbed,15)-round(linacc_z_IMU6,15))/h;
        
        % Set operating point again equal to the normal operating point.
        % For the next i, (in the next loop) the next state will be
        % perturbed while the other states will now be kept at their
        % original values of the normal operating point.
        OPperturb(i)=OP(i);
    end
    
    J = round(J,7);
    IEKF.H_current(:,:,end+1) = J(:,:,end);
    
    if str2 == 'y'
        disp('IEKF.H_current =');
        disp(IEKF.H_current(:,:,end))
    
        fprintf('  ------------------------------------------------------------------------\n')
        fprintf('  |                     Ended determining H_current:                      |\n')
        fprintf('  ------------------------------------------------------------------------\n')
    
        fprintf('  ------------------------------------------------------------------------\n')
        fprintf('  |                         Determining M_current:                       |\n')
        fprintf('  ------------------------------------------------------------------------\n')
    end
    
    % Assumed is that the measurement noise is ADDITIVE!
    % Hence the M_current Jacobian will just have ones on the derivatives
    % where the noise affects the measurement, and zero otherwise:
    % Hence an identity matrix!
    
    % Set Jacobian M_current to the identity matrix.
    J1 = eye(numRows1);  

    IEKF.M_current(:,:,end+1) = J1(:,:,end);
    
    if str2 == 'y'
        fprintf('  ------------------------------------------------------------------------\n')
        fprintf('  |                     Ended determining H_current:                      |\n')
        fprintf('  ------------------------------------------------------------------------\n')
    end
    
      % ___________________________________________________________________
      % CHECK IF THE UNKNOWN INPUT TAU CAN BE ESTIMATED USING THIS ITERATED
      % EXTENDED KALMAN FILTER APPROACH!

      % Verhaegen and Verdult Filtering ans System Identification
      % Section 5.8 pages: 166-171
      
      % CHECK OBSERVABILITY OF ([A B] , [C 0]) A = F_prev, C = H_current
      %                        ([0 I]        )

      % A = IEKF.F_prev(:,:,end) and C = IEKF.H_current(:,:,end)
      % Then check the following requirement to see if the unknown input
      % tau: joint torque, can be estimated using
      %  the Kalman Filter if and only if 
      %      rank(obsv(A,C)) = n
      %           where n = [numRows,~]=size(A)
      %
      % CHECKED AND rank(obsv(A,C)) = 2 == 2 = [numRows,~]=size(A)
      %
      % Hence, the unknown input tau can be estimated using this
      % Kalman filter approach for this simple pendulum system! :)
      % ___________________________________________________________________
      
%     if str2 == 'y'
%         disp('P_prev')
%         disp(P_prev)
%     end
    
    % Compute the Innovation Covariance: S = H*P*H' + M*R*M'
    IEKF.S_current(:,:,end+1) = (IEKF.H_current(:,:,end)*P_prev*(IEKF.H_current(:,:,end)')) + (IEKF.M_current(:,:,end)*IEKF.R*(IEKF.M_current(:,:,end)'));
    
%     if str2 == 'y'
%         disp('Hcurrent * Pprev * Hcurrent^T:')
%         disp(IEKF.H_current(:,:,end)*P_prev*(IEKF.H_current(:,:,end)'))
%         disp('Mcurrent * R * Mcurrent^T:')
%         disp(IEKF.M_current(:,:,end)*IEKF.R*(IEKF.M_current(:,:,end)'))
%         disp('S')
%         disp(IEKF.S_current(:,:,end))
%         disp('S^-1')
%         disp(inv(IEKF.S_current(:,:,end)))
%     end

    % Compute the Kalman Gain:  K = P*H'*S^-1 => (P*H')*inv(S) => (P*H')/S
    IEKF.K_current(:,:,end+1) = P_prev * ( IEKF.H_current(:,:,end)' )/IEKF.S_current(:,:,end);
%     if str2 == 'y'
%         disp('Kalman gain K:')
%         disp(IEKF.K_current(:,:,end))
%     end
    
    % Compute the predicted measurement h(x_iter,0)
    IEKF.h_current(:,:,end+1) = [angvel_x_IMU1;
                                 angvel_y_IMU1;
                                 angvel_z_IMU1; 
                                 linacc_x_IMU1;
                                 linacc_y_IMU1;
                                 linacc_z_IMU1;
                                 angvel_x_IMU2;
                                 angvel_y_IMU2;
                                 angvel_z_IMU2;
                                 linacc_x_IMU2;
                                 linacc_y_IMU2;
                                 linacc_z_IMU2;
                                 angvel_x_IMU3;
                                 angvel_y_IMU3;
                                 angvel_z_IMU3;
                                 linacc_x_IMU3;
                                 linacc_y_IMU3;
                                 linacc_z_IMU3;
                                 angvel_x_IMU4;
                                 angvel_y_IMU4;
                                 angvel_z_IMU4;
                                 linacc_x_IMU4;
                                 linacc_y_IMU4;
                                 linacc_z_IMU4;
                                 angvel_x_IMU5;
                                 angvel_y_IMU5;
                                 angvel_z_IMU5;
                                 linacc_x_IMU5;
                                 linacc_y_IMU5;
                                 linacc_z_IMU5;
                                 angvel_x_IMU6;
                                 angvel_y_IMU6;
                                 angvel_z_IMU6;
                                 linacc_x_IMU6;
                                 linacc_y_IMU6;
                                 linacc_z_IMU6];
            
    % Compute the predicted linearized measurement hat{y} = h(x_iter,0) + ( H * ( x_prev - x_iter) ) where 0 indicates 0 noise
    IEKF.hat_y_localF(:,:,end+1) = IEKF.h_current(:,:,end) + ( IEKF.H_current(:,:,end) * ( x_prev - x_iter ) );
                            
     
    % Read the measured value for the  gyroscope and accelerometer
    % t+1 as the first entry has no measurement in the created measurement
    % data. (Measurements start at the second column: hence t+1 = 1+1 = 2)
    % Measurement Xsens IMU 1 angular velocity and linear acceleration
    yNoise_AngVel_1_localX = [rSensor.Xsens1_AngVel(4,t+1);
                              rSensor.Xsens1_AngVel(5,t+1);
                              rSensor.Xsens1_AngVel(6,t+1)];
                          
    yNoise_LinAcc_1_localX = [rSensor.Xsens1_LinAcc(4,t+1);
                              rSensor.Xsens1_LinAcc(5,t+1);
                              rSensor.Xsens1_LinAcc(6,t+1)];
    
    % Measurement Xsens IMU 2 angular velocity and linear acceleration
    yNoise_AngVel_2_localX = [rSensor.Xsens2_AngVel(4,t+1);
                              rSensor.Xsens2_AngVel(5,t+1);
                              rSensor.Xsens2_AngVel(6,t+1)];
                          
    yNoise_LinAcc_2_localX = [rSensor.Xsens2_LinAcc(4,t+1);
                              rSensor.Xsens2_LinAcc(5,t+1);
                              rSensor.Xsens2_LinAcc(6,t+1)];
                          
    % Measurement Xsens IMU 3 angular velocity and linear acceleration
    yNoise_AngVel_3_localX = [rSensor.Xsens3_AngVel(4,t+1);
                              rSensor.Xsens3_AngVel(5,t+1);
                              rSensor.Xsens3_AngVel(6,t+1)];
                          
    yNoise_LinAcc_3_localX = [rSensor.Xsens3_LinAcc(4,t+1);
                              rSensor.Xsens3_LinAcc(5,t+1);
                              rSensor.Xsens3_LinAcc(6,t+1)];
                          
    % Measurement Xsens IMU 4 angular velocity and linear acceleration
    yNoise_AngVel_4_localX = [rSensor.Xsens4_AngVel(4,t+1);
                              rSensor.Xsens4_AngVel(5,t+1);
                              rSensor.Xsens4_AngVel(6,t+1)];
                          
    yNoise_LinAcc_4_localX = [rSensor.Xsens4_LinAcc(4,t+1);
                              rSensor.Xsens4_LinAcc(5,t+1);
                              rSensor.Xsens4_LinAcc(6,t+1)];
    
    % Measurement Xsens IMU 5 angular velocity and linear acceleration
    yNoise_AngVel_5_localX = [rSensor.Xsens5_AngVel(4,t+1);
                              rSensor.Xsens5_AngVel(5,t+1);
                              rSensor.Xsens5_AngVel(6,t+1)];
                          
    yNoise_LinAcc_5_localX = [rSensor.Xsens5_LinAcc(4,t+1);
                              rSensor.Xsens5_LinAcc(5,t+1);
                              rSensor.Xsens5_LinAcc(6,t+1)];
                          
    % Measurement Xsens IMU 6 angular velocity and linear acceleration
    yNoise_AngVel_6_localX = [rSensor.Xsens6_AngVel(4,t+1);
                              rSensor.Xsens6_AngVel(5,t+1);
                              rSensor.Xsens6_AngVel(6,t+1)];
                          
    yNoise_LinAcc_6_localX = [rSensor.Xsens6_LinAcc(4,t+1);
                              rSensor.Xsens6_LinAcc(5,t+1);
                              rSensor.Xsens6_LinAcc(6,t+1)]; 
    
    y_localX = [yNoise_AngVel_1_localX;
                yNoise_LinAcc_1_localX;
                yNoise_AngVel_2_localX;
                yNoise_LinAcc_2_localX;
                yNoise_AngVel_3_localX;
                yNoise_LinAcc_3_localX;
                yNoise_AngVel_4_localX;
                yNoise_LinAcc_4_localX;
                yNoise_AngVel_5_localX;
                yNoise_LinAcc_5_localX;
                yNoise_AngVel_6_localX;
                yNoise_LinAcc_6_localX];
          
    % Compute the Innovation (error): e = y^localX - hat{y}^localF
    IEKF.e_current(:,:,end+1) = y_localX - IEKF.hat_y_localF(:,end);
    if str2 == 'y'
        disp('x_prev - x_iter = ')
        disp(x_prev - x_iter)
        disp('IEKF.H_current(:,:,end) * ( x_prev - x_iter ) =')
        disp(IEKF.H_current(:,:,end) * ( x_prev - x_iter ))
        disp('e_current')
        disp(IEKF.e_current(:,:,end))
    end  
    
    % Updating the state according to the Kalman gain and obtained measurement
    % x_update = x_prev + ( K * e )
    x_update = x_prev + ( IEKF.K_current(:,:,end) * IEKF.e_current(:,:,end) );
    if str2 == 'y'
        disp('x_prev')
        disp(x_prev(:,end))
        disp('x_iter')
        disp(x_iter(:,end))
        disp('x_update')
        disp(x_update(:,end))
    end
    
    IEKF.x_updsIter(:,end+1) = x_update;   
    
    if str2 == 'y'
        fprintf(' ---------------------------------------------------------\n')
        fprintf(' |              End IEKF update iteration: %d             |\n ', t)                   
        fprintf('---------------------------------------------------------\n ')
    end
    
end