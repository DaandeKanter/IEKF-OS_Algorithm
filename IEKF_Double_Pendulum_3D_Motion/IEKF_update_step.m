function [IEKF, vSensor] = IEKF_update_step(IEKF, vSensor, rSensor, IMU1, IMU2, osimModel, state, t, torque1Gen, torque2Gen, ground, str2)
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
    osimModel.getCoordinateSet().get(0).setSpeedValue(state, x_iter(3,1));
    osimModel.getCoordinateSet().get(1).setSpeedValue(state, x_iter(4,1));
    torque1Gen.setValue(x_iter(5,1));
    torque2Gen.setValue(x_iter(6,1));
    
    % Checking if indeed the corect values have been set:
    q1Updated_set   = osimModel.getCoordinateSet().get(0).getValue(state);      % Set generalized coordinate 1
    q2Updated_set   = osimModel.getCoordinateSet().get(1).getValue(state);      % Set generalized coordinate 2
    u1Updated_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(state); % Set generalized velocity 1
    u2Updated_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(state); % Set generalized velocity 2
    tau1Updated_set = torque1Gen.getValue;                                      % Set joint torque 1
    tau2Updated_set = torque2Gen.getValue;                                      % Set joint torque 2
    
    if str2 == 'y'
        fprintf('Check: Value of   q1 should be set to the updated value of q1  : %16.15f\n', q1Updated_set)
        fprintf('Check: Value of   q2 should be set to the updated value of q2  : %16.15f\n', q2Updated_set)
        fprintf('Check: Value of   u1 should be set to the updated value of u1  : %16.15f\n', u1Updated_set)
        fprintf('Check: Value of   u2 should be set to the updated value of u2  : %16.15f\n', u2Updated_set)
        fprintf('Check: Value of tau1 should be set to the updated value of tau1: %16.15f\n', tau1Updated_set)
        fprintf('Check: Value of tau2 should be set to the updated value of tau2: %16.15f\n', tau2Updated_set)
    end
    
    % -----------------Reporting variables of attached IMUs----------------
    % h_t(x_t, e_t) = [omega_s]  where s denotes the s-th sensor.
    %                 [  a_s  ]
    
    % Again realize the acceleration stage to be able to compute Qdot and Udot correctly!
    osimModel.realizeAcceleration(state);

    
    % --------------------------------IMU1---------------------------------
    % Get the angular velocity of the IMU1.
    ground_IMU1_angvel = IMU1.getAngularVelocityInGround(state); % Get the values
    % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU1_angvel = ground.expressVectorInAnotherFrame(state, ground_IMU1_angvel, IMU1);
    local_IMU1_AngVel = [local_IMU1_angvel.get(0) local_IMU1_angvel.get(1) local_IMU1_angvel.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU1_AngVel(:,end+1) = local_IMU1_AngVel; % Store values in struct: "vSensor"
    
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
    
    % Get the linear acceleration of the IMU2.
    ground_IMU2_linacc = IMU2.getLinearAccelerationInGround(state); % Get the values
    % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
    local_IMU2_linacc = ground.expressVectorInAnotherFrame(state, ground_IMU2_linacc, IMU2);
    local_IMU2_LinAcc = [local_IMU2_linacc.get(0) local_IMU2_linacc.get(1) local_IMU2_linacc.get(2)]'; % Convert Vec3 values to MATLAB array
    vSensor.IMU2_LinAcc(:,end+1) = local_IMU2_LinAcc; % Store values in struct: "vSensor"
    
    
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
    
    % First six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU1 = local_IMU1_AngVel(1,1);
    angvel_y_IMU1 = local_IMU1_AngVel(2,1);
    angvel_z_IMU1 = local_IMU1_AngVel(3,1);
    
    linacc_x_IMU1 = local_IMU1_LinAcc(1,1);
    linacc_y_IMU1 = local_IMU1_LinAcc(2,1);
    linacc_z_IMU1 = local_IMU1_LinAcc(3,1);
    
    % Last six outputs in EvaluateOP for determining H_current below.
    angvel_x_IMU2 = local_IMU2_AngVel(1,1);
    angvel_y_IMU2 = local_IMU2_AngVel(2,1);
    angvel_z_IMU2 = local_IMU2_AngVel(3,1);

    linacc_x_IMU2 = local_IMU2_LinAcc(1,1);
    linacc_y_IMU2 = local_IMU2_LinAcc(2,1);
    linacc_z_IMU2 = local_IMU2_LinAcc(3,1);
    
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
    
    EvaluateOP   = [angvel_x_IMU1;  % Evaluate function prior to pertubing
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
                    linacc_z_IMU2];
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
        osimModel.getCoordinateSet().get(0).setValue(stateToBePerturbed, OPperturb(1,1));          % Set generalized coordinate 1
        osimModel.getCoordinateSet().get(1).setValue(stateToBePerturbed, OPperturb(2,1));          % Set generalized coordinate 2
        osimModel.getCoordinateSet().get(0).setSpeedValue(stateToBePerturbed, OPperturb(3,1));     % Set generalized velocity 1
        osimModel.getCoordinateSet().get(1).setSpeedValue(stateToBePerturbed, OPperturb(4,1));     % Set generalized velocity 2
        torque1Gen.setValue(OPperturb(5,1));                                                       % Set joint torque 1
        torque2Gen.setValue(OPperturb(6,1));                                                       % Set joint torque 2
        
        % Checking if indeed the corect values have been set:
        q1Perturbed_set   = osimModel.getCoordinateSet().get(0).getValue(stateToBePerturbed);      % Set generalized coordinate 1
        q2Perturbed_set   = osimModel.getCoordinateSet().get(1).getValue(stateToBePerturbed);      % Set generalized coordinate 2
        u1Perturbed_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(stateToBePerturbed); % Set generalized velocity 1
        u2Perturbed_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(stateToBePerturbed); % Set generalized velocity 2
        tau1Perturbed_set = torque1Gen.getValue;                                                   % Set joint torque 1
        tau2Perturbed_set = torque2Gen.getValue;                                                   % Set joint torque 2
        
        if str2 == 'y'
            fprintf('Check: Value of   q1 should be set to the perturbed value of   q1: %16.15f\n', q1Perturbed_set)
            fprintf('Check: Value of   q2 should be set to the perturbed value of   q2: %16.15f\n', q2Perturbed_set)
            fprintf('Check: Value of   u1 should be set to the perturbed value of   u1: %16.15f\n', u1Perturbed_set)
            fprintf('Check: Value of   u2 should be set to the perturbed value of   u2: %16.15f\n', u2Perturbed_set)
            fprintf('Check: Value of tau1 should be set to the perturbed value of tau1: %16.15f\n', tau1Perturbed_set)
            fprintf('Check: Value of tau2 should be set to the perturbed value of tau2: %16.15f\n', tau2Perturbed_set)
        end
        
        % Again realize the acceleration stage to be able to compute Qdot and Udot correctly!
        osimModel.realizeAcceleration(stateToBePerturbed);
        
        %-------------------------------IMU1-------------------------------
        % Get the angular velocity of IMU1.
        ground_angvelIMU1Perturbed = IMU1.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU1Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU1Perturbed, IMU1);
        angvelIMU1PerturbedValues = [local_angvelIMU1Perturbed.get(0) local_angvelIMU1Perturbed.get(1) local_angvelIMU1Perturbed.get(2)]';

        % First set of three outputs in EvaluateOP for determining H_current below.
        angvel_x1Perturbed = angvelIMU1PerturbedValues(1,1);
        angvel_y1Perturbed = angvelIMU1PerturbedValues(2,1);
        angvel_z1Perturbed = angvelIMU1PerturbedValues(3,1);

        % Get the linear acceleration of IMU1.
        ground_linaccIMU1Perturbed = IMU1.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU1: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU1Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU1Perturbed, IMU1);
        linaccIMU1PerturbedValues = [local_linaccIMU1Perturbed.get(0) local_linaccIMU1Perturbed.get(1) local_linaccIMU1Perturbed.get(2)]';

        % Second set of two outputs in EvaluateOP for determining H_current below.
        linacc_x1Perturbed = linaccIMU1PerturbedValues(1,1);
        linacc_y1Perturbed = linaccIMU1PerturbedValues(2,1);
        linacc_z1Perturbed = linaccIMU1PerturbedValues(3,1);
        
        %-------------------------------IMU2-------------------------------
        % Get the angular velocity of IMU2.
        ground_angvelIMU2Perturbed = IMU2.getAngularVelocityInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_angvelIMU2Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_angvelIMU2Perturbed, IMU2);
        angvelIMU2PerturbedValues = [local_angvelIMU2Perturbed.get(0) local_angvelIMU2Perturbed.get(1) local_angvelIMU2Perturbed.get(2)]';

        % Third set of three outputs in EvaluateOP for determining H_current below.
        angvel_x2Perturbed = angvelIMU2PerturbedValues(1,1);
        angvel_y2Perturbed = angvelIMU2PerturbedValues(2,1);
        angvel_z2Perturbed = angvelIMU2PerturbedValues(3,1);

        % Get the linear acceleration of IMU2.
        ground_linaccIMU2Perturbed = IMU2.getLinearAccelerationInGround(stateToBePerturbed);
        % Re-express in local sensor frame of IMU2: Inputs: (state, the vector to be re-expressed, the frame in which the vector will be re-expressed)
        local_linaccIMU2Perturbed = ground.expressVectorInAnotherFrame(stateToBePerturbed, ground_linaccIMU2Perturbed, IMU2);
        linaccIMU2PerturbedValues = [local_linaccIMU2Perturbed.get(0) local_linaccIMU2Perturbed.get(1) local_linaccIMU2Perturbed.get(2)]';

        % Third set of three outputs in EvaluateOP for determining H_current below.
        linacc_x2Perturbed = linaccIMU2PerturbedValues(1,1);
        linacc_y2Perturbed = linaccIMU2PerturbedValues(2,1);
        linacc_z2Perturbed = linaccIMU2PerturbedValues(3,1);
        
        EvaluateOPperturb = [angvel_x1Perturbed;  % Evaluate function after to pertubing
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
                             linacc_z2Perturbed];
                         
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
                                 linacc_z_IMU2];
            
                                 
     % Compute the predicted linearized measurement hat{y} = h(x_iter,0) + ( H * ( x_prev - x_iter) ) where 0 indicates 0 noise
     IEKF.hat_y_localF(:,:,end+1) = IEKF.h_current(:,:,end) + ( IEKF.H_current(:,:,end) * ( x_prev - x_iter ) );


    % Read the measured value for the  gyroscope and accelerometer
    % t+1 as the first entry has no measurement in the created measurement
    % data. (Measurements start at the second column: hence t+1 = 1+1 = 2)
    % For using the measurements which are perturbed with Gaussian noise:
    yNoise_localX = [rSensor.Xsens1_AngVel(4,t+1);
                     rSensor.Xsens1_AngVel(5,t+1);
                     rSensor.Xsens1_AngVel(6,t+1);
                     rSensor.Xsens1_LinAcc(4,t+1);
                     rSensor.Xsens1_LinAcc(5,t+1);
                     rSensor.Xsens1_LinAcc(6,t+1);
                     rSensor.Xsens2_AngVel(4,t+1);
                     rSensor.Xsens2_AngVel(5,t+1);
                     rSensor.Xsens2_AngVel(6,t+1);
                     rSensor.Xsens2_LinAcc(4,t+1);
                     rSensor.Xsens2_LinAcc(5,t+1);
                     rSensor.Xsens2_LinAcc(6,t+1)];
                 
% % For using the measurements which are not perturbed with Gaussian noise:
%     yNoNoise_localX = [rSensor.Xsens1_AngVel(1,t+1);
%                        rSensor.Xsens1_AngVel(2,t+1);
%                        rSensor.Xsens1_AngVel(3,t+1);
%                        rSensor.Xsens1_LinAcc(1,t+1);
%                        rSensor.Xsens1_LinAcc(2,t+1);
%                        rSensor.Xsens1_LinAcc(3,t+1);
%                        rSensor.Xsens2_AngVel(1,t+1);
%                        rSensor.Xsens2_AngVel(2,t+1);
%                        rSensor.Xsens2_AngVel(3,t+1);
%                        rSensor.Xsens2_LinAcc(1,t+1);
%                        rSensor.Xsens2_LinAcc(2,t+1);
%                        rSensor.Xsens2_LinAcc(3,t+1)];

    % Note that the orientations of the Xsens frame and the virtual IMU
    % frame in real experiments could differ by translations and
    % orientations! As such: Model the virtual frames in the best agreement
    % with the Xsens frame locations and orientations. During this
    % simulation, both these frames are located and oriented in the same
    % way: Hence, localX = localF.
    % Compute the Innovation (error): e = y^localX - hat{y}^localF
    IEKF.e_current(:,:,end+1) = yNoise_localX - IEKF.hat_y_localF(:,end);
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