function IEKF = IEKF_predict_step_part_1(IEKF, osimModel, state, torque1Gen, torque2Gen, torque3Gen, torque4Gen, torque5Gen, torque6Gen, t, str2)
 
    import org.opensim.modeling.*; % To import all OpenSim/Simbody functionality
    
    %          [ q_prev ]
    % x_prev = [ u_prev ]
    %          [tau_prev] 
    % As q_prev, u_prev and tau_prev are stacked, get them as follows.
    % To get the updated values of the previous q of the state:
    IEKF.q_prev   = IEKF.x_upds(1:length(IEKF.q_init),end);
    % and to get the updated values of the previous u of the state:
    IEKF.u_prev   = IEKF.x_upds(length(IEKF.q_init)+1:(length(IEKF.q_init)+length(IEKF.u_init)),end);
    % and to get the updated values of the previous tau of the state:
    IEKF.tau_prev = IEKF.x_upds((length(IEKF.q_init)+length(IEKF.u_init)+1):end,end);
   
    if str2 == 'y'
        fprintf(' \n')
        fprintf('Get the value of  q  from the last IEKF updated state from the previous timestep: %16.15f\n', IEKF.q_prev)
        fprintf('Get the value of  u  from the last IEKF updated state from the previous timestep: %16.15f\n', IEKF.u_prev)
        fprintf('Get the value of tau from the last IEKF updated state from the previous timestep: %16.15f\n', IEKF.tau_prev)
    end
    
    % Setting the previous values of the states
    osimModel.getCoordinateSet().get(0).setValue(state, IEKF.q_prev(1,1));
    osimModel.getCoordinateSet().get(1).setValue(state, IEKF.q_prev(2,1));
    osimModel.getCoordinateSet().get(2).setValue(state, IEKF.q_prev(3,1));
    osimModel.getCoordinateSet().get(3).setValue(state, IEKF.q_prev(4,1));
    osimModel.getCoordinateSet().get(4).setValue(state, IEKF.q_prev(5,1));
    osimModel.getCoordinateSet().get(5).setValue(state, IEKF.q_prev(6,1));
    
    osimModel.getCoordinateSet().get(0).setSpeedValue(state, IEKF.u_prev(1,1));
    osimModel.getCoordinateSet().get(1).setSpeedValue(state, IEKF.u_prev(2,1));
    osimModel.getCoordinateSet().get(2).setSpeedValue(state, IEKF.u_prev(3,1));
    osimModel.getCoordinateSet().get(3).setSpeedValue(state, IEKF.u_prev(4,1));
    osimModel.getCoordinateSet().get(4).setSpeedValue(state, IEKF.u_prev(5,1));
    osimModel.getCoordinateSet().get(5).setSpeedValue(state, IEKF.u_prev(6,1));
    
    torque1Gen.setValue(IEKF.tau_prev(1,1));
    torque2Gen.setValue(IEKF.tau_prev(2,1));
    torque3Gen.setValue(IEKF.tau_prev(3,1));
    torque4Gen.setValue(IEKF.tau_prev(4,1));
    torque5Gen.setValue(IEKF.tau_prev(5,1));
    torque6Gen.setValue(IEKF.tau_prev(6,1));
    
    % Checking if indeed the corect values have been set:
    q1_set   = osimModel.getCoordinateSet().get(0).getValue(state);       % Set generalized coordinate 1
    q2_set   = osimModel.getCoordinateSet().get(1).getValue(state);       % Set generalized coordinate 2
    q3_set   = osimModel.getCoordinateSet().get(2).getValue(state);       % Set generalized coordinate 3
    q4_set   = osimModel.getCoordinateSet().get(3).getValue(state);       % Set generalized coordinate 4
    q5_set   = osimModel.getCoordinateSet().get(4).getValue(state);       % Set generalized coordinate 5
    q6_set   = osimModel.getCoordinateSet().get(5).getValue(state);       % Set generalized coordinate 6
    
    u1_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(state);  % Set generalized velocity 1
    u2_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(state);  % Set generalized velocity 2
    u3_set   = osimModel.getCoordinateSet().get(2).getSpeedValue(state);  % Set generalized velocity 3
    u4_set   = osimModel.getCoordinateSet().get(3).getSpeedValue(state);  % Set generalized velocity 4
    u5_set   = osimModel.getCoordinateSet().get(4).getSpeedValue(state);  % Set generalized velocity 5
    u6_set   = osimModel.getCoordinateSet().get(5).getSpeedValue(state);  % Set generalized velocity 6
    
    tau1_set = torque1Gen.getValue;                                       % Set joint torque 1
    tau2_set = torque2Gen.getValue;                                       % Set joint torque 2
    tau3_set = torque3Gen.getValue;                                       % Set joint torque 3
    tau4_set = torque4Gen.getValue;                                       % Set joint torque 4
    tau5_set = torque5Gen.getValue;                                       % Set joint torque 5
    tau6_set = torque6Gen.getValue;                                       % Set joint torque 6
    
    if str2 == 'y'
        fprintf('Check: Value of  q1  should be set to the previous IEKF updated value of   q1: %16.15f\n', q1_set)
        fprintf('Check: Value of  q2  should be set to the previous IEKF updated value of   q2: %16.15f\n', q2_set)
        fprintf('Check: Value of  q3  should be set to the previous IEKF updated value of   q3: %16.15f\n', q3_set)
        fprintf('Check: Value of  q4  should be set to the previous IEKF updated value of   q4: %16.15f\n', q4_set)
        fprintf('Check: Value of  q5  should be set to the previous IEKF updated value of   q5: %16.15f\n', q5_set)
        fprintf('Check: Value of  q6  should be set to the previous IEKF updated value of   q6: %16.15f\n', q6_set)
        fprintf('Check: Value of  u1  should be set to the previous IEKF updated value of   u1: %16.15f\n', u1_set)
        fprintf('Check: Value of  u2  should be set to the previous IEKF updated value of   u2: %16.15f\n', u2_set)
        fprintf('Check: Value of  u3  should be set to the previous IEKF updated value of   u3: %16.15f\n', u3_set)
        fprintf('Check: Value of  u4  should be set to the previous IEKF updated value of   u4: %16.15f\n', u4_set)
        fprintf('Check: Value of  u5  should be set to the previous IEKF updated value of   u5: %16.15f\n', u5_set)
        fprintf('Check: Value of  u6  should be set to the previous IEKF updated value of   u6: %16.15f\n', u6_set)
        fprintf('Check: Value of tau1 should be set to the previous IEKF updated value of tau1: %16.15f\n', tau1_set)
        fprintf('Check: Value of tau2 should be set to the previous IEKF updated value of tau2: %16.15f\n', tau2_set)
        fprintf('Check: Value of tau3 should be set to the previous IEKF updated value of tau3: %16.15f\n', tau3_set)
        fprintf('Check: Value of tau4 should be set to the previous IEKF updated value of tau4: %16.15f\n', tau4_set)
        fprintf('Check: Value of tau5 should be set to the previous IEKF updated value of tau5: %16.15f\n', tau5_set)
        fprintf('Check: Value of tau6 should be set to the previous IEKF updated value of tau6: %16.15f\n', tau6_set)
    end
    
    % Get q, u, qdot and udot at the current time
    osimModel.realizeAcceleration(state);  % Realize the acceleration stage as Qdot and Udot are to be computed
    
    qStates       = state.getQ;
    Q1State       = qStates.get(0);
    Q2State       = qStates.get(1);
    Q3State       = qStates.get(2);
    Q4State       = qStates.get(3);
    Q5State       = qStates.get(4);
    Q6State       = qStates.get(5);
    
    qDotStates    = state.getQDot;
    QDot1State    = qDotStates.get(0);
    QDot2State    = qDotStates.get(1);
    QDot3State    = qDotStates.get(2);
    QDot4State    = qDotStates.get(3);
    QDot5State    = qDotStates.get(4);
    QDot6State    = qDotStates.get(5);
    
    uStates       = state.getU;
    U1State       = uStates.get(0);
    U2State       = uStates.get(1);
    U3State       = uStates.get(2);
    U4State       = uStates.get(3);
    U5State       = uStates.get(4);
    U6State       = uStates.get(5);
    
    uDotStates    = state.getUDot;
    UDot1State    = uDotStates.get(0);
    UDot2State    = uDotStates.get(1);
    UDot3State    = uDotStates.get(2);
    UDot4State    = uDotStates.get(3);
    UDot5State    = uDotStates.get(4);
    UDot6State    = uDotStates.get(5);
    
    tau1State     = IEKF.tau_prev(1,1);
    tau2State     = IEKF.tau_prev(2,1);
    tau3State     = IEKF.tau_prev(3,1);
    tau4State     = IEKF.tau_prev(4,1);
    tau5State     = IEKF.tau_prev(5,1);
    tau6State     = IEKF.tau_prev(6,1);
    
    TauDot1State = 0;
    TauDot2State = 0;
    TauDot3State = 0;
    TauDot4State = 0;
    TauDot5State = 0;
    TauDot6State = 0;
    
    % ---------------------------------------------------------------------
    % Still to choose which update model for tau to use, taudot = 0
    %                                   or               taudot = w_tau
    % ---------------------------------------------------------------------
    
    % The control input tau is updated via a random-walk proces similar to
    % Filtering and System Identification: A Least Squares Approach.
    %                   Verhaegen and Verdult
    %                        Page 166-171
    % tau(k+1) = tau(k) + w_tau(k), where w_tau = Q_tau^(1/2)*w_tau^nor(k)
    %                               where w_tau^nor(k)~(0, eye(size(tau)))
    
%     tauDotState = tauState + IEKF.w_tau*randn(length(tauState),1);
    if str2 == 'y'
        fprintf('At time %6.4f seconds the values of the states are:\n', state.getTime)
        fprintf('[q1dot] = %16.15f , [ q1 ] = %16.15f\n', [QDot1State,     Q1State]);
        fprintf('[q2dot] = %16.15f , [ q2 ] = %16.15f\n', [QDot2State,     Q2State]);
        fprintf('[q3dot] = %16.15f , [ q3 ] = %16.15f\n', [QDot3State,     Q3State]);
        fprintf('[q4dot] = %16.15f , [ q4 ] = %16.15f\n', [QDot4State,     Q4State]);
        fprintf('[q5dot] = %16.15f , [ q5 ] = %16.15f\n', [QDot5State,     Q5State]);
        fprintf('[q6dot] = %16.15f , [ q6 ] = %16.15f\n', [QDot6State,     Q6State]);
        fprintf('[u1dot] = %16.15f , [ u1 ] = %16.15f\n', [UDot1State,     U1State]);
        fprintf('[u2dot] = %16.15f , [ u2 ] = %16.15f\n', [UDot2State,     U2State]);
        fprintf('[u3dot] = %16.15f , [ u3 ] = %16.15f\n', [UDot3State,     U3State]);
        fprintf('[u4dot] = %16.15f , [ u4 ] = %16.15f\n', [UDot4State,     U4State]);
        fprintf('[u5dot] = %16.15f , [ u5 ] = %16.15f\n', [UDot5State,     U5State]);
        fprintf('[u6dot] = %16.15f , [ u6 ] = %16.15f\n', [UDot6State,     U6State]);
        fprintf('                              [tau1] = %16.15f\n', tau1State);
        fprintf('                              [tau2] = %16.15f\n', tau2State);
        fprintf('                              [tau3] = %16.15f\n', tau3State);
        fprintf('                              [tau4] = %16.15f\n', tau4State);
        fprintf('                              [tau5] = %16.15f\n', tau5State);
        fprintf('                              [tau6] = %16.15f\n', tau6State);
        
        fprintf(' ---------------------------------------------------------\n')
        fprintf(' |                   Iteration: %d                        |\n ', t)                   
        fprintf('---------------------------------------------------------\n ')  

        fprintf('---------------------------------------------------------\n')
        fprintf(' |                 Determining F_prev:                   |\n')
        fprintf(' ---------------------------------------------------------\n')
    end
    
    % Setting the operation point for the pertubation method of F_prev estimation
    OP = [  q1_set;
            q2_set;
            q3_set;
            q4_set;
            q5_set;
            q6_set;
            u1_set;
            u2_set;
            u3_set;
            u4_set;
            u5_set;
            u6_set;
          tau1_set;
          tau2_set;
          tau3_set;
          tau4_set;
          tau5_set;
          tau6_set];
      
    if str2 == 'y'      
        disp('Operating point = ')
        disp(OP)
    end
    
    [numRows, ~] = size(OP);     % Give length of Operation Point vector
    EvaluateOP = [  QDot1State;  % Evaluate function prior to pertubing
                    QDot2State;
                    QDot3State;
                    QDot4State;
                    QDot5State;
                    QDot6State;
                    UDot1State;
                    UDot2State;
                    UDot3State;
                    UDot4State;
                    UDot5State;
                    UDot6State;
                  TauDot1State;
                  TauDot2State;
                  TauDot3State;
                  TauDot4State;
                  TauDot5State;
                  TauDot6State]; % Expectation of zero-mean Gaussian noise is E[w_tau] = 0
              
    if str2 == 'y'          
        disp('EvaluateOP = ')
        disp(EvaluateOP)          
    end
    
    h   = 1.e-8;                 % Pertubation step

    % Declaring the pertubation operation point for the pertubation method of F_prev estimation
    OPperturb = [  q1_set;
                   q2_set;
                   q3_set;
                   q4_set;
                   q5_set;
                   q6_set;
                   u1_set;
                   u2_set;
                   u3_set;
                   u4_set;
                   u5_set;
                   u6_set;
                 tau1_set;
                 tau2_set;
                 tau3_set;
                 tau4_set;
                 tau5_set;
                 tau6_set];
    
    % Allocate memory for the Jacobian F_prev
    J = NaN(numRows, numRows);
    
    for i = 1:numRows
        if str2 == 'y'
            fprintf('Perturbing state %u yielding OPperturb = \n', i)
        end
        OPperturb(i)=OPperturb(i)+h;
        if str2 == 'y'
            disp(OPperturb)
        end
        
        % First, create a new copy of the source object, not a reference to the original object: 
        % ( Don't change the actual state in the simulation of the pendulum!!!)
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
            fprintf('Check: Value of   q1 should be set to the perturbed value of     q1: %16.15f\n', q1Perturbed_set)
            fprintf('Check: Value of   q2 should be set to the perturbed value of     q2: %16.15f\n', q2Perturbed_set)
            fprintf('Check: Value of   q3 should be set to the perturbed value of     q3: %16.15f\n', q3Perturbed_set)
            fprintf('Check: Value of   q4 should be set to the perturbed value of     q4: %16.15f\n', q4Perturbed_set)
            fprintf('Check: Value of   q5 should be set to the perturbed value of     q5: %16.15f\n', q5Perturbed_set)
            fprintf('Check: Value of   q6 should be set to the perturbed value of     q6: %16.15f\n', q6Perturbed_set)
            fprintf('Check: Value of   u1 should be set to the perturbed value of     u1: %16.15f\n', u1Perturbed_set)
            fprintf('Check: Value of   u2 should be set to the perturbed value of     u2: %16.15f\n', u2Perturbed_set)
            fprintf('Check: Value of   u3 should be set to the perturbed value of     u3: %16.15f\n', u3Perturbed_set)
            fprintf('Check: Value of   u4 should be set to the perturbed value of     u4: %16.15f\n', u4Perturbed_set)
            fprintf('Check: Value of   u5 should be set to the perturbed value of     u5: %16.15f\n', u5Perturbed_set)
            fprintf('Check: Value of   u6 should be set to the perturbed value of     u6: %16.15f\n', u6Perturbed_set)
            fprintf('Check: Value of tau1 should be set to the perturbed value of   tau1: %16.15f\n', tau1Perturbed_set)
            fprintf('Check: Value of tau2 should be set to the perturbed value of   tau2: %16.15f\n', tau2Perturbed_set)
            fprintf('Check: Value of tau3 should be set to the perturbed value of   tau3: %16.15f\n', tau3Perturbed_set)
            fprintf('Check: Value of tau4 should be set to the perturbed value of   tau4: %16.15f\n', tau4Perturbed_set)
            fprintf('Check: Value of tau5 should be set to the perturbed value of   tau5: %16.15f\n', tau5Perturbed_set)
            fprintf('Check: Value of tau6 should be set to the perturbed value of   tau6: %16.15f\n', tau6Perturbed_set)
        end
        
        % Again realize the acceleration stage to be able to compute Qdot and Udot correctly!
        osimModel.realizeAcceleration(stateToBePerturbed);
        
        QDotPerturb        = stateToBePerturbed.getQDot;
        QDot1Perturb       = QDotPerturb.get(0);
        QDot2Perturb       = QDotPerturb.get(1);
        QDot3Perturb       = QDotPerturb.get(2);
        QDot4Perturb       = QDotPerturb.get(3);
        QDot5Perturb       = QDotPerturb.get(4);
        QDot6Perturb       = QDotPerturb.get(5);
        
        UDotPerturb        = stateToBePerturbed.getUDot;
        UDot1Perturb       = UDotPerturb.get(0);
        UDot2Perturb       = UDotPerturb.get(1);
        UDot3Perturb       = UDotPerturb.get(2);
        UDot4Perturb       = UDotPerturb.get(3);
        UDot5Perturb       = UDotPerturb.get(4);
        UDot6Perturb       = UDotPerturb.get(5);
        
        TauDot1Perturb = 0;
        TauDot2Perturb = 0;
        TauDot3Perturb = 0;
        TauDot4Perturb = 0;
        TauDot5Perturb = 0;
        TauDot6Perturb = 0;
        
        EvaluateOPperturb = [  QDot1Perturb;
                               QDot2Perturb;
                               QDot3Perturb;
                               QDot4Perturb;
                               QDot5Perturb;
                               QDot6Perturb;
                               UDot1Perturb;
                               UDot2Perturb;
                               UDot3Perturb;
                               UDot4Perturb;
                               UDot5Perturb;
                               UDot6Perturb;
                             TauDot1Perturb;
                             TauDot2Perturb;
                             TauDot3Perturb;
                             TauDot4Perturb;
                             TauDot5Perturb;
                             TauDot6Perturb];
                         
        if str2 == 'y'                   
            fprintf('Evaluation of function prior pertubing:   q1dot =   %16.15f\n', EvaluateOP(1,1))
            fprintf('                                      :   q2dot =   %16.15f\n', EvaluateOP(2,1))
            fprintf('                                      :   q3dot =   %16.15f\n', EvaluateOP(3,1))
            fprintf('                                      :   q4dot =   %16.15f\n', EvaluateOP(4,1))
            fprintf('                                      :   q5dot =   %16.15f\n', EvaluateOP(5,1))
            fprintf('                                      :   q6dot =   %16.15f\n', EvaluateOP(6,1))
            fprintf('                                      :   u1dot =   %16.15f\n', EvaluateOP(7,1))
            fprintf('                                      :   u2dot =   %16.15f\n', EvaluateOP(8,1))
            fprintf('                                      :   u3dot =   %16.15f\n', EvaluateOP(9,1))
            fprintf('                                      :   u4dot =   %16.15f\n', EvaluateOP(10,1))
            fprintf('                                      :   u5dot =   %16.15f\n', EvaluateOP(11,1))
            fprintf('                                      :   u6dot =   %16.15f\n', EvaluateOP(12,1))
            fprintf('                                      : tau1dot =   %16.15f\n', EvaluateOP(13,1))
            fprintf('                                      : tau2dot =   %16.15f\n', EvaluateOP(14,1))
            fprintf('                                      : tau3dot =   %16.15f\n', EvaluateOP(15,1))
            fprintf('                                      : tau4dot =   %16.15f\n', EvaluateOP(16,1))
            fprintf('                                      : tau5dot =   %16.15f\n', EvaluateOP(17,1))
            fprintf('                                      : tau6dot =   %16.15f\n', EvaluateOP(18,1))
        
            fprintf('Evaluation of function after pertubing:   q1dot =   %16.15f\n', EvaluateOPperturb(1,1))
            fprintf('                                      :   q2dot =   %16.15f\n', EvaluateOPperturb(2,1))
            fprintf('                                      :   q3dot =   %16.15f\n', EvaluateOPperturb(3,1))
            fprintf('                                      :   q4dot =   %16.15f\n', EvaluateOPperturb(4,1))
            fprintf('                                      :   q5dot =   %16.15f\n', EvaluateOPperturb(5,1))
            fprintf('                                      :   q6dot =   %16.15f\n', EvaluateOPperturb(6,1))
            fprintf('                                      :   u1dot =   %16.15f\n', EvaluateOPperturb(7,1))
            fprintf('                                      :   u2dot =   %16.15f\n', EvaluateOPperturb(8,1))
            fprintf('                                      :   u3dot =   %16.15f\n', EvaluateOPperturb(9,1))
            fprintf('                                      :   u4dot =   %16.15f\n', EvaluateOPperturb(10,1))
            fprintf('                                      :   u5dot =   %16.15f\n', EvaluateOPperturb(11,1))
            fprintf('                                      :   u6dot =   %16.15f\n', EvaluateOPperturb(12,1))
            fprintf('                                      : tau1dot =   %16.15f\n', EvaluateOPperturb(13,1))
            fprintf('                                      : tau2dot =   %16.15f\n', EvaluateOPperturb(14,1))
            fprintf('                                      : tau3dot =   %16.15f\n', EvaluateOPperturb(15,1))
            fprintf('                                      : tau4dot =   %16.15f\n', EvaluateOPperturb(16,1))
            fprintf('                                      : tau5dot =   %16.15f\n', EvaluateOPperturb(17,1))
            fprintf('                                      : tau6dot =   %16.15f\n', EvaluateOPperturb(18,1))
        end
        
        % The elements of the Jacobian are computed as follows: J = ( (Evaluated Perturbed Operating Point) - (Evaluated Operating Point) ) / h
        J(1,i)  = (round(QDot1Perturb,15)-round(QDot1State,15))/h;
        J(2,i)  = (round(QDot2Perturb,15)-round(QDot2State,15))/h;
        J(3,i)  = (round(QDot3Perturb,15)-round(QDot3State,15))/h;
        J(4,i)  = (round(QDot4Perturb,15)-round(QDot4State,15))/h;
        J(5,i)  = (round(QDot5Perturb,15)-round(QDot5State,15))/h;
        J(6,i)  = (round(QDot6Perturb,15)-round(QDot6State,15))/h;
        J(7,i)  = (round(UDot1Perturb,15)-round(UDot1State,15))/h;
        J(8,i)  = (round(UDot2Perturb,15)-round(UDot2State,15))/h;
        J(9,i)  = (round(UDot3Perturb,15)-round(UDot3State,15))/h;
        J(10,i) = (round(UDot4Perturb,15)-round(UDot4State,15))/h;
        J(11,i) = (round(UDot5Perturb,15)-round(UDot5State,15))/h;
        J(12,i) = (round(UDot6Perturb,15)-round(UDot6State,15))/h;
        J(13,i) = (round(TauDot1Perturb,15)-round(TauDot1State,15))/h;
        J(14,i) = (round(TauDot2Perturb,15)-round(TauDot2State,15))/h;
        J(15,i) = (round(TauDot3Perturb,15)-round(TauDot3State,15))/h;
        J(16,i) = (round(TauDot4Perturb,15)-round(TauDot4State,15))/h;
        J(17,i) = (round(TauDot5Perturb,15)-round(TauDot5State,15))/h;
        J(18,i) = (round(TauDot6Perturb,15)-round(TauDot6State,15))/h;
        
        % Set operating point again equal to the normal operating point.
        % For the next i, (in the next loop) the next state will be
        % perturbed while the other states will now be kept at their
        % original values of the normal operating point.
        OPperturb(i)=OP(i);
    end
    
    % Rounding the elements of the Jacobian to 7 decimals, one + 7 decimals
    % is precision of 8. As the decimals of (UDotPerturbed-Udot)/(10^-8):
    % -0.509224690977287 - -0.509224593043088=
    % By hand comes down to:   -0.000000097934199
    % Matlab due to precision: -0.00000009793419897263789
    % From which it can be seen that the last 8 decimals don't mean
    % anything.
    % By hand:
    % (-0.000000097934199) / (10^-8) = 9.7934199 = 1 digit + 7 decimals
    % Hence, in MATLAB perform:
    % (-0.00000009793419897263789) /(10^-8) = -9.793419900000000
    % Hence, round J of to 7 decimals.
    
    % Augment the last row of IEKF.F_prev with zeros as dottau=0 hence
    % no dependency on the states q, u and tau:
    J = round(J,7);
    F_continuous = J;
    
    if str2 == 'y' 
        disp('F_continuous = ')
        disp(F_continuous)
    end
    
    % Note that this is the Jacobian matrix of the continuous time system.
    % For the IEKF, the Jacobian matrix of the discrete time system is
    % required. This can be converted in one of two ways. One can create a 
    % discrete version of the Equations of Motion functions. Or one can
    % simply discretize the Jacobian as a usually-acceptable approximation.
    % Let the discrete-time Jacobian be F= exp(J*dTime) which discretizes
    % the Jacobian J of the continuous time system using the matrix
    % exponential which is accurate, but computationally expensive. (Other
    % methods also exist to discretize the Jacobian matrix!)
    F_discrete = expm(F_continuous*IEKF.dTime);
    
    if str2 == 'y' 
        disp('F_discrete = IEKF.F_prev =')
        disp(F_discrete)
    end
    
    IEKF.F_prev(:,:,end+1) = F_discrete;

    if str2 == 'y' 
        fprintf('------------------------------------------------------------------------\n')
        fprintf('|                       Ended determining F_prev:                       |\n')
        fprintf('------------------------------------------------------------------------\n')
    
        fprintf('  ------------------------------------------------------------------------\n')
        fprintf('  |                          Determining Lprev:                          |\n')
        fprintf('  ------------------------------------------------------------------------\n')
    end
    
    % Assumed is that the process noise is ADDITIVE!
    % Hence the L_prev Jacobian will be just an identity matrix with the
    % size of the number of states:
    
    % Allocate memory and set Jacobian L_prev to the identity matrix. 
    J1 = eye(length(IEKF.q_init) + length(IEKF.u_init) + length(IEKF.tau_init));
    L_continuous = J1;
    
    if str2 == 'y' 
        disp('L_continuous = ')
        disp(L_continuous)
    end
    
    L_discrete = expm(L_continuous*IEKF.dTime);
    
    if str2 == 'y' 
        disp('L_discrete = IEKF.L_prev = ')
        disp(L_discrete)
    end
    
    IEKF.L_prev(:,:,end+1) = L_discrete;
    
    if str2 == 'y' 
        fprintf('  ------------------------------------------------------------------------\n')
        fprintf('  |                       Ended determining Lprev:                       |\n')
        fprintf('  ------------------------------------------------------------------------\n')
    end
    
end