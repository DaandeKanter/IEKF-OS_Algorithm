function IEKF = IEKF_predict_step_part_1(IEKF, osimModel, state, torque1Gen, torque2Gen, t, str2)
 
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
    osimModel.getCoordinateSet().get(0).setSpeedValue(state, IEKF.u_prev(1,1));
    osimModel.getCoordinateSet().get(1).setSpeedValue(state, IEKF.u_prev(2,1));
    torque1Gen.setValue(IEKF.tau_prev(1,1));
    torque2Gen.setValue(IEKF.tau_prev(2,1));
    
    % Checking if indeed the corect values have been set:
    q1_set   = osimModel.getCoordinateSet().get(0).getValue(state);       % Set generalized coordinate 1
    q2_set   = osimModel.getCoordinateSet().get(1).getValue(state);       % Set generalized coordinate 2
    u1_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(state);  % Set generalized velocity 1
    u2_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(state);  % Set generalized velocity 2
    tau1_set = torque1Gen.getValue;                                       % Set joint torque 1
    tau2_set = torque2Gen.getValue;                                       % Set joint torque 2
    
    if str2 == 'y'
        fprintf('Check: Value of  q1  should be set to the previous IEKF updated value of   q1: %16.15f\n', q1_set)
        fprintf('Check: Value of  q2  should be set to the previous IEKF updated value of   q2: %16.15f\n', q2_set)
        fprintf('Check: Value of  u1  should be set to the previous IEKF updated value of   u1: %16.15f\n', u1_set)
        fprintf('Check: Value of  u2  should be set to the previous IEKF updated value of   u2: %16.15f\n', u2_set)
        fprintf('Check: Value of tau1 should be set to the previous IEKF updated value of tau1: %16.15f\n', tau1_set)
        fprintf('Check: Value of tau2 should be set to the previous IEKF updated value of tau2: %16.15f\n', tau2_set)
    end
    
    % Get q, u, qdot and udot at the current time
    osimModel.realizeAcceleration(state);  % Realize the acceleration stage as Qdot and Udot are to be computed
    qStates       = state.getQ;
    Q1State       = qStates.get(0);
    Q2State       = qStates.get(1);
    
    qDotStates    = state.getQDot;
    QDot1State    = qDotStates.get(0);
    QDot2State    = qDotStates.get(1);
    
    uStates       = state.getU;
    U1State       = uStates.get(0);
    U2State       = uStates.get(1);
    
    uDotStates    = state.getUDot;
    UDot1State    = uDotStates.get(0);
    UDot2State    = uDotStates.get(1);
    
    tau1State     = IEKF.tau_prev(1,1);
    tau2State     = IEKF.tau_prev(2,1);
    
    TauDot1State = 0;
    TauDot2State = 0;

    % _____________________________________________________________________
    % The control input tau is updated via a random-walk proces similar to
    % Filtering and System Identification: A Least Squares Approach.
    %                   Verhaegen and Verdult
    %                        Page 166-171
    % tau(k+1) = tau(k) + w_tau(k), where w_tau = Q_tau^(1/2)*w_tau^nor(k)
    %                               where w_tau^nor(k)~(0, eye(size(tau)))
    % _____________________________________________________________________
    

    if str2 == 'y'
        fprintf('At time %6.4f seconds the values of the states are:\n', state.getTime)
        fprintf('[q1dot] = %16.15f , [ q1 ] = %16.15f\n', [QDot1State,     Q1State]);
        fprintf('[q2dot] = %16.15f , [ q2 ] = %16.15f\n', [QDot2State,     Q2State]); 
        fprintf('[u1dot] = %16.15f , [ u1 ] = %16.15f\n', [UDot1State,     U1State]);
        fprintf('[u2dot] = %16.15f , [ u2 ] = %16.15f\n', [UDot2State,     U2State]);
        fprintf('                              [tau1] = %16.15f\n', tau1State);
        fprintf('                              [tau2] = %16.15f\n', tau2State);
        
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
            u1_set;
            u2_set;
          tau1_set;
          tau2_set];
      
    if str2 == 'y'      
        disp('Operating point = ')
        disp(OP)
    end
    
    [numRows, ~] = size(OP);     % Give length of Operation Point vector
    EvaluateOP = [  QDot1State;  % Evaluate function prior to pertubing
                    QDot2State;
                    UDot1State;
                    UDot2State;
                  TauDot1State;
                  TauDot2State]; % Expectation of zero-mean Gaussian noise is E[w_tau] = 0
              
    if str2 == 'y'          
        disp('EvaluateOP = ')
        disp(EvaluateOP)          
    end
    
    h   = 1.e-8;                 % Pertubation step

    % Declaring the pertubation operation point for the pertubation method of F_prev estimation
    OPperturb = [  q1_set;
                   q2_set;
                   u1_set;
                   u2_set;
                 tau1_set;
                 tau2_set];
    
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
        osimModel.getCoordinateSet().get(0).setValue(stateToBePerturbed, OPperturb(1,1));      % Set generalized coordinate 1
        osimModel.getCoordinateSet().get(1).setValue(stateToBePerturbed, OPperturb(2,1));      % Set generalized coordinate 2
        osimModel.getCoordinateSet().get(0).setSpeedValue(stateToBePerturbed, OPperturb(3,1)); % Set generalized velocity 1
        osimModel.getCoordinateSet().get(1).setSpeedValue(stateToBePerturbed, OPperturb(4,1)); % Set generalized velocity 2
        torque1Gen.setValue(OPperturb(5,1));                                                   % Set joint torque 1
        torque2Gen.setValue(OPperturb(6,1));                                                   % Set joint torque 2
        
        % Checking if indeed the corect values have been set:
        q1Perturbed_set   = osimModel.getCoordinateSet().get(0).getValue(stateToBePerturbed);      % Set generalized coordinate 1
        q2Perturbed_set   = osimModel.getCoordinateSet().get(1).getValue(stateToBePerturbed);      % Set generalized coordinate 2
        u1Perturbed_set   = osimModel.getCoordinateSet().get(0).getSpeedValue(stateToBePerturbed); % Set generalized velocity 1
        u2Perturbed_set   = osimModel.getCoordinateSet().get(1).getSpeedValue(stateToBePerturbed); % Set generalized velocity 2
        tau1Perturbed_set = torque1Gen.getValue;                                                   % Set joint torque 1
        tau2Perturbed_set = torque2Gen.getValue;                                                   % Set joint torque 2
        
        if str2 == 'y'
            fprintf('Check: Value of   q1 should be set to the perturbed value of     q1: %16.15f\n', q1Perturbed_set)
            fprintf('Check: Value of   q2 should be set to the perturbed value of     q2: %16.15f\n', q2Perturbed_set)
            fprintf('Check: Value of   u1 should be set to the perturbed value of     u1: %16.15f\n', u1Perturbed_set)
            fprintf('Check: Value of   u2 should be set to the perturbed value of     u2: %16.15f\n', u2Perturbed_set)
            fprintf('Check: Value of tau1 should be set to the perturbed value of   tau1: %16.15f\n', tau1Perturbed_set)
            fprintf('Check: Value of tau2 should be set to the perturbed value of   tau2: %16.15f\n', tau2Perturbed_set)
        end
        
        % Again realize the acceleration stage to be able to compute Qdot and Udot correctly!
        osimModel.realizeAcceleration(stateToBePerturbed);
        QDotPerturb        = stateToBePerturbed.getQDot;
        QDot1Perturb       = QDotPerturb.get(0);
        QDot2Perturb       = QDotPerturb.get(1);

        UDotPerturb        = stateToBePerturbed.getUDot;
        UDot1Perturb       = UDotPerturb.get(0);
        UDot2Perturb       = UDotPerturb.get(1);

        TauDot1Perturb = 0;
        TauDot2Perturb = 0;

        
        EvaluateOPperturb = [  QDot1Perturb;
                               QDot2Perturb;
                               UDot1Perturb;
                               UDot2Perturb;
                             TauDot1Perturb;
                             TauDot2Perturb];
        if str2 == 'y'                   
            fprintf('Evaluation of function prior pertubing:   q1dot =   %16.15f\n', EvaluateOP(1,1))
            fprintf('                                      :   q2dot =   %16.15f\n', EvaluateOP(2,1))
            fprintf('                                      :   u1dot =   %16.15f\n', EvaluateOP(3,1))
            fprintf('                                      :   u2dot =   %16.15f\n', EvaluateOP(4,1))
            fprintf('                                      : tau1dot =   %16.15f\n', EvaluateOP(5,1))
            fprintf('                                      : tau2dot =   %16.15f\n', EvaluateOP(6,1))

        
            fprintf('Evaluation of function after pertubing:   q1dot =   %16.15f\n', EvaluateOPperturb(1,1))
            fprintf('                                      :   q2dot =   %16.15f\n', EvaluateOPperturb(2,1))
            fprintf('                                      :   u1dot =   %16.15f\n', EvaluateOPperturb(3,1))
            fprintf('                                      :   u2dot =   %16.15f\n', EvaluateOPperturb(4,1))
            fprintf('                                      : tau1dot =   %16.15f\n', EvaluateOPperturb(5,1))
            fprintf('                                      : tau2dot =   %16.15f\n', EvaluateOPperturb(6,1))
        end
        
        % The elements of the Jacobian are computed as follows: J = ( (Evaluated Perturbed Operating Point) - (Evaluated Operating Point) ) / h
        J(1,i) = (round(QDot1Perturb,15)-round(QDot1State,15))/h;
        J(2,i) = (round(QDot2Perturb,15)-round(QDot2State,15))/h;
        J(3,i) = (round(UDot1Perturb,15)-round(UDot1State,15))/h;
        J(4,i) = (round(UDot2Perturb,15)-round(UDot2State,15))/h;
        J(5,i) = (round(TauDot1Perturb,15)-round(TauDot1State,15))/h;
        J(6,i) = (round(TauDot2Perturb,15)-round(TauDot2State,15))/h;
        
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