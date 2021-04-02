function IEKF = IEKF_predict_step_part_2(IEKF, osimModel, state, t, str2)

    import org.opensim.modeling.*; % To import all OpenSim/Simbody functionality
    
    osimModel.realizeAcceleration(state);
    % Get the predicted states from the integrated EoM at the new time
    q     = state.getQ;
    q1    = q.get(0);
    q2    = q.get(1);
    q3    = q.get(2);
    q4    = q.get(3);
    q5    = q.get(4);
    q6    = q.get(5);
    
    u     = state.getU;
    u1    = u.get(0);
    u2    = u.get(1);
    u3    = u.get(2);
    u4    = u.get(3);
    u5    = u.get(4);
    u6    = u.get(5);
    
    qdot  = state.getQDot;
    qdot1 = qdot.get(0);
    qdot2 = qdot.get(1);
    qdot3 = qdot.get(2);
    qdot4 = qdot.get(3);
    qdot5 = qdot.get(4);
    qdot6 = qdot.get(5);
    
    udot  = state.getUDot;
    udot1 = udot.get(0);
    udot2 = udot.get(1);
    udot3 = udot.get(2);
    udot4 = udot.get(3);
    udot5 = udot.get(4);
    udot6 = udot.get(5);

    % When results are bad, choose a different tau model:
    % BUT change F_prev accordingly!
    % -------For tau_dot = w_tau, the following models can be chosen-------
    % Euler method for integration
    % Hence, tau_{k+1} = tau_{k} + tau_dot_{k}*dTime
    % tau = IEKF.tau_prev + (IEKF.dTime*IEKF.stdJointTorque*randn(length(IEKF.tau_init),1)); % Euler method for integration
    % tau = IEKF.tau_prev + (IEKF.stdJointTorque*randn(length(IEKF.tau_init),1));
    % tau = IEKF.stdJointTorque*randn(length(IEKF.tau_init),1);
    
    % _____________________________________________________________________
    % Above incorrect, as the noise is already specified in the IEKF.Q
    % matrix in the IEKF_init.m script!
    
    % -------For tau_dot = 0, the following model can be chosen--------
    % No random walk model, just taudot = 0 hence tau_{k+1} = tau{k}
    tau1  = IEKF.tau_prev(1,1);
    tau2  = IEKF.tau_prev(2,1);
    tau3  = IEKF.tau_prev(3,1);
    tau4  = IEKF.tau_prev(4,1);
    tau5  = IEKF.tau_prev(5,1);
    tau6  = IEKF.tau_prev(6,1);
    
    if str2 == 'y'
        fprintf('At time %6.4f seconds the values of the states are:\n', state.getTime)
        fprintf('[q1dot] = %16.15f , [ q1 ] = %16.15f\n', [qdot1,     q1]);
        fprintf('[q2dot] = %16.15f , [ q2 ] = %16.15f\n', [qdot2,     q2]);
        fprintf('[q3dot] = %16.15f , [ q3 ] = %16.15f\n', [qdot3,     q3]);
        fprintf('[q4dot] = %16.15f , [ q4 ] = %16.15f\n', [qdot4,     q4]);
        fprintf('[q5dot] = %16.15f , [ q5 ] = %16.15f\n', [qdot5,     q5]);
        fprintf('[q6dot] = %16.15f , [ q6 ] = %16.15f\n', [qdot6,     q6]);
        fprintf('[u1dot] = %16.15f , [ u1 ] = %16.15f\n', [udot1,     u1]);
        fprintf('[u2dot] = %16.15f , [ u2 ] = %16.15f\n', [udot2,     u2]);
        fprintf('[u3dot] = %16.15f , [ u3 ] = %16.15f\n', [udot3,     u3]);
        fprintf('[u4dot] = %16.15f , [ u4 ] = %16.15f\n', [udot4,     u4]);
        fprintf('[u5dot] = %16.15f , [ u5 ] = %16.15f\n', [udot5,     u5]);
        fprintf('[u6dot] = %16.15f , [ u6 ] = %16.15f\n', [udot6,     u6]);
        fprintf('                              [tau1] = %16.15f\n', tau1);
        fprintf('                              [tau2] = %16.15f\n', tau2);
        fprintf('                              [tau3] = %16.15f\n', tau3);
        fprintf('                              [tau4] = %16.15f\n', tau4);
        fprintf('                              [tau5] = %16.15f\n', tau5);
        fprintf('                              [tau6] = %16.15f\n', tau6);
    end
    
    % Set the states:
    x = [ q1 ;
          q2 ;
          q3 ;
          q4 ;
          q5 ;
          q6 ;
          u1 ;
          u2 ;
          u3 ;
          u4 ;
          u5 ;
          u6 ;
         tau1;
         tau2;
         tau3;
         tau4;
         tau5;
         tau6];

    P_prev = IEKF.P_upds(:,:,end); % Previous state covariance
    if str2 == 'y' 
        disp('P_prev')
        disp(IEKF.P_upds(:,:,end))
    end
    
    % Compute the predicted state covariance matrix P
    P = (IEKF.F_prev(:,:,end)*P_prev*(IEKF.F_prev(:,:,end)'))+(IEKF.L_prev(:,:,end)*IEKF.Q*(IEKF.L_prev(:,:,end)'));
    if str2 == 'y'
        disp('Fprev * Pprev * Fprev^T: ')
        disp(IEKF.F_prev(:,:,end)*P_prev*(IEKF.F_prev(:,:,end)'))
        disp('Lprev * Q * Lprev^T: ')
        disp(IEKF.L_prev(:,:,end)*IEKF.Q*(IEKF.L_prev(:,:,end)'))
        disp('P updated')
        disp(P)
    end
    
    % After the predicted x and P have been computed, store them in the struct as the latest predicted state 
    IEKF.x_preds(:,end+1) = x;
    IEKF.P_preds(:,:,end+1) = P;  
    IEKF.T(end+1) = t*IEKF.dTime;
    
    % Also set the 'updated' values for the new time step equal to the predicted ones
    IEKF.x_upds(:,end+1) = x;
    IEKF.P_upds(:,:,end+1) = P;
    % Also set the 'updated' iteration values for the new time step equal to the predicted ones
    IEKF.x_updsIter(:,end+1) = x;
end