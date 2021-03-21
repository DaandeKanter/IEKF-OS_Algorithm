function IEKF = IEKF_predict_step_part_2(IEKF, osimModel, state, t, str2)

    import org.opensim.modeling.*; % To import all OpenSim/Simbody functionality
    
    osimModel.realizeAcceleration(state);
    % Get the predicted states from the integrated EoM at the new time
    q     = state.getQ;
    q1    = q.get(0);
    q2    = q.get(1);
    
    u     = state.getU;
    u1    = u.get(0);
    u2    = u.get(1);
    
    qdot  = state.getQDot;
    qdot1 = qdot.get(0);
    qdot2 = qdot.get(1);
    
    udot  = state.getUDot;
    udot1 = udot.get(0);
    udot2 = udot.get(1);

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
    
    if str2 == 'y'
        fprintf('At time %6.4f seconds the values of the states are:\n', state.getTime)
        fprintf('[q1dot] = %16.15f , [ q1 ] = %16.15f\n', [qdot1,     q1]);
        fprintf('[q2dot] = %16.15f , [ q2 ] = %16.15f\n', [qdot2,     q2]);
        fprintf('[u1dot] = %16.15f , [ u1 ] = %16.15f\n', [udot1,     u1]);
        fprintf('[u2dot] = %16.15f , [ u2 ] = %16.15f\n', [udot2,     u2]);
        fprintf('                              [tau1] = %16.15f\n', tau1);
        fprintf('                              [tau2] = %16.15f\n', tau2);
    end
    
    % Set the states:
    x = [ q1 ;
          q2 ;
          u1 ;
          u2 ;
         tau1;
         tau2];

    P_prev = IEKF.P_upds(:,:,end); % Previous state covariance
%     if str2 == 'y' 
%         disp('P_prev')
%         disp(IEKF.P_upds(:,:,end))
%     end
    
    % Compute the predicted state covariance matrix P
    P = (IEKF.F_prev(:,:,end)*P_prev*(IEKF.F_prev(:,:,end)'))+(IEKF.L_prev(:,:,end)*IEKF.Q*(IEKF.L_prev(:,:,end)'));
    if str2 == 'y'
%         disp('Fprev * Pprev * Fprev^T: ')
%         disp(IEKF.F_prev(:,:,end)*P_prev*(IEKF.F_prev(:,:,end)'))
%         disp('Lprev * Q * Lprev^T: ')
%         disp(IEKF.L_prev(:,:,end)*IEKF.Q*(IEKF.L_prev(:,:,end)'))
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