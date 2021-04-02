function RMSE = RMSEcalculate(IEKF, Encoders, Xsens)
%% Function computing the RMSE values
%  For the actual vs predicted states.

% RMSE values computed after the 1 second of IEKF initialization
% System sampled at 100 Hz, hence samples 101:end are selected.
StartValue = 101;
RMSE = struct;
EndValue = Xsens.TotalSimulationTimeValue;

%% Reading data for RMSE of joint angles

% IEKF estimated generalized coordinates (angles of the joints)
q1hat = IEKF.x_upds(1, StartValue:EndValue);
q2hat = IEKF.x_upds(2, StartValue:EndValue);
q3hat = IEKF.x_upds(3, StartValue:EndValue);
q4hat = IEKF.x_upds(4, StartValue:EndValue);
q5hat = IEKF.x_upds(5, StartValue:EndValue);
q6hat = IEKF.x_upds(6, StartValue:EndValue);

% Joint angles from the KUKA encoders
q1 = Encoders.q1(1, StartValue:EndValue);
q2 = Encoders.q2(1, StartValue:EndValue);
q3 = Encoders.q3(1, StartValue:EndValue);
q4 = Encoders.q4(1, StartValue:EndValue);
q5 = Encoders.q5(1, StartValue:EndValue);
q6 = Encoders.q6(1, StartValue:EndValue);

% Compute the RMSE values in radians:
fprintf('The joint angle RMSE values in radians are: \n')
% First joint angle RMSE value in radians
RMSE.q1 = sqrt(mean((q1 - q1hat).^2));
% Second joint angle RMSE value in radians
RMSE.q2 = sqrt(mean((q2 - q2hat).^2));
% Third joint angle RMSE value in radians
RMSE.q3 = sqrt(mean((q3 - q3hat).^2));
% Fourth joint angle RMSE value in radians
RMSE.q4 = sqrt(mean((q4 - q4hat).^2));
% Fifth joint angle RMSE value in radians
RMSE.q5 = sqrt(mean((q5 - q5hat).^2));
% Sixth joint angle RMSE value in radians
RMSE.q6 = sqrt(mean((q6 - q6hat).^2));
fprintf('q1 = %2.17f\n', RMSE.q1)
fprintf('q2 = %2.17f\n', RMSE.q2)
fprintf('q3 = %2.17f\n', RMSE.q3)
fprintf('q4 = %2.17f\n', RMSE.q4)
fprintf('q5 = %2.17f\n', RMSE.q5)
fprintf('q6 = %2.17f\n', RMSE.q6)

%% Reading data for RMSE of joint angular velocity
% IEKF estimated joint angular velocities
u1hat = IEKF.x_upds(7, StartValue:EndValue);
u2hat = IEKF.x_upds(8, StartValue:EndValue);
u3hat = IEKF.x_upds(9, StartValue:EndValue);
u4hat = IEKF.x_upds(10, StartValue:EndValue);
u5hat = IEKF.x_upds(11, StartValue:EndValue);
u6hat = IEKF.x_upds(12, StartValue:EndValue);

% Joint angular velocity from the KUKA encoders
u1 = Encoders.u1(1, StartValue:EndValue);
u2 = Encoders.u2(1, StartValue:EndValue);
u3 = Encoders.u3(1, StartValue:EndValue);
u4 = Encoders.u4(1, StartValue:EndValue);
u5 = Encoders.u5(1, StartValue:EndValue);
u6 = Encoders.u6(1, StartValue:EndValue);

% Compute the RMSE values in radians:
fprintf('The joint angular velocity RMSE values in radians/second are: \n')
% First joint angular velocity RMSE value in radians/second
RMSE.u1 = sqrt(mean((u1 - u1hat).^2));
% Second joint angular velocity RMSE value in radians/second
RMSE.u2 = sqrt(mean((u2 - u2hat).^2));
% Third joint angular velocity RMSE value in radians/second
RMSE.u3 = sqrt(mean((u3 - u3hat).^2));
% Fourth joint angular velocity RMSE value in radians/second
RMSE.u4 = sqrt(mean((u4 - u4hat).^2));
% Fifth joint angular velocity RMSE value in radians/second
RMSE.u5 = sqrt(mean((u5 - u5hat).^2));
% Sixth joint angular velocity RMSE value in radians/second
RMSE.u6 = sqrt(mean((u6 - u6hat).^2));
fprintf('u1 = %2.17f\n', RMSE.u1)
fprintf('u2 = %2.17f\n', RMSE.u2)
fprintf('u3 = %2.17f\n', RMSE.u3)
fprintf('u4 = %2.17f\n', RMSE.u4)
fprintf('u5 = %2.17f\n', RMSE.u5)
fprintf('u6 = %2.17f\n', RMSE.u6)
fprintf('\n')

% Compute the RMSE values in degrees:
fprintf('The joint angle RMSE values in degrees are: \n')
% First joint angle RMSE value in degrees
RMSE.q1deg = rad2deg(RMSE.q1);
% Second joint angle RMSE value in degrees
RMSE.q2deg = rad2deg(RMSE.q2);
% Third joint angle RMSE value in degrees
RMSE.q3deg = rad2deg(RMSE.q3);
% Fourth joint angle RMSE value in degrees
RMSE.q4deg = rad2deg(RMSE.q4);
% Fifth joint angle RMSE value in degrees
RMSE.q5deg = rad2deg(RMSE.q5);
% Sixth joint angle RMSE value in degrees
RMSE.q6deg = rad2deg(RMSE.q6);
fprintf('q1 = %2.17f\n', RMSE.q1deg)
fprintf('q2 = %2.17f\n', RMSE.q2deg)
fprintf('q3 = %2.17f\n', RMSE.q3deg)
fprintf('q4 = %2.17f\n', RMSE.q4deg)
fprintf('q5 = %2.17f\n', RMSE.q5deg)
fprintf('q6 = %2.17f\n', RMSE.q6deg)

% Compute the RMSE values in degrees/second:
fprintf('The joint angular velocity RMSE values in degrees/second are: \n')
% First joint angular velocity RMSE value in degrees/second
RMSE.u1deg = rad2deg(RMSE.u1);
% Second joint angular velocity RMSE value in degrees/second
RMSE.u2deg = rad2deg(RMSE.u2);
% Third joint angular velocity RMSE value in degrees/second
RMSE.u3deg = rad2deg(RMSE.u3);
% Fourth joint angular velocity RMSE value in degrees/second
RMSE.u4deg = rad2deg(RMSE.u4);
% Fifth joint angular velocity RMSE value in degrees/second
RMSE.u5deg = rad2deg(RMSE.u5);
% Sixth joint angular velocity RMSE value in degrees/second
RMSE.u6deg = rad2deg(RMSE.u6);
fprintf('u1 = %2.17f\n', RMSE.u1deg)
fprintf('u2 = %2.17f\n', RMSE.u2deg)
fprintf('u3 = %2.17f\n', RMSE.u3deg)
fprintf('u4 = %2.17f\n', RMSE.u4deg)
fprintf('u5 = %2.17f\n', RMSE.u5deg)
fprintf('u6 = %2.17f\n', RMSE.u6deg)

end