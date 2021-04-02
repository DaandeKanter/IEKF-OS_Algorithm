function RMSD = RMSDcalculate(IEKF, Results)
%% Function computing the RMSE or equivalently the RMSD values
%  for the actual vs predicted states

% RMSD values computed after the 1 second of IEKF initialization
% System sampled at 100 Hz, hence samples 101:end are selected.
StartValue = 101;
RMSD = struct;

%% Reading data
% Joint angles
q1 = Results.q1(1,StartValue:end);
q2 = Results.q2(1,StartValue:end);
q3 = Results.q3(1,StartValue:end);
q4 = Results.q4(1,StartValue:end);
q5 = Results.q5(1,StartValue:end);
q6 = Results.q6(1,StartValue:end);

% Generalized coordinates (angles of the joints)
q1hat = IEKF.x_upds(1,StartValue:end);
q2hat = IEKF.x_upds(2,StartValue:end);
q3hat = IEKF.x_upds(3,StartValue:end);
q4hat = IEKF.x_upds(4,StartValue:end);
q5hat = IEKF.x_upds(5,StartValue:end);
q6hat = IEKF.x_upds(6,StartValue:end);

% Joint angular velocities
u1 = Results.u1(1,StartValue:end);
u2 = Results.u2(1,StartValue:end);
u3 = Results.u3(1,StartValue:end);
u4 = Results.u4(1,StartValue:end);
u5 = Results.u5(1,StartValue:end);
u6 = Results.u6(1,StartValue:end);

% Generalized velocities (angular velocities of the joints)
u1hat = IEKF.x_upds(7,StartValue:end);
u2hat = IEKF.x_upds(8,StartValue:end);
u3hat = IEKF.x_upds(9,StartValue:end);
u4hat = IEKF.x_upds(10,StartValue:end);
u5hat = IEKF.x_upds(11,StartValue:end);
u6hat = IEKF.x_upds(12,StartValue:end);


%% Compute the RMSD values in radians

fprintf('The RMSD values in radians are: \n')
% First joint angle RMSD value in radians
RMSD.q1 = sqrt(mean((q1 - q1hat).^2));
% Second joint angle RMSD value in radians
RMSD.q2 = sqrt(mean((q2 - q2hat).^2));
% Third joint angle RMSD value in radians
RMSD.q3 = sqrt(mean((q3 - q3hat).^2));
% Fourth joint angle RMSD value in radians
RMSD.q4 = sqrt(mean((q4 - q4hat).^2));
% Fifth joint angle RMSD value in radians
RMSD.q5 = sqrt(mean((q5 - q5hat).^2));
% Sixth joint angle RMSD value in radians
RMSD.q6 = sqrt(mean((q6 - q6hat).^2));
fprintf('q1 = %2.17f\n', RMSD.q1)
fprintf('q2 = %2.17f\n', RMSD.q2)
fprintf('q3 = %2.17f\n', RMSD.q3)
fprintf('q4 = %2.17f\n', RMSD.q4)
fprintf('q5 = %2.17f\n', RMSD.q5)
fprintf('q6 = %2.17f\n', RMSD.q6)

fprintf('The RMSD values in radians/second are: \n')
% First joint angular velocity RMSD value in radians/second
RMSD.u1 = sqrt(mean((u1 - u1hat).^2));
% Second joint angular velocity RMSD value in radians/second
RMSD.u2 = sqrt(mean((u2 - u2hat).^2));
% Third joint angular velocity RMSD value in radians/second
RMSD.u3 = sqrt(mean((u3 - u3hat).^2));
% Fourth joint angular velocity RMSD value in radians/second
RMSD.u4 = sqrt(mean((u4 - u4hat).^2));
% Fifth joint angular velocity RMSD value in radians/second
RMSD.u5 = sqrt(mean((u5 - u5hat).^2));
% Sixth joint angular velocity RMSD value in radians/second
RMSD.u6 = sqrt(mean((u6 - u6hat).^2));
fprintf('u1 = %2.17f\n', RMSD.u1)
fprintf('u2 = %2.17f\n', RMSD.u2)
fprintf('u3 = %2.17f\n', RMSD.u3)
fprintf('u4 = %2.17f\n', RMSD.u4)
fprintf('u5 = %2.17f\n', RMSD.u5)
fprintf('u6 = %2.17f\n', RMSD.u6)

%% Compute the RMSD values in degrees:

q1deg = rad2deg(Results.q1(1,StartValue:end));
q2deg = rad2deg(Results.q2(1,StartValue:end));
q3deg = rad2deg(Results.q3(1,StartValue:end));
q4deg = rad2deg(Results.q4(1,StartValue:end));
q5deg = rad2deg(Results.q5(1,StartValue:end));
q6deg = rad2deg(Results.q6(1,StartValue:end));

% Generalized coordinates (angles of the joints)
q1hatdeg = rad2deg(IEKF.x_upds(1,StartValue:end));
q2hatdeg = rad2deg(IEKF.x_upds(2,StartValue:end));
q3hatdeg = rad2deg(IEKF.x_upds(3,StartValue:end));
q4hatdeg = rad2deg(IEKF.x_upds(4,StartValue:end));
q5hatdeg = rad2deg(IEKF.x_upds(5,StartValue:end));
q6hatdeg = rad2deg(IEKF.x_upds(6,StartValue:end));

% Joint angular velocities
u1deg_s = rad2deg(Results.u1(1,StartValue:end));
u2deg_s = rad2deg(Results.u2(1,StartValue:end));
u3deg_s = rad2deg(Results.u3(1,StartValue:end));
u4deg_s = rad2deg(Results.u4(1,StartValue:end));
u5deg_s = rad2deg(Results.u5(1,StartValue:end));
u6deg_s = rad2deg(Results.u6(1,StartValue:end));

% Generalized velocities (angular velocities of the joints)
u1hatdeg_s = rad2deg(IEKF.x_upds(7,StartValue:end));
u2hatdeg_s = rad2deg(IEKF.x_upds(8,StartValue:end));
u3hatdeg_s = rad2deg(IEKF.x_upds(9,StartValue:end));
u4hatdeg_s = rad2deg(IEKF.x_upds(10,StartValue:end));
u5hatdeg_s = rad2deg(IEKF.x_upds(11,StartValue:end));
u6hatdeg_s = rad2deg(IEKF.x_upds(12,StartValue:end));

%% Compute the RMSD values

fprintf('The RMSD values in degrees are: \n')
% First joint angle RMSD value in degrees
RMSD.q1deg = sqrt(mean((q1deg - q1hatdeg).^2));
% Second joint angle RMSD value in degrees
RMSD.q2deg = sqrt(mean((q2deg - q2hatdeg).^2));
% Third joint angle RMSD value in degrees
RMSD.q3deg = sqrt(mean((q3deg - q3hatdeg).^2));
% Fourth joint angle RMSD value in degrees
RMSD.q4deg = sqrt(mean((q4deg - q4hatdeg).^2));
% Fifth joint angle RMSD value in degrees
RMSD.q5deg = sqrt(mean((q5deg - q5hatdeg).^2));
% Sixth joint angle RMSD value in degrees
RMSD.q6deg = sqrt(mean((q6deg - q6hatdeg).^2));
fprintf('q1 = %2.17f\n', RMSD.q1deg)
fprintf('q2 = %2.17f\n', RMSD.q2deg)
fprintf('q3 = %2.17f\n', RMSD.q3deg)
fprintf('q4 = %2.17f\n', RMSD.q4deg)
fprintf('q5 = %2.17f\n', RMSD.q5deg)
fprintf('q6 = %2.17f\n', RMSD.q6deg)

fprintf('The RMSD values in degrees/second are: \n')
% First joint angular velocity RMSD value in degrees/second
RMSD.u1deg_s = sqrt(mean((u1deg_s - u1hatdeg_s).^2));
% Second joint angular velocity RMSD value in degrees/second
RMSD.u2deg_s = sqrt(mean((u2deg_s - u2hatdeg_s).^2));
% Third joint angular velocity RMSD value in degrees/second
RMSD.u3deg_s = sqrt(mean((u3deg_s - u3hatdeg_s).^2));
% Fourth joint angular velocity RMSD value in degrees/second
RMSD.u4deg_s = sqrt(mean((u4deg_s - u4hatdeg_s).^2));
% Fifth joint angular velocity RMSD value in degrees/second
RMSD.u5deg_s = sqrt(mean((u5deg_s - u5hatdeg_s).^2));
% Sixth joint angular velocity RMSD value in degrees/second
RMSD.u6deg_s = sqrt(mean((u6deg_s - u6hatdeg_s).^2));
fprintf('u1 = %2.17f\n', RMSD.u1deg_s)
fprintf('u2 = %2.17f\n', RMSD.u2deg_s)
fprintf('u3 = %2.17f\n', RMSD.u3deg_s)
fprintf('u4 = %2.17f\n', RMSD.u4deg_s)
fprintf('u5 = %2.17f\n', RMSD.u5deg_s)
fprintf('u6 = %2.17f\n', RMSD.u6deg_s)

end