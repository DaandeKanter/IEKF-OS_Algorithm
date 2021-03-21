function RMSD = RMSDcalculate(IEKF, Results)
%% Function computing the RMSE or equivalently the RMSD values
%  for the actual vs predicted states
RMSD = struct;

%% Reading data
% (76:end) is taken as the system as to initialize the simulation, hence the
% 0.76 first second up until the last simulated state variable is taken.
startPoint = 76;

% Joint angles
q1 = Results.q1(1,startPoint:end); 
q2 = Results.q2(1,startPoint:end);

% Generalized coordinates (angles of the joints)
q1hat = IEKF.x_upds(1,startPoint:end);
q2hat = IEKF.x_upds(2,startPoint:end);

% Joint angular velocities
u1 = Results.u1(1,startPoint:end);
u2 = Results.u2(1,startPoint:end);

% Generalized velocities (angular velocities of the joints)
u1hat = IEKF.x_upds(3,startPoint:end);
u2hat = IEKF.x_upds(4,startPoint:end);

% Joint torques
tau1 = Results.tau1(1,startPoint:end);
tau2 = Results.tau2(1,startPoint:end);

% Generalized joint torques (torques applied to the joints)
tau1hat = IEKF.x_upds(5,startPoint:end);
tau2hat = IEKF.x_upds(6,startPoint:end);

%% Compute the RMSD values in radians

fprintf('The RMSD values in radians are: \n')
% First joint angle RMSD value in radians
RMSD.q1 = sqrt(mean((q1 - q1hat).^2));
% Second joint angle RMSD value in radians
RMSD.q2 = sqrt(mean((q2 - q2hat).^2));
fprintf('q1 = %2.17f\n', RMSD.q1)
fprintf('q2 = %2.17f\n', RMSD.q2)


fprintf('The RMSD values in radians/second are: \n')
% First joint angular velocity RMSD value in radians/second
RMSD.u1 = sqrt(mean((u1 - u1hat).^2));
% Second joint angular velocity RMSD value in radians/second
RMSD.u2 = sqrt(mean((u2 - u2hat).^2));
fprintf('u1 = %2.17f\n', RMSD.u1)
fprintf('u2 = %2.17f\n', RMSD.u2)

fprintf('The RMSD values in Newton*meter are: \n')
% First joint torque RMSD value in Newton*meter
RMSD.tau1 = sqrt(mean((tau1 - tau1hat).^2)); 
% Second joint torque RMSD value Newton*meter
RMSD.tau2 = sqrt(mean((tau2 - tau2hat).^2));
fprintf('tau1 = %2.17f\n', RMSD.tau1)
fprintf('tau2 = %2.17f\n', RMSD.tau2)

%% Compute the RMSD values in degrees:

q1deg = rad2deg(Results.q1(1,startPoint:end));
q2deg = rad2deg(Results.q2(1,startPoint:end));

% Generalized coordinates (angles of the joints)
q1hatdeg = rad2deg(IEKF.x_upds(1,startPoint:end));
q2hatdeg = rad2deg(IEKF.x_upds(2,startPoint:end));

% Joint angular velocities
u1deg_s = rad2deg(Results.u1(1,startPoint:end));
u2deg_s = rad2deg(Results.u2(1,startPoint:end));

% Generalized velocities (angular velocities of the joints)
u1hatdeg_s = rad2deg(IEKF.x_upds(3,startPoint:end));
u2hatdeg_s = rad2deg(IEKF.x_upds(4,startPoint:end));

%% Compute the RMSD values

fprintf('The RMSD values in degrees are: \n')
% First joint angle RMSD value in degrees
RMSD.q1deg = sqrt(mean((q1deg - q1hatdeg).^2));
% Second joint angle RMSD value in degrees
RMSD.q2deg = sqrt(mean((q2deg - q2hatdeg).^2));
fprintf('q1 = %2.17f\n', RMSD.q1deg)
fprintf('q2 = %2.17f\n', RMSD.q2deg)

fprintf('The RMSD values in degrees/second are: \n')
% First joint angular velocity RMSD value in degrees/second
RMSD.u1deg_s = sqrt(mean((u1deg_s - u1hatdeg_s).^2));
% Second joint angular velocity RMSD value in degrees/second
RMSD.u2deg_s = sqrt(mean((u2deg_s - u2hatdeg_s).^2));
fprintf('u1 = %2.17f\n', RMSD.u1deg_s)
fprintf('u2 = %2.17f\n', RMSD.u2deg_s)

end