%% Loading the initial state values from the KUKA joint encoder values
% All movements are sampled with a sampling frequency of 200 Hz.
% Thus downsampling required to match the frequency of 100 Hz of the Xsens
% IMUs.

function [Encoders, q_init] = LoadEncoderValues(Trial)

% _________________________________________________________________________
% First 7 columns are the joint angles from the joint encoders
q1_200Hz = Trial(:,1)';
q2_200Hz = Trial(:,2)';
q3_200Hz = Trial(:,3)';
q4_200Hz = Trial(:,4)';
q5_200Hz = Trial(:,5)';
q6_200Hz = Trial(:,6)';
q7_200Hz = Trial(:,7)';

% Subsequent 7 columns are the joint torques from the joint encoders
tau1_200Hz = Trial(:,8)';
tau2_200Hz = Trial(:,9)';
tau3_200Hz = Trial(:,10)';
tau4_200Hz = Trial(:,11)';
tau5_200Hz = Trial(:,12)';
tau6_200Hz = Trial(:,13)';
tau7_200Hz = Trial(:,14)';

% _________________________________________________________________________
% Store the values in the struct Encoders for later validation.
Encoders = struct;

Encoders.q1_200Hz = q1_200Hz;
Encoders.q2_200Hz = q2_200Hz;
Encoders.q3_200Hz = q3_200Hz;
Encoders.q4_200Hz = q4_200Hz;
Encoders.q5_200Hz = q5_200Hz;
Encoders.q6_200Hz = q6_200Hz;
Encoders.q7_200Hz = q7_200Hz;

Encoders.tau1_200Hz = tau1_200Hz;
Encoders.tau2_200Hz = tau2_200Hz;
Encoders.tau3_200Hz = tau3_200Hz;
Encoders.tau4_200Hz = tau4_200Hz;
Encoders.tau5_200Hz = tau5_200Hz;
Encoders.tau6_200Hz = tau6_200Hz;
Encoders.tau7_200Hz = tau7_200Hz;


% _________________________________________________________________________
% Decimate the data of the KUKA encoders sampled at 200 Hz to 100 Hz.
% Xsens IMUs sample at 100 Hz hence downsample the encoder values for later
% use.

% Reduces the sample rate of the encoders by a factor of 2 -> Hence 100 Hz.
% Decimate uses a lowpass Chebyshev Type I infinite impulse response (IIR) filter of order 8.
% Decimate the joint encoder values q:
Encoders.q1 = decimate(q1_200Hz, 2); % Data at 100Hz
Encoders.q2 = decimate(q2_200Hz, 2); % Data at 100Hz
Encoders.q3 = decimate(q3_200Hz, 2); % Data at 100Hz
Encoders.q4 = decimate(q4_200Hz, 2); % Data at 100Hz
Encoders.q5 = decimate(q5_200Hz, 2); % Data at 100Hz
Encoders.q6 = decimate(q6_200Hz, 2); % Data at 100Hz
Encoders.q7 = decimate(q7_200Hz, 2); % Data at 100Hz

% Decimate the torque encoder values tau:
Encoders.tau1 = decimate(tau1_200Hz, 2); % Data at 100Hz
Encoders.tau2 = decimate(tau2_200Hz, 2); % Data at 100Hz
Encoders.tau3 = decimate(tau3_200Hz, 2); % Data at 100Hz
Encoders.tau4 = decimate(tau4_200Hz, 2); % Data at 100Hz
Encoders.tau5 = decimate(tau5_200Hz, 2); % Data at 100Hz
Encoders.tau6 = decimate(tau6_200Hz, 2); % Data at 100Hz
Encoders.tau7 = decimate(tau7_200Hz, 2); % Data at 100Hz

% _________________________________________________________________________
% Store the initial values to set the OpenSim model accordingly.
% The begin pose should not deviate too much from the actual starting pose
% of the IEKF-OS KUKA system. Otherwise the filter might not converge.
q_init = [Encoders.q1(1,1) Encoders.q2(1,1) Encoders.q3(1,1) Encoders.q4(1,1) Encoders.q5(1,1) Encoders.q6(1,1) Encoders.q7(1,1)];

end