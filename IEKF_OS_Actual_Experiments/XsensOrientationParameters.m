%% KUKA experiment 04-12-2020

% Orientation calibration data to determine OpenSim rotation parameters.
% _________________________________________________________________________
% ----------------------------- VERY IMPORTANT ----------------------------
% Make sure that the encoders of the robot give joint angles of about 0.
% Hence, make sure that the robot is the upright fully extended position!
% _________________________________________________________________________

%% Script rotation matrix to Euler XYZ angles

% 1:  Calibrate the gyroscope -> stationary.
% 2:  Calibrate the accelerometer -> rotationary.
% 3:  Calibrate the magnetometer using the Xsens Magfield Mapper.
% 4:  Perform an inclination reset on the KUKA plateau.
%            % Roll and pitch should be approximately 0 now.
% 5:  Perfrom an heading reset on the KUKA plateau.
%            % Yaw should be approximately 0 now.
% 6:  Start recording.
% 7:  Rotate the sensor around the OpenSim Joint Center Z axis.
% 8:  Rotate the sensor around the OpenSim Joint Center X axis.
% 9:  When in final position stop recording and save the rotation matrix.
% 10: Open the last value of the rotation matrix and store these values
%     below in the matrix Rotm_KUKA_LinkX where X = 1:6.

% Fill in the entries of the Rotation Matrices in Rotm_KUKA_LinkX below as:
% [ Mat[1][1] Mat[2][1] Mat[3][1] ]
% [ Mat[2][1] Mat[2][2] Mat[3][2] ]
% [ Mat[3][1] Mat[3][2] Mat[3][3] ];

%% Create a struct to store all the rotation and translation parameters

Orientation = struct;

% Add the constant offset parameters from the left down corner to the
% accelerometer triad location. This information is obtained from
% the Xsens MTw manual page 64, section 11.1.5.
% Offset X expressed in OpenSim base frame with
% IMU pointing with the X upwards, Y to the left, and Z towards you.
% Hence, for the actual translations, check if these constants need to be
% added or subtracted!
% Express in meters!
Orientation.Xoffset = 0.0242; % Meters
Orientation.Yoffset = 0.0088; % Meters
Orientation.Zoffset = 0.0262; % Meters

%% KUKA Link 1
% Look at the last row of the file:
% Orientation_Calibration/Xsens1/OrientationData1.txt

% Determine the rotation parameters of Xsens IMU 1.
Rotm_KUKA_Link1 = [-0.009910 -0.999508 -0.029753
                   -0.219128  0.031202 -0.975197
                    0.975646 -0.003145 -0.291330];
eul_KUKA_Link1 = rotm2eul(Rotm_KUKA_Link1, 'XYZ');

Orientation.Xsens_1_Rot_X = eul_KUKA_Link1(1,1);
Orientation.Xsens_1_Rot_Y = eul_KUKA_Link1(1,2);
Orientation.Xsens_1_Rot_Z = eul_KUKA_Link1(1,3);

% Determine the translation parameters of Xsens IMU 1.
% XYZ measurements are approximately and measured towards the blue dot
% located in the left down corner of the IMU.

% Translations must be expressed in meters in OpenSim:
Orientation.Xsens_1_Trans_X = -0.015 + Orientation.Xoffset;
Orientation.Xsens_1_Trans_Y = -0.071 - Orientation.Yoffset;
Orientation.Xsens_1_Trans_Z =  0.005 + Orientation.Zoffset;

%% KUKA Link 2

% Look at the last row of the file:
% Orientation_Calibration/Xsens2/OrientationData2.txt

% Determine the rotation parameters of Xsens IMU 2.
Rotm_KUKA_Link2 = [-0.002486  0.999980 0.005814
                   -0.337802 -0.006312 0.941196
                    0.941214  0.000376 0.337811];
eul_KUKA_Link2 = rotm2eul(Rotm_KUKA_Link2, 'XYZ');

Orientation.Xsens_2_Rot_X = eul_KUKA_Link2(1,1);
Orientation.Xsens_2_Rot_Y = eul_KUKA_Link2(1,2);
Orientation.Xsens_2_Rot_Z = eul_KUKA_Link2(1,3);

% Determine the translation parameters of Xsens IMU 2.
% XYZ measurements are approximately and measured towards the blue dot
% located in the left down corner of the IMU.

% Translations must be expressed in meters in OpenSim:
Orientation.Xsens_2_Trans_X = 0.016 - Orientation.Xoffset;
Orientation.Xsens_2_Trans_Y = 0.085 + Orientation.Yoffset;
Orientation.Xsens_2_Trans_Z = 0.123 + Orientation.Zoffset;

%% KUKA Link 3

% Look at the last row of the file:
% Orientation_Calibration/Xsens3/OrientationData3.txt

% Determine the rotation parameters of Xsens IMU 3.
Rotm_KUKA_Link3 = [-0.035916 0.999355 -0.000436
                    0.167472 0.006448  0.985856
                    0.985222 0.035335 -0.167595];
eul_KUKA_Link3 = rotm2eul(Rotm_KUKA_Link3, 'XYZ');

Orientation.Xsens_3_Rot_X = eul_KUKA_Link3(1,1);
Orientation.Xsens_3_Rot_Y = eul_KUKA_Link3(1,2);
Orientation.Xsens_3_Rot_Z = eul_KUKA_Link3(1,3);

% Determine the translation parameters of Xsens IMU 3.
% XYZ measurements are approximately and measured towards the blue dot
% located in the left down corner of the IMU.

% Translations must be expressed in meters in OpenSim:
Orientation.Xsens_3_Trans_X = 0.013 - Orientation.Xoffset;
Orientation.Xsens_3_Trans_Y = 0.072 + Orientation.Yoffset;
Orientation.Xsens_3_Trans_Z = 0.032 + Orientation.Zoffset;

%% KUKA Link 4

% Look at the last row of the file:
% Orientation_Calibration/Xsens4/OrientationData4.txt

% Determine the rotation parameters of Xsens IMU 4.
Rotm_KUKA_Link4 = [-0.001750 -0.999702 -0.024337
                    0.278974  0.022883 -0.960026
                    0.960297 -0.008470  0.278851];
eul_KUKA_Link4 = rotm2eul(Rotm_KUKA_Link4, 'XYZ');

Orientation.Xsens_4_Rot_X = eul_KUKA_Link4(1,1);
Orientation.Xsens_4_Rot_Y = eul_KUKA_Link4(1,2);
Orientation.Xsens_4_Rot_Z = eul_KUKA_Link4(1,3);

% Determine the translation parameters of Xsens IMU 4.
% XYZ measurements are approximately and measured towards the blue dot
% located in the left down corner of the IMU.

% Translations must be expressed in meters in OpenSim:
Orientation.Xsens_4_Trans_X = -0.014 + Orientation.Xoffset;
Orientation.Xsens_4_Trans_Y = -0.082 - Orientation.Yoffset;
Orientation.Xsens_4_Trans_Z =  0.14  + Orientation.Zoffset;


%% KUKA Link 5

% Look at the last row of the file:
% Orientation_Calibration/Xsens5/OrientationData5.txt

% Determine the rotation parameters of Xsens IMU 5.
Rotm_KUKA_Link5 = [ 0.014934 -0.999874  0.005357
                   -0.234970 -0.008717 -0.971963
                    0.971888  0.013257 -0.235071];
eul_KUKA_Link5 = rotm2eul(Rotm_KUKA_Link5, 'XYZ');

Orientation.Xsens_5_Rot_X = eul_KUKA_Link5(1,1);
Orientation.Xsens_5_Rot_Y = eul_KUKA_Link5(1,2);
Orientation.Xsens_5_Rot_Z = eul_KUKA_Link5(1,3);

% Determine the translation parameters of Xsens IMU 5.
% XYZ measurements are approximately and measured towards the blue dot
% located in the left down corner of the IMU.

% Translations must be expressed in meters in OpenSim:
Orientation.Xsens_5_Trans_X = -0.017 + Orientation.Xoffset;
Orientation.Xsens_5_Trans_Y = -0.071 - Orientation.Yoffset;
Orientation.Xsens_5_Trans_Z =  0.041 + Orientation.Zoffset;

%% KUKA Link 6

% Look at the last row of the file:
% Orientation_Calibration/Xsens6/OrientationData6.txt

% Determine the rotation parameters of Xsens IMU 6.
Rotm_KUKA_Link6 = [-0.000394  0.999953 0.009680
                   -0.263590 -0.009442 0.964589
                    0.964635 -0.002172 0.263581];
eul_KUKA_Link6 = rotm2eul(Rotm_KUKA_Link6, 'XYZ');

Orientation.Xsens_6_Rot_X = eul_KUKA_Link6(1,1);
Orientation.Xsens_6_Rot_Y = eul_KUKA_Link6(1,2);
Orientation.Xsens_6_Rot_Z = eul_KUKA_Link6(1,3);

% Determine the translation parameters of Xsens IMU 6.
% XYZ measurements are approximately and measured towards the blue dot
% located in the left down corner of the IMU.

% Translations must be expressed in meters in OpenSim:
Orientation.Xsens_6_Trans_X = 0.016 - Orientation.Xoffset;
Orientation.Xsens_6_Trans_Y = 0.128 + Orientation.Yoffset;
Orientation.Xsens_6_Trans_Z = 0.028 + Orientation.Zoffset; % Value from Google Drive shifted 1 cm down, was 3,8 cm, now 2,8 cm

%% Save the parameters to a .mat file
save('XsensOrientationParametersForOpenSim.mat','Orientation')