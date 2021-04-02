function [Xsens, CalParam, Covariances] = CalibrateIMUData(SampleFreq, XsensRaw)

    % Create structs.
    Xsens       = struct;
    CalParam    = struct;
    Covariances = struct;

    
    %% Determine the covariance matrices of each IMU's gyroscope and accelerometer using stationary data
    
    %% IMU 1
    
    % Create the stationary data of IMU1
    % Gyroscope 1 stationary data
    GyroStat_X_1 = XsensRaw.StationaryDataIMU1{1,5}(:,1)';
    GyroStat_Y_1 = XsensRaw.StationaryDataIMU1{1,6}(:,1)';
    GyroStat_Z_1 = XsensRaw.StationaryDataIMU1{1,7}(:,1)';

    GyroStat_1 = [GyroStat_X_1;
                  GyroStat_Y_1;
                  GyroStat_Z_1];

    % Accelerometer 1 stationary data
    AccStat_X_1  = XsensRaw.StationaryDataIMU1{1,2}(:,1)';
    AccStat_Y_1  = XsensRaw.StationaryDataIMU1{1,3}(:,1)';
    AccStat_Z_1  = XsensRaw.StationaryDataIMU1{1,4}(:,1)';

    AccStat_1 =[AccStat_X_1;
                AccStat_Y_1;
                AccStat_Z_1];
            
    Covariances.Noise_Cov_Gyro_1 = (GyroStat_1 - mean(GyroStat_1,2))*(GyroStat_1 - mean(GyroStat_1,2))'/(size(GyroStat_1,2));
    Covariances.Noise_Cov_Acc_1  = (AccStat_1 - mean(AccStat_1,2))*(AccStat_1 - mean(AccStat_1,2))'/(size(AccStat_1,2));
    
    %% IMU 2
    
    % Create the stationary data of IMU2
    % Gyroscope 2 stationary data
    GyroStat_X_2 = XsensRaw.StationaryDataIMU2{1,5}(:,1)';
    GyroStat_Y_2 = XsensRaw.StationaryDataIMU2{1,6}(:,1)';
    GyroStat_Z_2 = XsensRaw.StationaryDataIMU2{1,7}(:,1)';

    GyroStat_2 = [GyroStat_X_2;
                  GyroStat_Y_2;
                  GyroStat_Z_2];

    % Accelerometer 2 stationary data
    AccStat_X_2  = XsensRaw.StationaryDataIMU2{1,2}(:,1)';
    AccStat_Y_2  = XsensRaw.StationaryDataIMU2{1,3}(:,1)';
    AccStat_Z_2  = XsensRaw.StationaryDataIMU2{1,4}(:,1)';

    AccStat_2 =[AccStat_X_2;
                AccStat_Y_2;
                AccStat_Z_2];
            
    Covariances.Noise_Cov_Gyro_2 = (GyroStat_2 - mean(GyroStat_2,2))*(GyroStat_2 - mean(GyroStat_2,2))'/(size(GyroStat_2,2));
    Covariances.Noise_Cov_Acc_2  = (AccStat_2 - mean(AccStat_2,2))*(AccStat_2 - mean(AccStat_2,2))'/(size(AccStat_2,2));
    
    %% IMU 3
    
    % Create the stationary data of IMU3
    % Gyroscope 3 stationary data
    GyroStat_X_3 = XsensRaw.StationaryDataIMU3{1,5}(:,1)';
    GyroStat_Y_3 = XsensRaw.StationaryDataIMU3{1,6}(:,1)';
    GyroStat_Z_3 = XsensRaw.StationaryDataIMU3{1,7}(:,1)';

    GyroStat_3 = [GyroStat_X_3;
                  GyroStat_Y_3;
                  GyroStat_Z_3];

    % Accelerometer 3 stationary data
    AccStat_X_3  = XsensRaw.StationaryDataIMU3{1,2}(:,1)';
    AccStat_Y_3  = XsensRaw.StationaryDataIMU3{1,3}(:,1)';
    AccStat_Z_3  = XsensRaw.StationaryDataIMU3{1,4}(:,1)';

    AccStat_3 =[AccStat_X_3;
                AccStat_Y_3;
                AccStat_Z_3];
            
    Covariances.Noise_Cov_Gyro_3 = (GyroStat_3 - mean(GyroStat_3,2))*(GyroStat_3 - mean(GyroStat_3,2))'/(size(GyroStat_3,2));
    Covariances.Noise_Cov_Acc_3  = (AccStat_3 - mean(AccStat_3,2))*(AccStat_3 - mean(AccStat_3,2))'/(size(AccStat_3,2));
    
    %% IMU 4
    
    % Create the stationary data of IMU4
    % Gyroscope 4 stationary data
    GyroStat_X_4 = XsensRaw.StationaryDataIMU4{1,5}(:,1)';
    GyroStat_Y_4 = XsensRaw.StationaryDataIMU4{1,6}(:,1)';
    GyroStat_Z_4 = XsensRaw.StationaryDataIMU4{1,7}(:,1)';

    GyroStat_4 = [GyroStat_X_4;
                  GyroStat_Y_4;
                  GyroStat_Z_4];

    % Accelerometer 4 stationary data
    AccStat_X_4  = XsensRaw.StationaryDataIMU4{1,2}(:,1)';
    AccStat_Y_4  = XsensRaw.StationaryDataIMU4{1,3}(:,1)';
    AccStat_Z_4  = XsensRaw.StationaryDataIMU4{1,4}(:,1)';

    AccStat_4 =[AccStat_X_4;
                AccStat_Y_4;
                AccStat_Z_4];
            
    Covariances.Noise_Cov_Gyro_4 = (GyroStat_4 - mean(GyroStat_4,2))*(GyroStat_4 - mean(GyroStat_4,2))'/(size(GyroStat_4,2));
    Covariances.Noise_Cov_Acc_4  = (AccStat_4 - mean(AccStat_4,2))*(AccStat_4 - mean(AccStat_4,2))'/(size(AccStat_4,2));
    
    %% IMU 5
    
    % Create the stationary data of IMU5
    % Gyroscope 5 stationary data
    GyroStat_X_5 = XsensRaw.StationaryDataIMU5{1,5}(:,1)';
    GyroStat_Y_5 = XsensRaw.StationaryDataIMU5{1,6}(:,1)';
    GyroStat_Z_5 = XsensRaw.StationaryDataIMU5{1,7}(:,1)';

    GyroStat_5 = [GyroStat_X_5;
                  GyroStat_Y_5;
                  GyroStat_Z_5];

    % Accelerometer 5 stationary data
    AccStat_X_5  = XsensRaw.StationaryDataIMU5{1,2}(:,1)';
    AccStat_Y_5  = XsensRaw.StationaryDataIMU5{1,3}(:,1)';
    AccStat_Z_5  = XsensRaw.StationaryDataIMU5{1,4}(:,1)';

    AccStat_5 =[AccStat_X_5;
                AccStat_Y_5;
                AccStat_Z_5];
            
    Covariances.Noise_Cov_Gyro_5 = (GyroStat_5 - mean(GyroStat_5,2))*(GyroStat_5 - mean(GyroStat_5,2))'/(size(GyroStat_5,2));
    Covariances.Noise_Cov_Acc_5  = (AccStat_5 - mean(AccStat_5,2))*(AccStat_5 - mean(AccStat_5,2))'/(size(AccStat_5,2));
    
    %% IMU 6
    
    % Create the stationary data of IMU6
    % Gyroscope 6 stationary data
    GyroStat_X_6 = XsensRaw.StationaryDataIMU6{1,5}(:,1)';
    GyroStat_Y_6 = XsensRaw.StationaryDataIMU6{1,6}(:,1)';
    GyroStat_Z_6 = XsensRaw.StationaryDataIMU6{1,7}(:,1)';

    GyroStat_6 = [GyroStat_X_6;
                  GyroStat_Y_6;
                  GyroStat_Z_6];

    % Accelerometer 6 stationary data
    AccStat_X_6  = XsensRaw.StationaryDataIMU6{1,2}(:,1)';
    AccStat_Y_6  = XsensRaw.StationaryDataIMU6{1,3}(:,1)';
    AccStat_Z_6  = XsensRaw.StationaryDataIMU6{1,4}(:,1)';

    AccStat_6 =[AccStat_X_6;
                AccStat_Y_6;
                AccStat_Z_6];
            
    Covariances.Noise_Cov_Gyro_6 = (GyroStat_6 - mean(GyroStat_6,2))*(GyroStat_6 - mean(GyroStat_6,2))'/(size(GyroStat_6,2));
    Covariances.Noise_Cov_Acc_6  = (AccStat_6 - mean(AccStat_6,2))*(AccStat_6 - mean(AccStat_6,2))'/(size(AccStat_6,2));
    
    %  End of determining the IMU's covariance matrices
    % _____________________________________________________________________
    
    
    
    
    
    
    %% Start estimation of calibration parameters and calibrating raw experimental IMU data.         
    % First,  the gyroscope calibration parameters are determined.
    % Second, the accelerometer calibration parameters are determined.
    % Third,  the raw experimental data is calibrated using the obtained parameters.
    

    %  ------------- Calibration Parameters for the gyroscopes ------------
    % For a stationary IMU, the bias of the gyroscope should be equal to the
    % mean of the STATIONARY gyroscope data:

    % IMU 1
    CalParam.Bias_Gyro_X_1 = mean(GyroStat_X_1);
    CalParam.Bias_Gyro_Y_1 = mean(GyroStat_Y_1);
    CalParam.Bias_Gyro_Z_1 = mean(GyroStat_Z_1);

    % IMU 2
    CalParam.Bias_Gyro_X_2 = mean(GyroStat_X_2);
    CalParam.Bias_Gyro_Y_2 = mean(GyroStat_Y_2);
    CalParam.Bias_Gyro_Z_2 = mean(GyroStat_Z_2);
    
    % IMU 3
    CalParam.Bias_Gyro_X_3 = mean(GyroStat_X_3);
    CalParam.Bias_Gyro_Y_3 = mean(GyroStat_Y_3);
    CalParam.Bias_Gyro_Z_3 = mean(GyroStat_Z_3);
    
    % IMU 4
    CalParam.Bias_Gyro_X_4 = mean(GyroStat_X_4);
    CalParam.Bias_Gyro_Y_4 = mean(GyroStat_Y_4);
    CalParam.Bias_Gyro_Z_4 = mean(GyroStat_Z_4);
    
    % IMU 5
    CalParam.Bias_Gyro_X_5 = mean(GyroStat_X_5);
    CalParam.Bias_Gyro_Y_5 = mean(GyroStat_Y_5);
    CalParam.Bias_Gyro_Z_5 = mean(GyroStat_Z_5);
    
    % IMU 6
    CalParam.Bias_Gyro_X_6 = mean(GyroStat_X_6);
    CalParam.Bias_Gyro_Y_6 = mean(GyroStat_Y_6);
    CalParam.Bias_Gyro_Z_6 = mean(GyroStat_Z_6);
    
    
    %  ----------- Calibration Parameters for the accelerometers ----------
    
    % Create the rotary IMU data for each accelerometer
    % Accelerometer 1 data
    AccRot_X_1 = XsensRaw.RotaryDataIMU1{1,2}(:,1)';
    AccRot_Y_1 = XsensRaw.RotaryDataIMU1{1,3}(:,1)';
    AccRot_Z_1 = XsensRaw.RotaryDataIMU1{1,4}(:,1)';
    
    RawAcc_1  = [AccRot_X_1;
                 AccRot_Y_1;
                 AccRot_Z_1];
    
    % Accelerometer 2 data
    AccRot_X_2 = XsensRaw.RotaryDataIMU2{1,2}(:,1)';
    AccRot_Y_2 = XsensRaw.RotaryDataIMU2{1,3}(:,1)';
    AccRot_Z_2 = XsensRaw.RotaryDataIMU2{1,4}(:,1)';
    
    RawAcc_2  = [AccRot_X_2;
                 AccRot_Y_2;
                 AccRot_Z_2];
    
    % Accelerometer 3 data
    AccRot_X_3 = XsensRaw.RotaryDataIMU3{1,2}(:,1)';
    AccRot_Y_3 = XsensRaw.RotaryDataIMU3{1,3}(:,1)';
    AccRot_Z_3 = XsensRaw.RotaryDataIMU3{1,4}(:,1)';
    
    RawAcc_3  = [AccRot_X_3;
                 AccRot_Y_3;
                 AccRot_Z_3];
    
    % Accelerometer 4 data
    AccRot_X_4 = XsensRaw.RotaryDataIMU4{1,2}(:,1)';
    AccRot_Y_4 = XsensRaw.RotaryDataIMU4{1,3}(:,1)';
    AccRot_Z_4 = XsensRaw.RotaryDataIMU4{1,4}(:,1)';
    
    RawAcc_4  = [AccRot_X_4;
                 AccRot_Y_4;
                 AccRot_Z_4];
    
    % Accelerometer 5 data
    AccRot_X_5 = XsensRaw.RotaryDataIMU5{1,2}(:,1)';
    AccRot_Y_5 = XsensRaw.RotaryDataIMU5{1,3}(:,1)';
    AccRot_Z_5 = XsensRaw.RotaryDataIMU5{1,4}(:,1)';
    
    RawAcc_5  = [AccRot_X_5;
                 AccRot_Y_5;
                 AccRot_Z_5];
    
    % Accelerometer 6 data
    AccRot_X_6 = XsensRaw.RotaryDataIMU6{1,2}(:,1)';
    AccRot_Y_6 = XsensRaw.RotaryDataIMU6{1,3}(:,1)';
    AccRot_Z_6 = XsensRaw.RotaryDataIMU6{1,4}(:,1)';
    
    RawAcc_6  = [AccRot_X_6;
                 AccRot_Y_6;
                 AccRot_Z_6];
    

    %  ----------- Calibration parameters of the accelerometers -----------
    % Determine the accelerometer parameters using the ellipsoid fitting method
    
    % IMU 1
    [Bias_Acc_1, Dinv_Acc_1] = AccelerometerEllipsoidFitting(RawAcc_1);
    % Save results to the struct CalParam
    CalParam.Bias_Acc_1 = Bias_Acc_1;
    CalParam.Dinv_Acc_1 = Dinv_Acc_1;

    % IMU 2
    [Bias_Acc_2, Dinv_Acc_2] = AccelerometerEllipsoidFitting(RawAcc_2);
    % Save results to the struct CalParam
    CalParam.Bias_Acc_2 = Bias_Acc_2;
    CalParam.Dinv_Acc_2 = Dinv_Acc_2;
    
    % IMU 3
    [Bias_Acc_3, Dinv_Acc_3] = AccelerometerEllipsoidFitting(RawAcc_3);
    % Save results to the struct CalParam
    CalParam.Bias_Acc_3 = Bias_Acc_3;
    CalParam.Dinv_Acc_3 = Dinv_Acc_3;
    
    % IMU 4
    [Bias_Acc_4, Dinv_Acc_4] = AccelerometerEllipsoidFitting(RawAcc_4);
    % Save results to the struct CalParam
    CalParam.Bias_Acc_4 = Bias_Acc_4;
    CalParam.Dinv_Acc_4 = Dinv_Acc_4;
    
    % IMU 5
    [Bias_Acc_5, Dinv_Acc_5] = AccelerometerEllipsoidFitting(RawAcc_5);
    % Save results to the struct CalParam
    CalParam.Bias_Acc_5 = Bias_Acc_5;
    CalParam.Dinv_Acc_5 = Dinv_Acc_5;
    
    % IMU 6
    [Bias_Acc_6, Dinv_Acc_6] = AccelerometerEllipsoidFitting(RawAcc_6);
    % Save results to the struct CalParam
    CalParam.Bias_Acc_6 = Bias_Acc_6;
    CalParam.Dinv_Acc_6 = Dinv_Acc_6;

    %  End of determining the calibration parameters
    % _____________________________________________________________________
    
    
    
    
    
    
    %% Load the experimental IMU data
    
    %% IMU 1
    % Gyroscope 1 data
    Gyro_X_1 = XsensRaw.IMU1{1,5}(:,1)';
    Gyro_Y_1 = XsensRaw.IMU1{1,6}(:,1)';
    Gyro_Z_1 = XsensRaw.IMU1{1,7}(:,1)';

    % Accelerometer 1 data
    Acc_X_1  = XsensRaw.IMU1{1,2}(:,1)';
    Acc_Y_1  = XsensRaw.IMU1{1,3}(:,1)';
    Acc_Z_1  = XsensRaw.IMU1{1,4}(:,1)';

    %% IMU 2
    % Gyroscope 2 data
    Gyro_X_2 = XsensRaw.IMU2{1,5}(:,1)';
    Gyro_Y_2 = XsensRaw.IMU2{1,6}(:,1)';
    Gyro_Z_2 = XsensRaw.IMU2{1,7}(:,1)';

    % Accelerometer 2 data
    Acc_X_2  = XsensRaw.IMU2{1,2}(:,1)';
    Acc_Y_2  = XsensRaw.IMU2{1,3}(:,1)';
    Acc_Z_2  = XsensRaw.IMU2{1,4}(:,1)';
    
    %% IMU 3
    % Gyroscope 3 data
    Gyro_X_3 = XsensRaw.IMU3{1,5}(:,1)';
    Gyro_Y_3 = XsensRaw.IMU3{1,6}(:,1)';
    Gyro_Z_3 = XsensRaw.IMU3{1,7}(:,1)';

    % Accelerometer 3 data
    Acc_X_3  = XsensRaw.IMU3{1,2}(:,1)';
    Acc_Y_3  = XsensRaw.IMU3{1,3}(:,1)';
    Acc_Z_3  = XsensRaw.IMU3{1,4}(:,1)';
    
    %% IMU 4
    % Gyroscope 4 data
    Gyro_X_4 = XsensRaw.IMU4{1,5}(:,1)';
    Gyro_Y_4 = XsensRaw.IMU4{1,6}(:,1)';
    Gyro_Z_4 = XsensRaw.IMU4{1,7}(:,1)';

    % Accelerometer 4 data
    Acc_X_4  = XsensRaw.IMU4{1,2}(:,1)';
    Acc_Y_4  = XsensRaw.IMU4{1,3}(:,1)';
    Acc_Z_4  = XsensRaw.IMU4{1,4}(:,1)';
    
    %% IMU 5
    % Gyroscope 5 data
    Gyro_X_5 = XsensRaw.IMU5{1,5}(:,1)';
    Gyro_Y_5 = XsensRaw.IMU5{1,6}(:,1)';
    Gyro_Z_5 = XsensRaw.IMU5{1,7}(:,1)';

    % Accelerometer 5 data
    Acc_X_5  = XsensRaw.IMU5{1,2}(:,1)';
    Acc_Y_5  = XsensRaw.IMU5{1,3}(:,1)';
    Acc_Z_5  = XsensRaw.IMU5{1,4}(:,1)';

    %% IMU 6
    % Gyroscope 6 data
    Gyro_X_6 = XsensRaw.IMU6{1,5}(:,1)';
    Gyro_Y_6 = XsensRaw.IMU6{1,6}(:,1)';
    Gyro_Z_6 = XsensRaw.IMU6{1,7}(:,1)';

    % Accelerometer 5 data
    Acc_X_6  = XsensRaw.IMU6{1,2}(:,1)';
    Acc_Y_6  = XsensRaw.IMU6{1,3}(:,1)';
    Acc_Z_6  = XsensRaw.IMU6{1,4}(:,1)';
    
    %  End of loading the experimental data of the IMUs.
    % _____________________________________________________________________
    
    
    
    
    
    
    %% Calibrate the raw experimental IMU data
    
    N = size(XsensRaw.IMU1{1, 2}, 1); % Number of measurements

    % Initializing struct variables for the * angular velocity
    %                                       * linear acceleration
    % --------------------------------IMU1---------------------------------
    Xsens.IMU1_AngVel          = zeros(3,N+1); % At t=0 no measurement, calibrated
    Xsens.IMU1_LinAcc          = zeros(3,N+1); % At t=0 no measurement, calibrated

    % --------------------------------IMU2---------------------------------
    Xsens.IMU2_AngVel          = zeros(3,N+1); % At t=0 no measurement, calibrated
    Xsens.IMU2_LinAcc          = zeros(3,N+1); % At t=0 no measurement, calibrated
    
    % --------------------------------IMU3---------------------------------
    Xsens.IMU3_AngVel          = zeros(3,N+1); % At t=0 no measurement, calibrated
    Xsens.IMU3_LinAcc          = zeros(3,N+1); % At t=0 no measurement, calibrated
    
    % --------------------------------IMU4---------------------------------
    Xsens.IMU4_AngVel          = zeros(3,N+1); % At t=0 no measurement, calibrated
    Xsens.IMU4_LinAcc          = zeros(3,N+1); % At t=0 no measurement, calibrated

    % --------------------------------IMU5---------------------------------
    Xsens.IMU5_AngVel          = zeros(3,N+1); % At t=0 no measurement, calibrated
    Xsens.IMU5_LinAcc          = zeros(3,N+1); % At t=0 no measurement, calibrated

    % --------------------------------IMU6---------------------------------
    Xsens.IMU6_AngVel          = zeros(3,N+1); % At t=0 no measurement, calibrated
    Xsens.IMU6_LinAcc          = zeros(3,N+1); % At t=0 no measurement, calibrated
    
%     %  T will store the time indices 
%     Xsens.T = [];
%     Xsens.T(1,end+1) = 0; % This is time t = 0
    

    % --------------------------------IMU1---------------------------------
    % Calibrate Gyroscope 1:
    Calibrated_AngVel_X_IMU1 = Gyro_X_1-CalParam.Bias_Gyro_X_1;
    Calibrated_AngVel_Y_IMU1 = Gyro_Y_1-CalParam.Bias_Gyro_Y_1;
    Calibrated_AngVel_Z_IMU1 = Gyro_Z_1-CalParam.Bias_Gyro_Z_1;

    % Gyroscope data, calibrated
    Xsens.IMU1_AngVel(1,2:N+1) = Calibrated_AngVel_X_IMU1; % Rotation around IMU X-Axes
    Xsens.IMU1_AngVel(2,2:N+1) = Calibrated_AngVel_Y_IMU1; % Rotation around IMU Y-Axes
    Xsens.IMU1_AngVel(3,2:N+1) = Calibrated_AngVel_Z_IMU1; % Rotation around IMU Z-Axes

    % Calibrate accelerometer 1
    % Calibrate Accelerometer: y_acc_calibrated =  Dinv * ( y_acc - y_bias_acc)
    Calibrated_LinAcc_IMU1 = CalParam.Dinv_Acc_1 * ( [Acc_X_1 - CalParam.Bias_Acc_1(1,1);
                                                      Acc_Y_1 - CalParam.Bias_Acc_1(2,1);
                                                      Acc_Z_1 - CalParam.Bias_Acc_1(3,1)] );

    % Accelerometer data, calibrated
    Xsens.IMU1_LinAcc(1,2:N+1) = Calibrated_LinAcc_IMU1(1,:); % Linear acceleration IMU X-Axes
    Xsens.IMU1_LinAcc(2,2:N+1) = Calibrated_LinAcc_IMU1(2,:); % Linear acceleration IMU Y-Axes
    Xsens.IMU1_LinAcc(3,2:N+1) = Calibrated_LinAcc_IMU1(3,:); % Linear acceleration IMU Z-Axes
    
    % --------------------------------IMU2---------------------------------
    % Calibrate Gyroscope 2:
    Calibrated_AngVel_X_IMU2 = Gyro_X_2-CalParam.Bias_Gyro_X_2;
    Calibrated_AngVel_Y_IMU2 = Gyro_Y_2-CalParam.Bias_Gyro_Y_2;
    Calibrated_AngVel_Z_IMU2 = Gyro_Z_2-CalParam.Bias_Gyro_Z_2;

    % Gyroscope data, calibrated
    Xsens.IMU2_AngVel(1,2:N+1) = Calibrated_AngVel_X_IMU2; % Rotation around IMU X-Axes
    Xsens.IMU2_AngVel(2,2:N+1) = Calibrated_AngVel_Y_IMU2; % Rotation around IMU Y-Axes
    Xsens.IMU2_AngVel(3,2:N+1) = Calibrated_AngVel_Z_IMU2; % Rotation around IMU Z-Axes

    % Calibrate accelerometer 2
    % Calibrate Accelerometer: y_acc_calibrated =  Dinv * ( y_acc - y_bias_acc)
    Calibrated_LinAcc_IMU2 = CalParam.Dinv_Acc_2 * ( [Acc_X_2 - CalParam.Bias_Acc_2(1,1);
                                                      Acc_Y_2 - CalParam.Bias_Acc_2(2,1);
                                                      Acc_Z_2 - CalParam.Bias_Acc_2(3,1)] );

    % Accelerometer data, calibrated
    Xsens.IMU2_LinAcc(1,2:N+1) = Calibrated_LinAcc_IMU2(1,:); % Linear acceleration IMU X-Axes
    Xsens.IMU2_LinAcc(2,2:N+1) = Calibrated_LinAcc_IMU2(2,:); % Linear acceleration IMU Y-Axes
    Xsens.IMU2_LinAcc(3,2:N+1) = Calibrated_LinAcc_IMU2(3,:); % Linear acceleration IMU Z-Axes
    
    % --------------------------------IMU3---------------------------------
    % Calibrate Gyroscope 3:
    Calibrated_AngVel_X_IMU3 = Gyro_X_3-CalParam.Bias_Gyro_X_3;
    Calibrated_AngVel_Y_IMU3 = Gyro_Y_3-CalParam.Bias_Gyro_Y_3;
    Calibrated_AngVel_Z_IMU3 = Gyro_Z_3-CalParam.Bias_Gyro_Z_3;

    % Gyroscope data, calibrated
    Xsens.IMU3_AngVel(1,2:N+1) = Calibrated_AngVel_X_IMU3; % Rotation around IMU X-Axes
    Xsens.IMU3_AngVel(2,2:N+1) = Calibrated_AngVel_Y_IMU3; % Rotation around IMU Y-Axes
    Xsens.IMU3_AngVel(3,2:N+1) = Calibrated_AngVel_Z_IMU3; % Rotation around IMU Z-Axes

    % Calibrate accelerometer 3
    % Calibrate Accelerometer: y_acc_calibrated =  Dinv * ( y_acc - y_bias_acc)
    Calibrated_LinAcc_IMU3 = CalParam.Dinv_Acc_3 * ( [Acc_X_3 - CalParam.Bias_Acc_3(1,1);
                                                      Acc_Y_3 - CalParam.Bias_Acc_3(2,1);
                                                      Acc_Z_3 - CalParam.Bias_Acc_3(3,1)] );

    % Accelerometer data, calibrated
    Xsens.IMU3_LinAcc(1,2:N+1) = Calibrated_LinAcc_IMU3(1,:); % Linear acceleration IMU X-Axes
    Xsens.IMU3_LinAcc(2,2:N+1) = Calibrated_LinAcc_IMU3(2,:); % Linear acceleration IMU Y-Axes
    Xsens.IMU3_LinAcc(3,2:N+1) = Calibrated_LinAcc_IMU3(3,:); % Linear acceleration IMU Z-Axes
    
    % --------------------------------IMU4---------------------------------
    % Calibrate Gyroscope 4:
    Calibrated_AngVel_X_IMU4 = Gyro_X_4-CalParam.Bias_Gyro_X_4;
    Calibrated_AngVel_Y_IMU4 = Gyro_Y_4-CalParam.Bias_Gyro_Y_4;
    Calibrated_AngVel_Z_IMU4 = Gyro_Z_4-CalParam.Bias_Gyro_Z_4;

    % Gyroscope data, calibrated
    Xsens.IMU4_AngVel(1,2:N+1) = Calibrated_AngVel_X_IMU4; % Rotation around IMU X-Axes
    Xsens.IMU4_AngVel(2,2:N+1) = Calibrated_AngVel_Y_IMU4; % Rotation around IMU Y-Axes
    Xsens.IMU4_AngVel(3,2:N+1) = Calibrated_AngVel_Z_IMU4; % Rotation around IMU Z-Axes

    % Calibrate accelerometer 4
    % Calibrate Accelerometer: y_acc_calibrated =  Dinv * ( y_acc - y_bias_acc)
    Calibrated_LinAcc_IMU4 = CalParam.Dinv_Acc_4 * ( [Acc_X_4 - CalParam.Bias_Acc_4(1,1);
                                                      Acc_Y_4 - CalParam.Bias_Acc_4(2,1);
                                                      Acc_Z_4 - CalParam.Bias_Acc_4(3,1)] );

    % Accelerometer data, calibrated
    Xsens.IMU4_LinAcc(1,2:N+1) = Calibrated_LinAcc_IMU4(1,:); % Linear acceleration IMU X-Axes
    Xsens.IMU4_LinAcc(2,2:N+1) = Calibrated_LinAcc_IMU4(2,:); % Linear acceleration IMU Y-Axes
    Xsens.IMU4_LinAcc(3,2:N+1) = Calibrated_LinAcc_IMU4(3,:); % Linear acceleration IMU Z-Axes
    
    % --------------------------------IMU5---------------------------------
    % Calibrate Gyroscope 5:
    Calibrated_AngVel_X_IMU5 = Gyro_X_5-CalParam.Bias_Gyro_X_5;
    Calibrated_AngVel_Y_IMU5 = Gyro_Y_5-CalParam.Bias_Gyro_Y_5;
    Calibrated_AngVel_Z_IMU5 = Gyro_Z_5-CalParam.Bias_Gyro_Z_5;

    % Gyroscope data, calibrated
    Xsens.IMU5_AngVel(1,2:N+1) = Calibrated_AngVel_X_IMU5; % Rotation around IMU X-Axes
    Xsens.IMU5_AngVel(2,2:N+1) = Calibrated_AngVel_Y_IMU5; % Rotation around IMU Y-Axes
    Xsens.IMU5_AngVel(3,2:N+1) = Calibrated_AngVel_Z_IMU5; % Rotation around IMU Z-Axes

    % Calibrate accelerometer 5
    % Calibrate Accelerometer: y_acc_calibrated =  Dinv * ( y_acc - y_bias_acc)
    Calibrated_LinAcc_IMU5 = CalParam.Dinv_Acc_5 * ( [Acc_X_5 - CalParam.Bias_Acc_5(1,1);
                                                      Acc_Y_5 - CalParam.Bias_Acc_5(2,1);
                                                      Acc_Z_5 - CalParam.Bias_Acc_5(3,1)] );

    % Accelerometer data, calibrated
    Xsens.IMU5_LinAcc(1,2:N+1) = Calibrated_LinAcc_IMU5(1,:); % Linear acceleration IMU X-Axes
    Xsens.IMU5_LinAcc(2,2:N+1) = Calibrated_LinAcc_IMU5(2,:); % Linear acceleration IMU Y-Axes
    Xsens.IMU5_LinAcc(3,2:N+1) = Calibrated_LinAcc_IMU5(3,:); % Linear acceleration IMU Z-Axes
    
    % --------------------------------IMU6---------------------------------
    % Calibrate Gyroscope 6:
    Calibrated_AngVel_X_IMU6 = Gyro_X_6-CalParam.Bias_Gyro_X_6;
    Calibrated_AngVel_Y_IMU6 = Gyro_Y_6-CalParam.Bias_Gyro_Y_6;
    Calibrated_AngVel_Z_IMU6 = Gyro_Z_6-CalParam.Bias_Gyro_Z_6;

    % Gyroscope data, calibrated
    Xsens.IMU6_AngVel(1,2:N+1) = Calibrated_AngVel_X_IMU6; % Rotation around IMU X-Axes
    Xsens.IMU6_AngVel(2,2:N+1) = Calibrated_AngVel_Y_IMU6; % Rotation around IMU Y-Axes
    Xsens.IMU6_AngVel(3,2:N+1) = Calibrated_AngVel_Z_IMU6; % Rotation around IMU Z-Axes

    % Calibrate accelerometer 6
    % Calibrate Accelerometer: y_acc_calibrated =  Dinv * ( y_acc - y_bias_acc)
    Calibrated_LinAcc_IMU6 = CalParam.Dinv_Acc_6 * ( [Acc_X_6 - CalParam.Bias_Acc_6(1,1);
                                                      Acc_Y_6 - CalParam.Bias_Acc_6(2,1);
                                                      Acc_Z_6 - CalParam.Bias_Acc_6(3,1)] );

    % Accelerometer data, calibrated
    Xsens.IMU6_LinAcc(1,2:N+1) = Calibrated_LinAcc_IMU6(1,:); % Linear acceleration IMU X-Axes
    Xsens.IMU6_LinAcc(2,2:N+1) = Calibrated_LinAcc_IMU6(2,:); % Linear acceleration IMU Y-Axes
    Xsens.IMU6_LinAcc(3,2:N+1) = Calibrated_LinAcc_IMU6(3,:); % Linear acceleration IMU Z-Axes
    
    %  End of calibrating the experimental data of the IMUs.
    % _____________________________________________________________________
    
    % -------------------------- End of function --------------------------
end
