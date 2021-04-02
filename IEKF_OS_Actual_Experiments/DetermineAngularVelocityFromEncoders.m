%% Determine the angular velocity from the joint encoder measurements
%  Finite differencing method
function Encoders = DetermineAngularVelocityFromEncoders(Encoders, Xsens)

    % First define the encoder measurement arrays from which the angular
    % velocity estimates will be determined.
    
    % Define joint angle encoder values q
    q1_Encoder = Encoders.q1(1,1:length(Xsens.T));
    q2_Encoder = Encoders.q2(1,1:length(Xsens.T));
    q3_Encoder = Encoders.q3(1,1:length(Xsens.T));
    q4_Encoder = Encoders.q4(1,1:length(Xsens.T));
    q5_Encoder = Encoders.q5(1,1:length(Xsens.T));
    q6_Encoder = Encoders.q6(1,1:length(Xsens.T));
    
    % Define joint angular velocity encoder values u
    u1_Encoder = zeros(1,length(Xsens.T));
    u2_Encoder = zeros(1,length(Xsens.T));
    u3_Encoder = zeros(1,length(Xsens.T));
    u4_Encoder = zeros(1,length(Xsens.T));
    u5_Encoder = zeros(1,length(Xsens.T));
    u6_Encoder = zeros(1,length(Xsens.T));
    
    
    for i =1:length(Xsens.T)-1
        u1_Encoder(1,i+1) = ( q1_Encoder(1,i+1)-q1_Encoder(1,i) ) / ( Xsens.T(1, i+1) - Xsens.T(1,i) );
        u2_Encoder(1,i+1) = ( q2_Encoder(1,i+1)-q2_Encoder(1,i) ) / ( Xsens.T(1, i+1) - Xsens.T(1,i) );
        u3_Encoder(1,i+1) = ( q3_Encoder(1,i+1)-q3_Encoder(1,i) ) / ( Xsens.T(1, i+1) - Xsens.T(1,i) );
        u4_Encoder(1,i+1) = ( q4_Encoder(1,i+1)-q4_Encoder(1,i) ) / ( Xsens.T(1, i+1) - Xsens.T(1,i) );
        u5_Encoder(1,i+1) = ( q5_Encoder(1,i+1)-q5_Encoder(1,i) ) / ( Xsens.T(1, i+1) - Xsens.T(1,i) );
        u6_Encoder(1,i+1) = ( q6_Encoder(1,i+1)-q6_Encoder(1,i) ) / ( Xsens.T(1, i+1) - Xsens.T(1,i) );
    end

    % Store the estimated velocities in the struct Encoders for later visualization.
    Encoders.u1 = u1_Encoder;
    Encoders.u2 = u2_Encoder;
    Encoders.u3 = u3_Encoder;
    Encoders.u4 = u4_Encoder;
    Encoders.u5 = u5_Encoder;
    Encoders.u6 = u6_Encoder;
end