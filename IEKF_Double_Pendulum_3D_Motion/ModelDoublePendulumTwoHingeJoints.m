%% This script creates the models which can be used for simulation later on
% ModelDoublePendulumTwoHingeJoints.m creates the double pendulum model and the model: ModelDoublePendulumTwoHingeJoints.osim
 
%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*
modelFilename = 'ModelDoublePendulumTwoHingeJoints.osim';

%% Instantiate an (empty) OpenSim Model
model = Model();
model.setName('ModelDoublePendulumTwoHingeJoints');

% Get a reference to the ground object
ground = model.getGround();

%% Create the bodies
% Make and add a Upper Pendulum Body
Pendulum1 = Body();
Pendulum1.setName('UpperPendulum');
% "The mass of the body (kg)"
Pendulum1.setMass(1);
% "The location (Vec3) of the mass center in the body frame."
Pendulum1.setMassCenter( Vec3(0,0.5,0) )
% Set the inertia
% Modeled as a solid cylinder of radius r=0.1, height h=1 and mass m=1
% Inertia Tensor : Ixx = Iyy = (1/12)*m*(3*(r^2)+h^2) Izz = 1/2*m*(r^2)
Pendulum1.setInertia(Inertia(0.085833333333333,0.085833333333333,0.005,0,0,0)) 
model.addBody(Pendulum1);

% Make and add a Lower Pendulum Body
Pendulum2 = Body();
Pendulum2.setName('LowerPendulum');
% "The mass of the body (kg)"
Pendulum2.setMass(1);
% "The location (Vec3) of the mass center in the body frame."
Pendulum2.setMassCenter( Vec3(0,0.5,0) )
% Set the inertia
% Modeled as a solid cylinder of radius r=0.1, height h=1 and mass m=1
% Inertia Tensor : Ixx = Iyy = (1/12)*m*(3*(r^2)+h^2) Izz = 1/2*m*(r^2)
Pendulum2.setInertia(Inertia(0.085833333333333,0.085833333333333,0.005,0,0,0))
model.addBody(Pendulum2);

%% Create the joints
% Connecting the upper pendulum to the ground
upperPendulumToGround = PinJoint('UpperPendulumToGround',...  % Joint Name
                                  ground,...                  % Parent Frame
                                  Vec3(0,2,0),...             % Translation in Parent Frame
                                  Vec3(0,0,0),...             % Orientation in Parent Frame 
                                  Pendulum1,...               % Child Frame
                                  Vec3(0,1,0),...             % Translation in Child Frame
                                  Vec3(0));                   % Orientation in Child Frame

% Connecting the lower pendulum to the upper pendulum
lowerPendulumToUpperPendulum = PinJoint('LowerPendulumToUpperPendulum',... % Joint Name
                                         Pendulum1,...                     % Parent Frame
                                         Vec3(0,0,0),...                   % Translation in Parent Frame
                                         Vec3(0,1.570796326794897,0),...   % Orientation in Parent Frame 
                                         Pendulum2,...                     % Child Frame
                                         Vec3(0,1,0),...                   % Translation in Child Frame
                                         Vec3(0,0,0));                     % Orientation in Child Frame
                            
model.addJoint(upperPendulumToGround);
model.addJoint(lowerPendulumToUpperPendulum);

%% Create geometries
% Define pendulum dimensions
pendulumHalfLength = 0.5;
pendulumRadius     = 0.1;

% Cylinder(r, halfheight) Convenience constructor that takes radius and half-height.
g = Cylinder(pendulumRadius, pendulumHalfLength);
% Set opacity
g.setOpacity(0.33);
g.setColor(Vec3(1));

% Attach geometry to the bodies
Pendulum1Geom = PhysicalOffsetFrame();
Pendulum1Geom.setName('Pendulum1Geom');
% The location of the Parent Frame is located at (0,1,0).
Pendulum1Geom.setParentFrame(Pendulum1);
Pendulum1Geom.setOffsetTransform(Transform(Vec3(0, pendulumHalfLength, 0))); % From (0,1,0) upwards pendulumHalfLength: Hence location: (0, 1.5, 0)
Pendulum1.addComponent(Pendulum1Geom);
Pendulum1Geom.attachGeometry(g.clone());

Pendulum2Geom = PhysicalOffsetFrame();
Pendulum2Geom.setName('Pendulum2Geom');
% The location of the Parent Frame is located at (0,0,0).
Pendulum2Geom.setParentFrame(Pendulum2);
Pendulum2Geom.setOffsetTransform(Transform(Vec3(0, pendulumHalfLength, 0))); % From (0,0,0) upwards pendulumHalfLength: Hence location: (0, 0.5, 0)
Pendulum2.addComponent(Pendulum2Geom);
Pendulum2Geom.attachGeometry(g.clone());

% We must finalize connections to save the input-output connections in the
% model file.
model.finalizeConnections();

%% Save the .osim model to a file
model.print(modelFilename);
disp('ModelDoublePendulumTwoHingeJoints.osim printed!');