clearvars ; close all ; clc ; beep off ;

% Procedure
% 1. Load model
% 2. Choose desired EE state, and solve inverse kinematics
% 3. Control the EE to desired state

%% Load the robot
[RBT_kuka, robotData] = loadrobot('kukaIiwa7') ; % 7-DoF example

% Generate URDF and export it in simscape (this is already done, generating RBT_kuka.urdf and the manipulator model in simscape_kuka)
% exporter = urdfExporter(RBT_kuka) ;
% writefile(exporter) ;
% smimport('RBT_kuka.urdf') ;

%% Desired EE state and inverse kinematics (simulink_inverse_kinematics.slx)

% Desired EE state (position + attitude)
EE_DCM_desired = [0, 0, 1; 0, 1, 0; -1, 0, 0] * eye(3) ; % 90 degree rotation around y
EE_pos_desired = [0.4; 0; 0.4] ;

% Generate the configuration structure - setConfiguration()
guess_configuration = randomConfiguration(RBT_kuka) ;
[guess_configuration.JointPosition]

% Compute inverse kinematics through simulink
model = 'simulink_inverse_kinematics' ;
open_system(model) ;
set_param(model, 'StopTime', '1') ; % Any simulation time is fine
simOut = sim(model) ;
ExitFlag_IK = simOut.inverse_kin_info.ExitFlag.Data(end) ; % Exit flags: https://it.mathworks.com/help/robotics/ug/inverse-kinematics-algorithms.html#bve7b43-2

% Desired configuration that robot needs to be controlled to (solution of inverse kinematics)
q_desired = simOut.inverse_kin_config.signals.values(:,:,end) ;

%% Control EE to desired state (simscape_kuka.slx)

% Randomize initial configuration in Simscape
rng('shuffle') ;
configuration = rand(7,1) ;

% Show robot plot
show(RBT_kuka, setConfiguration(q_desired, RBT_kuka)) ;
hold on ;
title('Desired manipulator configuration') ;





%% Local functions

function config = setConfiguration(configuration_vec, RBT)

config = randomConfiguration(RBT) ; % Initialize structure

for k = 1:length(configuration_vec)
    config(k).JointPosition = configuration_vec(k) ;
end

end