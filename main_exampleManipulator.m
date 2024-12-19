clearvars ; close all ; clc ; beep off ;

% Robotics system toolbox (possibly)

% Variables
AbsTol = 1e-6 ; % Tolerance to compute inverse kinematics (position and attitude of EE)

% Load the robot
[RBT_kuka, robotData] = loadrobot('kukaIiwa7') ;

% Generate URDF and export it in simscape
% exporter = urdfExporter(RBT_kuka) ;
% writefile(exporter) ;
% smimport('RBT_kuka.urdf') ;

% Desired EE state (position + attitude)
EE_DCM_desired = roty(90)*eye(3) ;
EE_pos_desired = [0.4; 0; 0.4] ;

% Generate the configuration structure - setConfiguration()
guess_configuration = randomConfiguration(RBT_kuka) ;
[guess_configuration.JointPosition]

% Compute inverse kinematics through simulink
model = 'simulink_manipulator_example' ;
open_system(model) ;
set_param(model, 'StopTime', '1') ;
simOut = sim(model) ;

%
ExitFlag = simOut.inverse_kin_info.ExitFlag.Data(end) ;

% Desired configuration that robot needs to be controlled to 
q_desired = simOut.inverse_kin_config.signals.values(:,:,end) ;

% Initial configuration
rng('shuffle') ;
configuration = rand(7,1) ;

%
show(RBT_kuka, setConfiguration(q_desired, RBT_kuka)) ;


function config = setConfiguration(configuration_vec, RBT)

config = randomConfiguration(RBT) ;

for k = 1:length(configuration_vec)
    config(k).JointPosition = configuration_vec(k) ;
end

end