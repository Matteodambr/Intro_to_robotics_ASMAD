clearvars ; close all ; clc ; beep off ;

% Procedure
% 1. Load model
% 2. Choose desired EE state, and solve inverse kinematics
% 3. Control the EE to desired state

% Load the robot from matlab pre-loaded models
RBT_kuka = loadrobot('kukaiiwa7') ;

% (alternatively) Load the robot from urdf file
% RBT_kuka = importrobot('RBT_kuka.urdf') ;

% How to export a rigid body tree to urdf format
% exporter = urdfExporter(RBT_kuka) ;
% writefile(exporter) ;

% Import any urdf robot to Simscape Multibody (you can do it directly with any URDF file, without the RBT)
% smimport('RBT_kuka.urdf') ;

% Show robot in plot, in the "home" configuration
show(RBT_kuka) ; hold on, title('Home manipulator configuration') ;

% Find end-effector of robot
endEffector_body = RBT_kuka.BodyNames{end} ;

%% Solve DIRECT KINEMATICS in home configuration -> Find home EE pose
home_configuration = homeConfiguration(RBT_kuka) ;
T_home = getTransform(RBT_kuka, home_configuration, endEffector_body) ; % Find homogenous transformation matrix of end-effector, in home configuration
EE_home_POS = tform2trvec(T_home)' ; % or EE_home_POS = T_home(1:3, end) ;
EE_home_DCM = tform2rotm(T_home) ; % or EE_home_DCM = T_home(1:3, 1:3) ;

%% Choose EE target pose, and solve inverse kinematics

% Target EE pose
EE_target_POS = [0.4; 0.2; 0.7] ; % Goal position for the end-effector
EE_target_DCM = [0, 0, 1; 0, 1, 0; -1, 0, 0] * eye(3) ; % eye(3) is the home configuration corresponding to joint angles (0,...,0), while [0, 0, 1; 0, 1, 0; -1, 0, 0] is a matrix representing a 90 deg rotation around Y
T_target = eye(4) ;
T_target(1:3,1:3) = EE_target_DCM ;
T_target(1:3, end) = EE_target_POS ;

% Set up inverse kinematics solver (optimization-based)
solver_params.StepTolerance = 1e-8 ;
ik_solver = inverseKinematics('RigidBodyTree', RBT_kuka, 'SolverAlgorithm', 'LevenbergMarquardt') ;
weight = ones(6, 1) ; % xyz_rotation, xyz_translation

% Compute inverse kinematics
[target_configuration, sol_info] = ik_solver(endEffector_body, T_target, weight, home_configuration) ;
assert(sol_info.ExitFlag == 1, 'Solver failed to converge. Target pose is likely outside of the manipulator''s workspace.') ;

% Display initial guess and inverse kinematics solution
inverse_kinematics_initial_guess =  [home_configuration.JointPosition] ;
inverse_kinematics_final_solution = [target_configuration.JointPosition] ;
joint_labels = arrayfun(@(i) sprintf('theta_%d', i), 1:numel([home_configuration.JointPosition]), 'UniformOutput', false) ;
row_labels = {'Rand Guess  (deg)'; 'IK Solution (deg)'};
results = array2table([rad2deg(inverse_kinematics_initial_guess); rad2deg(inverse_kinematics_final_solution)], 'VariableNames', joint_labels, 'RowNames', row_labels) ;
disp(results) ;

% Check that configuration found via IK corresponds to desired EE target,
% using direct kinematics
T_target_check = getTransform(RBT_kuka, target_configuration, endEffector_body) ;
tol = 1e-6 ;
assert(all(abs(T_target - T_target_check) < tol, 'all'), 'Computed joint angles do not position end-effector at the specified target.') ;

% Show target manipulator configuration
figure ;
show(RBT_kuka, setConfiguration(inverse_kinematics_final_solution, RBT_kuka)) ;
hold on ;
title('Target manipulator configuration') ;

%% Workspace analysis

% Generate robot workspace
[workspace, configurations] = generateRobotWorkspace(RBT_kuka, {}) ;

% Find manipulability index (using Yoshikawa)
manipulability_index = manipulabilityIndex(RBT_kuka, configurations, ...
                                           IndexType="yoshikawa") ;

% Plot workspace with manipulability index
figure ;
show(RBT_kuka) ;
hold on ;
showWorkspaceAnalysis(workspace, manipulability_index) ;

%% How well does the robot manipulate around a specific position? (i.e. your target)
samples_per_axis = 25 ;
grid_size = 0.3 ; % [m]
y_vec = linspace(EE_target_POS(2)-grid_size, EE_target_POS(2)+grid_size, samples_per_axis) ;
z_vec = linspace(EE_target_POS(3)-grid_size, EE_target_POS(3)+grid_size, samples_per_axis+1) ;

% Compute manipuability at each point
manipulability = zeros(samples_per_axis, samples_per_axis+1) ;
T_target_kj = T_target ;
target_configuration_kj = target_configuration ;
reachable = false(samples_per_axis, samples_per_axis+1) ;
for k = 1:samples_per_axis+1
    z = z_vec(k) ;
    for j = 1:samples_per_axis
        y = y_vec(j) ;
        T_target_kj(2:3, end) = [y; z] ;
        [manip_configuration, sol_info] = ik_solver(endEffector_body, T_target_kj, weight, target_configuration_kj) ; % Guess is already the target configuration, since it is close to solution
        T = getTransform(RBT_kuka, manip_configuration, endEffector_body) ;
        pos_err = norm(T(1:3, end) - T_target_kj(1:3, end)) ;
        IK_Successful = pos_err < 1e-3 ;

        % Fill manipulability matrix
        if IK_Successful
            reachable(j,k) = true ;
            J = geometricJacobian(RBT_kuka, manip_configuration, endEffector_body) ;
            s = svd(J(1:3,:)) ;
            manipulability(j,k) = prod(s) ;
            target_configuration_kj = manip_configuration ;
        else
            manipulability(j,k) = NaN ;
        end
    end
end

% Plot manipulability map (ZY slice)
figure ;
[Y_VEC, Z_VEC] = meshgrid(y_vec, z_vec) ; 
scatter(Y_VEC(reachable'), Z_VEC(reachable'), 40, manipulability(reachable), 'filled', 'DisplayName', 'Manipulability index') ;
hold on ;
scatter(EE_target_POS(2), EE_target_POS(3), 80, 'red', 'filled', 'o', 'DisplayName', 'Target')
axis equal ; xlabel('Y [m]') ; ylabel('Z [m]') ;
title('Manipulability around target position (ZY plane)') ;
colorbar ; colormap('parula') ;
legend() ;

%% Control EE to desired state (simscape_kuka.slx)


%% Local functions

function config = setConfiguration(configuration_vec, RBT)

config = randomConfiguration(RBT) ; % Initialize structure

for k = 1:length(configuration_vec)
    config(k).JointPosition = configuration_vec(k) ;
end

end