clearvars ; close all ; clc ; beep off ;
set(0, 'DefaultFigureUnits', 'pixels') ;
set(0, 'DefaultFigurePosition', [100, 100, 707, 531]) ;
set(findall(0, 'Type', 'figure'), 'Units', 'pixels', 'Position', [200, 200, 707, 531]) ;

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
xlabel('X [m]') ; ylabel('Y [m]') ; zlabel('Z [m]') ; grid on ;
xlim([-0.7, 0.7]) ; ylim([-0.7, 0.7]) ; zlim([0, 1.5]) ;

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
EE_target_DCM = [0, 0, 1; 0, 1, 0; -1, 0, 0] * eye(3) ; % 90 deg rotation around Y from initial DCM
T_target = eye(4) ; % Create homogenous transformation matrix
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
inverse_kinematics_initial_guess =  [home_configuration.JointPosition];
inverse_kinematics_final_solution = [target_configuration.JointPosition];
joint_labels = arrayfun(@(i) sprintf('theta_%d', i), ...
    1:numel([home_configuration.JointPosition]), ...
    'UniformOutput', false);
column_labels = {'Rand Guess  (deg)', 'IK Solution (deg)'};
results = array2table(rad2deg([inverse_kinematics_initial_guess; ...
                               inverse_kinematics_final_solution])', ...
    'VariableNames', column_labels, ...
    'RowNames', joint_labels);
disp(results);


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
xlabel('X [m]') ; ylabel('Y [m]') ; zlabel('Z [m]') ; grid on ;
xlim([-0.7, 0.7]) ; ylim([-0.7, 0.7]) ; zlim([0, 1.2]) ;

%% Workspace analysis

% Generate robot workspace
rng('shuffle') ;
[workspace, configurations] = generateRobotWorkspace(RBT_kuka, {}) ;

% Plot workspace
figure ;
show(RBT_kuka) ;
hold on ;
workspace_area = alphaShape(workspace(:,1), ...
                            workspace(:,2), ...
                            workspace(:,3)) ;
plot(workspace_area, FaceAlpha=0.45, EdgeColor='none') ;
title('Manipulator workspace') ;
xlabel('X [m]') ; ylabel('Y [m]') ; zlabel('Z [m]') ; axis equal ; grid on ;

%% Manipulability index

% Find manipulability index (using Yoshikawa)
manipulability_index = manipulabilityIndex(RBT_kuka, configurations, ...
                                           IndexType="yoshikawa") ;

% Plot workspace with manipulability index
figure ;
show(RBT_kuka) ;
hold on ;
showWorkspaceAnalysis(workspace, manipulability_index) ;
title('Manipulability index in workspace') ;
xlabel('X [m]') ; ylabel('Y [m]') ; zlabel('Z [m]') ; axis equal ; grid on ;

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

        % Compute IK
        T_target_kj(2:3, end) = [y; z] ;
        [manip_configuration, sol_info] = ik_solver(endEffector_body, T_target_kj, weight, target_configuration_kj) ; % Guess is already the target configuration, since it is close to solution
        T = getTransform(RBT_kuka, manip_configuration, endEffector_body) ;
        pos_err = norm(T(1:3, end) - T_target_kj(1:3, end)) ;
        IK_Successful = pos_err < 1e-3 ;

        % Fill manipulability matrix
        if IK_Successful
            reachable(j,k) = true ;
            J = geometricJacobian(RBT_kuka, manip_configuration, endEffector_body) ;
            s = svd(J) ;
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
grid on ;
legend() ;

%% Control EE to desired state (simscape_kuka.slx)

% Define variables for simscape_kuka.slx
initial_configuration = [home_configuration.JointPosition] ;
final_configuration = [target_configuration.JointPosition] ;

% Add Mesh folder to path
if isfolder('Meshes')
    addpath(genpath('Meshes')) ;
end

% Link geometries to simscape simulator - workaround to easily run on any pc
simModel = 'simscape_kuka' ;
load_system(simModel) ;
visBlocks = find_system(simModel, 'MaskType', 'File Solid') ;
link_blocks(visBlocks, simModel) ;

% Control params
K_P = diag([2, 3, 3, 3, 0.5, 0.5, 0.05])  ;
K_D = diag([2, 9, 9, 6,   1,   1, 0.05])  ;
K_I = 0 ;

% Run simulation
simOut = sim(simModel, 'SimulationMode', 'normal') ;

% Plot torque evolution
figure ; hold on ; grid on ;
plot(repmat(simOut.tau_control.time, 1, simOut.tau_control.signals.dimensions), simOut.tau_control.signals.values, LineWidth=2) ;
legend(arrayfun(@(i) ['Joint ', num2str(i)], 1:7, 'UniformOutput', false), 'Location', 'southeast', 'FontSize', 13) ;
yMin = min(simOut.tau_control.signals.values, [], 'all') ;
yMax = max(simOut.tau_control.signals.values, [], 'all') ;
yRange = yMax - yMin ;
ylim([yMin - 0.15*yRange, yMax + 0.15*yRange]) ;
title('Manipulator joint control') ;
xlabel('Time [s]', 'FontSize', 13) ; ylabel('Torque [Nm]', 'FontSize', 13) ;

% Plot error evolution
figure ; hold on ; grid on ;
plot(repmat(simOut.q_err_rad.time, 1, simOut.q_err_rad.signals.dimensions), rad2deg(simOut.q_err_rad.signals.values), LineWidth=2) ;
legend(arrayfun(@(i) ['Joint ', num2str(i)], 1:7, 'UniformOutput', false), 'Location', 'southeast', 'FontSize', 13) ;
yMin = min(rad2deg(simOut.q_err_rad.signals.values), [], 'all') ;
yMax = max(rad2deg(simOut.q_err_rad.signals.values), [], 'all') ;
yRange = yMax - yMin ;
ylim([yMin - 0.15*yRange, yMax + 0.15*yRange]) ;
title('Manipulator joint control') ;
xlabel('Time [s]', 'FontSize', 13) ; ylabel('q - q_{ref} [deg]', 'FontSize', 13) ;

%% Check torques experienced by manipulator, with a target attached
target_mass = 150 ; % [kg]
target_width = 0.6 ; % [m]
target_length = 0.6 ; % [m]
target_height = 0.5 ; % [m]

% Control params unchanged from previous case
simModel = 'simscape_kuka_target' ;
load_system(simModel) ;
visBlocks = find_system(simModel, 'MaskType', 'File Solid') ;
link_blocks(visBlocks, simModel) ;

% Run simulation
simOut = sim(simModel, 'SimulationMode', 'normal') ;

% Plot torque evolution
figure ; hold on ; grid on ;
plot(repmat(simOut.tau_control.time, 1, simOut.tau_control.signals.dimensions), simOut.tau_control.signals.values, LineWidth=2) ;
legend(arrayfun(@(i) ['Joint ', num2str(i)], 1:7, 'UniformOutput', false), 'Location', 'southeast', 'FontSize', 13) ;
yMin = min(simOut.tau_control.signals.values, [], 'all') ;
yMax = max(simOut.tau_control.signals.values, [], 'all') ;
yRange = yMax - yMin ;
ylim([yMin - 0.15*yRange, yMax + 0.15*yRange]) ;
title('Manipulator joint control - w/ target') ;
xlabel('Time [s]', 'FontSize', 13) ; ylabel('Torque [Nm]', 'FontSize', 13) ;

% Plot error evolution
figure ; hold on ; grid on ;
plot(repmat(simOut.q_err_rad.time, 1, simOut.q_err_rad.signals.dimensions), rad2deg(simOut.q_err_rad.signals.values), LineWidth=2) ;
legend(arrayfun(@(i) ['Joint ', num2str(i)], 1:7, 'UniformOutput', false), 'Location', 'southeast', 'FontSize', 13) ;
yMin = min(rad2deg(simOut.q_err_rad.signals.values), [], 'all') ;
yMax = max(rad2deg(simOut.q_err_rad.signals.values), [], 'all') ;
yRange = yMax - yMin ;
ylim([yMin - 0.15*yRange, yMax + 0.15*yRange]) ;
title('Manipulator joint control - w/ target') ;
xlabel('Time [s]', 'FontSize', 13) ; ylabel('q - q_{ref} [deg]', 'FontSize', 13) ;

% Final pose errors with previous controller
final_joint_angles = num2cell(simOut.theta.signals.values(end,:)) ; % Final joint angles
final_target_configuration = target_configuration ; % Initialize
[final_target_configuration.JointPosition] = deal(final_joint_angles{:}) ;

% Erros with respect to desired pose
final_joint_error_angles_deg = rad2deg([final_joint_angles{:}] - final_configuration ) ; % [deg]
% TODO: Compute the same errors, on the end-effector cartesian state
% (position, attitude), through direct kinematics, using final_target_configuration

% Run same analysis with updated control parameters, to estimate maximum required
% torques

%% Save all open figures to PNG (300 DPI) in current folder
figs = findall(0, 'Type', 'figure') ;
if ~isempty(figs)
    [~, idx] = sort([figs.Number]) ;
    figs = figs(idx) ;
    for i = 1:numel(figs)
        fig = figs(i) ;
        drawnow ;
        fname = fullfile(pwd, sprintf('figure_%02d.png', fig.Number)) ;
        set(fig, 'PaperPositionMode', 'auto') ;
        print(fig, fname, '-dpng', '-r300') ;
    end
end

%% Local functions

function config = setConfiguration(configuration_vec, RBT)

config = randomConfiguration(RBT) ; % Initialize structure

for k = 1:length(configuration_vec)
    config(k).JointPosition = configuration_vec(k) ;
end

end

function link_blocks(visBlocks, simModel)
meshFolder = 'Meshes' ;
for i = 1:numel(visBlocks)
    blkPath = visBlocks{i} ;

    % Extract link index from block path (e.g. iiwa_link_3 → 3)
    tokens = regexp(blkPath, 'iiwa_link_(\d+)', 'tokens') ;
    if isempty(tokens)
        fprintf('  Could not parse link index for %s\n', blkPath) ;
        continue ;
    end
    linkIdx = tokens{1}{1} ;

    % Expected STL file name
    stlFile = fullfile(meshFolder, sprintf('link_%s.stl', linkIdx)) ;

    if ~isfile(stlFile)
        fprintf('  Mesh not found for link %s (%s)\n', linkIdx, stlFile) ;
        continue ;
    end

    % Update the File Solid block
    set_param(blkPath, 'ExtGeomFileName', fullfile(meshFolder, ['link_', num2str(linkIdx), '.stl'])) ;

end

save_system(simModel) ;

end
