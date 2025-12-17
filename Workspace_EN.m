%% ==========================================================
%  Link, Workspace and Singularity Point Visualization
%  
%  Display Contents:
%  1. Workspace cross-section (X-Z plane at q1=0)
%  2. Singularity positions (red triangles - Elbow singular, magenta squares - Arm-over-Base singular)
%  3. Typical link configurations under singular states
%  4. 3D workspace point cloud with multiple viewpoints
%% ==========================================================
clc; clear; close all;

%% 1. Robot definition
% DH parameters for 4-DOF elderly care robotic arm
d     = [80,   0,   0,   0   ];      % Link offset (mm)
a     = [0, 200, 180, 50 ];          % Link length (mm)
alpha = [-pi/2, 0, 0, -pi/2];        % Link twist angle (rad)

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');
L(4) = Link([0, d(4), a(4), alpha(4)], 'standard');

% Joint limits (elderly care arm constraints)
L(1).qlim = deg2rad([-90  90]);
L(2).qlim = deg2rad([-120 20]);
L(3).qlim = deg2rad([-30  120]);
L(4).qlim = deg2rad([-180 180]);

arm = SerialLink(L, 'name', 'ElderlyCareArm');

% End-effector horizontal constraint: q4 = -(q2 + q3)
% This ensures the end-effector remains horizontal regardless of q2 and q3
keep_horizontal = @(q) [q(1), q(2), q(3), -(q(2) + q(3))];

%% 2. Scan q2-q3 space to compute workspace cross-section (X-Z plane at q1=0)

N2 = 160;              % Number of grid points for q2
N3 = 160;              % Number of grid points for q3

theta2 = linspace(L(2).qlim(1), L(2).qlim(2), N2);
theta3 = linspace(L(3).qlim(1), L(3).qlim(2), N3);
[TH2, TH3] = meshgrid(theta2, theta3);

X = [];   % End-effector X coordinate (mm)
Z = [];   % End-effector Z coordinate (mm)

% Scan all joint angle combinations
for i = 1:numel(TH2)
    q2 = TH2(i);
    q3 = TH3(i);
    % q1 = 0, q4 determined by horizontal constraint
    q  = keep_horizontal([0, q2, q3, 0]);

    % Skip if q4 violates joint limits
    if q(4) < L(4).qlim(1) || q(4) > L(4).qlim(2)
        continue;
    end

    T = arm.fkine(q);
    p = transl(T);       % End-effector position [x y z] (mm)

    X(end+1,1) = p(1);
    Z(end+1,1) = p(3);
end

%% 3. Identify singularity configurations
% This 4-DOF arm has two types of singularities:
% 1. Elbow Singularity: occurs when sin(θ3) = 0
%    Meaning: Links 2 and 3 are collinear (fully extended or folded back)
%    Result: Loss of DOF in extending/retracting along link 2-3 direction
%
% 2. Arm-over-Base (Wrist) Singularity: when end-effector is directly above base on Z-axis
%    Condition: 200*cos(θ2) + 180*cos(θ2+θ3) + 50 = 0 (derived from DH parameters)
%    Meaning: End-effector directly above base, loses rotational control about Z-axis
%    Result: Loss of DOF in rotating about base

N_singular = 100;  % Scanning resolution for singularity detection
theta2_fine = linspace(L(2).qlim(1), L(2).qlim(2), N_singular);
theta3_fine = linspace(L(3).qlim(1), L(3).qlim(2), N_singular);

Xe = []; Ze = [];   % End-effector positions at elbow singularities
Xa = []; Za = [];   % End-effector positions at arm-over-base singularities

% Scan parameter space to find all singularities
for i2 = 1:length(theta2_fine)
    for i3 = 1:length(theta3_fine)
        q2 = theta2_fine(i2);
        q3 = theta3_fine(i3);
        q = keep_horizontal([0, q2, q3, 0]);

        % Check joint angle limits
        if q(4) < L(4).qlim(1) || q(4) > L(4).qlim(2)
            continue;
        end

        % Condition 1: Elbow singularity - sin(θ3) ≈ 0
        % Threshold 0.05 rad ≈ 2.9 degrees
        if abs(sin(q3)) < 0.05
            T = arm.fkine(q);
            p = transl(T);
            Xe(end+1,1) = p(1);
            Ze(end+1,1) = p(3);
        end
        
        % Condition 2: Arm-over-Base singularity
        % Mathematical condition: 200*cos(θ2) + 180*cos(θ2+θ3) + 50 = 0
        % (DH parameters: a2=200mm, a3=180mm, a4=50mm)
        condition_arm_over_base = 200*cos(q2) + 180*cos(q2+q3) + 50;
        
        if abs(condition_arm_over_base) < 10  % Tolerance: 10 mm
            T = arm.fkine(q);
            p = transl(T);
            Xa(end+1,1) = p(1);
            Za(end+1,1) = p(3);
        end
    end
end

% Remove duplicate singularity points (round to 1mm precision)
if ~isempty(Xe)
    Xe_rounded = round(Xe*10)/10;
    Ze_rounded = round(Ze*10)/10;
    [~, idx_unique] = unique([Xe_rounded, Ze_rounded], 'rows', 'stable');
    Xe = Xe(idx_unique);
    Ze = Ze(idx_unique);
end

if ~isempty(Xa)
    Xa_rounded = round(Xa*10)/10;
    Za_rounded = round(Za*10)/10;
    [~, idx_unique] = unique([Xa_rounded, Za_rounded], 'rows', 'stable');
    Xa = Xa(idx_unique);
    Za = Za(idx_unique);
end

%% 4. Plot workspace boundary with singularity points
figure('Name','Workspace with Singular Configurations','NumberTitle','off');
hold on; grid on; box on; axis equal;

% Plot workspace envelope in blue
if ~isempty(X)
    k = boundary(X, Z, 0.8);  % Shrink factor 0.8 produces tighter boundary
    patch(X(k), Z(k), [0.2 0.7 1.0], ...
          'FaceAlpha', 0.6, 'EdgeColor', 'blue', 'LineWidth', 2, ...
          'DisplayName','Reachable Workspace');
end

% Plot elbow singularity points (red triangles)
if ~isempty(Xe)
    scatter(Xe, Ze, 100, 'r', 'filled', '^', ...
        'DisplayName','Elbow Singularity', 'LineWidth', 1.5);
end

% Plot arm-over-base singularity points (magenta squares)
if ~isempty(Xa)
    scatter(Xa, Za, 100, 'm', 'filled', 's', ...
        'DisplayName','Arm-over-Base Singularity', 'LineWidth', 1.5);
end

xlabel('X (mm)', 'FontSize', 12);
ylabel('Z (mm)', 'FontSize', 12);
title('Workspace Cross-section (q_1 = 0) with Singularities', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location','best', 'FontSize', 11);
set(gca, 'FontSize', 11);

%% 5. Plot arm links under typical singular configurations

% 5.1 Typical elbow singularity configuration (θ3 ≈ 0)
if ~isempty(Xe)
    % Select first elbow singularity point
    idx_elbow_config = 1;
    
    % Find joint angles that correspond to this singularity point
    min_dist = inf;
    best_q2 = 0;
    best_q3 = 0;
    
    for i2 = 1:length(theta2_fine)
        for i3 = 1:length(theta3_fine)
            q2_test = theta2_fine(i2);
            q3_test = theta3_fine(i3);
            
            if abs(sin(q3_test)) < 0.05
                T_test = arm.fkine(keep_horizontal([0, q2_test, q3_test, 0]));
                p_test = transl(T_test);
                dist = sqrt((p_test(1) - Xe(idx_elbow_config))^2 + (p_test(3) - Ze(idx_elbow_config))^2);
                
                if dist < min_dist
                    min_dist = dist;
                    best_q2 = q2_test;
                    best_q3 = q3_test;
                end
            end
        end
    end
    
    q_elbow = keep_horizontal([0, best_q2, best_q3, 0]);
    [xj, zj] = jointXZ(arm, q_elbow);
    
    % Draw links with black solid line
    plot(xj, zj, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Elbow Singular Config');
    plot(xj, zj, 'ko', 'MarkerSize', 8, 'MarkerFaceColor','w', 'LineWidth', 1.5);
    
    % Label near end-effector
    text(xj(end)+15, zj(end)+20, 'Elbow Singular', ...
        'Color','r','FontSize', 11,'FontWeight','bold');
end

% 5.2 Typical arm-over-base singularity configuration
if ~isempty(Xa)
    % Select first arm-over-base singularity point
    idx_arm_config = 1;
    
    % Find joint angles that correspond to this singularity point
    min_dist = inf;
    best_q2_a = 0;
    best_q3_a = 0;
    
    for i2 = 1:length(theta2_fine)
        for i3 = 1:length(theta3_fine)
            q2_test = theta2_fine(i2);
            q3_test = theta3_fine(i3);
            condition_test = 200*cos(q2_test) + 180*cos(q2_test+q3_test) + 50;
            
            if abs(condition_test) < 10
                T_test = arm.fkine(keep_horizontal([0, q2_test, q3_test, 0]));
                p_test = transl(T_test);
                dist = sqrt((p_test(1) - Xa(idx_arm_config))^2 + (p_test(3) - Za(idx_arm_config))^2);
                
                if dist < min_dist
                    min_dist = dist;
                    best_q2_a = q2_test;
                    best_q3_a = q3_test;
                end
            end
        end
    end
    
    q_arm = keep_horizontal([0, best_q2_a, best_q3_a, 0]);
    [xj2, zj2] = jointXZ(arm, q_arm);
    
    % Draw links with gray dashed line
    plot(xj2, zj2, 'Color',[0.5 0.5 0.5], ...
         'LineStyle','--', 'LineWidth', 2.5, 'DisplayName', 'Arm-over-Base Singular Config');
    plot(xj2, zj2, 'o', 'Color',[0.5 0.5 0.5], ...
         'MarkerSize', 8, 'MarkerFaceColor','w', 'LineWidth', 1.5);
    
    text(xj2(end)+15, zj2(end)-30, 'Arm-over-Base Singular', ...
        'Color','m','FontSize',11,'FontWeight','bold');
end

legend('Location','best', 'FontSize', 11);
hold off;

%% 6. 3D workspace point cloud visualization

fprintf('Computing 3D workspace... ');

figure('Name','3D Workspace Envelope','NumberTitle','off');
hold on; grid on; box on;

% Scan all joint combinations to generate 3D workspace
N1 = 25;              % Number of q1 grid points
N2_3d = 40;           % Number of q2 grid points
N3_3d = 40;           % Number of q3 grid points

theta1_3d = linspace(L(1).qlim(1), L(1).qlim(2), N1);
theta2_3d = linspace(L(2).qlim(1), L(2).qlim(2), N2_3d);
theta3_3d = linspace(L(3).qlim(1), L(3).qlim(2), N3_3d);

X_all = [];
Y_all = [];
Z_all = [];

% Triple nested loop to scan all joint combinations
for i1 = 1:N1
    for i2 = 1:N2_3d
        for i3 = 1:N3_3d
            q1 = theta1_3d(i1);
            q2 = theta2_3d(i2);
            q3 = theta3_3d(i3);
            q  = keep_horizontal([q1, q2, q3, 0]);

            % Check joint limits
            if q(4) < L(4).qlim(1) || q(4) > L(4).qlim(2)
                continue;
            end

            T = arm.fkine(q);
            p = transl(T);

            X_all(end+1,1) = p(1);
            Y_all(end+1,1) = p(2);
            Z_all(end+1,1) = p(3);
        end
    end
    if mod(i1, 5) == 0
        fprintf('.');
    end
end
fprintf(' Done!\n');

% Draw workspace point cloud colored by Z coordinate
if ~isempty(X_all)
    s = scatter3(X_all, Y_all, Z_all, 8, Z_all, 'filled', 'MarkerFaceAlpha', 0.7);
    colormap('jet');
    cbar = colorbar;
    cbar.Label.String = 'Z (mm)';
    cbar.Label.FontSize = 11;
end

xlabel('X (mm)', 'FontSize', 12);
ylabel('Y (mm)', 'FontSize', 12);
zlabel('Z (mm)', 'FontSize', 12);
title('3D Workspace Envelope (all joint combinations)', 'FontSize', 14, 'FontWeight', 'bold');
view(45, 30);
grid on;
set(gca, 'FontSize', 11);

% Mark singularity points on 3D plot
if ~isempty(Xe)
    scatter3(Xe, zeros(size(Xe)), Ze, 120, 'r', 'filled', '^', ...
        'DisplayName','Elbow Singularity', 'LineWidth', 1.5, 'MarkerEdgeColor', '#8B0000');
end
if ~isempty(Xa)
    scatter3(Xa, zeros(size(Xa)), Za, 120, 'm', 'filled', 's', ...
        'DisplayName','Arm-over-Base Singularity', 'LineWidth', 1.5, 'MarkerEdgeColor', '#8B008B');
end

legend('Location','best', 'FontSize', 11);
hold off;

%% 7. 3D workspace with multiple projection views

figure('Name','3D Workspace - Multiple Views','NumberTitle','off');
set(gcf, 'Position', [100 100 1400 900]);

% View 1: Top projection (X-Y plane)
subplot(2, 2, 1);
scatter(X_all, Y_all, 5, Z_all, 'filled', 'MarkerFaceAlpha', 0.6);
colormap('jet');
xlabel('X (mm)');
ylabel('Y (mm)');
title('Top View (q1 variation)');
axis equal;
grid on;

% View 2: Side projection (X-Z plane)
subplot(2, 2, 2);
scatter(X_all, Z_all, 5, Z_all, 'filled', 'MarkerFaceAlpha', 0.6);
xlabel('X (mm)');
ylabel('Z (mm)');
title('Side View (q2-q3 variation)');
axis equal;
grid on;

% View 3: Front projection (Y-Z plane)
subplot(2, 2, 3);
scatter(Y_all, Z_all, 5, Z_all, 'filled', 'MarkerFaceAlpha', 0.6);
xlabel('Y (mm)');
ylabel('Z (mm)');
title('Front View (q1-q2-q3)');
axis equal;
grid on;

% View 4: 3D isometric
subplot(2, 2, 4);
scatter3(X_all, Y_all, Z_all, 5, Z_all, 'filled', 'MarkerFaceAlpha', 0.6);
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('3D Isometric View');
view(45, 30);
grid on;

%% 8. Workspace cross-sections at multiple q1 values

figure('Name','Multiple q1 Slices','NumberTitle','off');
set(gcf, 'Position', [100 100 1400 400]);

% Select three q1 values: minimum, zero, maximum
q1_values = [L(1).qlim(1), 0, L(1).qlim(2)];
num_slices = length(q1_values);

for slice_idx = 1:num_slices
    subplot(1, num_slices, slice_idx);
    hold on; grid on; box on; axis equal;

    q1_fixed = q1_values(slice_idx);
    
    X_slice = [];
    Z_slice = [];

    % Scan q2-q3 space for fixed q1
    for i2 = 1:length(theta2)
        for i3 = 1:length(theta3)
            q2 = theta2(i2);
            q3 = theta3(i3);
            q  = keep_horizontal([q1_fixed, q2, q3, 0]);

            if q(4) < L(4).qlim(1) || q(4) > L(4).qlim(2)
                continue;
            end

            T = arm.fkine(q);
            p = transl(T);

            X_slice(end+1,1) = p(1);
            Z_slice(end+1,1) = p(3);
        end
    end

    % Plot boundary of this cross-section
    if ~isempty(X_slice)
        k = boundary(X_slice, Z_slice, 0.8);
        patch(X_slice(k), Z_slice(k), [0.2 0.7 1.0], ...
              'FaceAlpha', 0.6, 'EdgeColor', 'blue', 'LineWidth', 1.5);
    end

    xlabel('X (mm)', 'FontSize', 11);
    ylabel('Z (mm)', 'FontSize', 11);
    title(sprintf('q_1 = %.1f degrees', rad2deg(q1_fixed)), 'FontSize', 12, 'FontWeight', 'bold');
    xlim([-600, 600]);
    ylim([0, 800]);
    set(gca, 'FontSize', 10);
    hold off;
end

%% 9. Comprehensive display: Robot at zero config + workspace boundary + singularities

figure('Name','Robot Zero Config + Workspace + Singularities','NumberTitle','off');
hold on; grid on; box on; axis equal;

% Display robot arm at home position
q_zero = [0, 0, 0, 0];
arm.plot(q_zero, 'workspace', [-600 600 -600 600 0 800], 'scale', 0.8);

% Plot workspace boundary
if ~isempty(X)
    k = boundary(X, Z, 0.8);
    patch(X(k), Z(k), [0.3 0.8 0.3], ...
          'FaceAlpha', 0.25, 'EdgeColor', 'green', 'LineWidth', 2.5, ...
          'DisplayName','Workspace Boundary');
end

% Plot singularity points
if ~isempty(Xe)
    scatter(Xe, Ze, 120, 'r', 'filled', '^', ...
        'DisplayName','Elbow Singularity', 'LineWidth', 1.5);
end

if ~isempty(Xa)
    scatter(Xa, Za, 120, 'm', 'filled', 's', ...
        'DisplayName','Arm-over-Base Singularity', 'LineWidth', 1.5);
end

xlabel('X (mm)', 'FontSize', 12);
ylabel('Y (mm)', 'FontSize', 12);
zlabel('Z (mm)', 'FontSize', 12);
title('Robot Zero Config + Workspace Boundary + Singularities', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location','best', 'FontSize', 11);
set(gca, 'FontSize', 11);
view(45, 30);
hold off;


%% =================== Helper Functions ======================

function [xj, zj] = jointXZ(arm, q)
% JOINTXZ  Compute joint positions in X-Z plane given joint angles q
%   
%   Input:
%     arm: SerialLink robot object
%     q:   Joint angle vector (radians)
%   
%   Output:
%     xj: X coordinates of joint positions (n+1 points: base + n joints)
%     zj: Z coordinates of joint positions (n+1 points: base + n joints)

    n = arm.n;
    xj = zeros(n+1, 1);
    zj = zeros(n+1, 1);

    % Base origin
    xj(1) = 0;
    zj(1) = 0;

    % Compute position of each joint using forward kinematics
    for i = 1:n
        % Create temporary robot with first i links only
        L_temp = arm.links(1:i);
        
        % Compute cumulative transformation matrix
        T = eye(4);
        for j = 1:i
            % Extract DH parameters from link j
            d_j = L_temp(j).d;
            a_j = L_temp(j).a;
            alpha_j = L_temp(j).alpha;
            theta_j = L_temp(j).offset + q(j);
            
            % Build standard DH transformation matrix for link j
            c = cos(theta_j);
            s = sin(theta_j);
            ca = cos(alpha_j);
            sa = sin(alpha_j);
            
            T_j = [c, -s*ca, s*sa, a_j*c;
                   s, c*ca, -c*sa, a_j*s;
                   0, sa, ca, d_j;
                   0, 0, 0, 1];
            
            T = T * T_j;
        end
        
        % Extract X and Z coordinates from transformation matrix
        p = transl(T);
        xj(i+1) = p(1);
        zj(i+1) = p(3);
    end
end
