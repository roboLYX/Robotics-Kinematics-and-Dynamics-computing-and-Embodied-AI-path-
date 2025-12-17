%% ElderlyCareArm: Analytical IK -> Joint Trajectory -> FK to Generate End-Effector Path (Position-only + Horizontal End-Effector)
% 
% Workflow:
% 1) Define 4 target end-effector positions P1~P4 (position only, orientation not constrained)
% 2) Solve analytical IK for each position to get joint angles q_wp(i,1:3)
% 3) Apply keep_horizontal constraint to compute q_wp_full(i,1:4)
% 4) Use FK to verify position errors at keypoints (theoretically ~1e-12 mm)
% 5) Use jtraj for joint-space interpolation to generate q_traj_full
% 6) Use FK to generate continuous end-effector trajectory p_fk and plot 3D path + joint velocities
%
% Key Constraint:
% - End-effector remains horizontal: q4 = -(q2 + q3)
% - Analytical IK only constrains position (x,y,z), orientation implicitly determined by constraint

clc; clear; close all;

%% 1) Define the 4R robotic arm (Standard DH Convention)

% DH parameters for elderly care arm
d     = [80,   0,   0,   0   ];      % Link offset (mm)
a     = [0, 200, 180, 50 ];          % Link length (mm)
alpha = [-pi/2, 0, 0, -pi/2];        % Link twist angle (rad)

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');
L(4) = Link([0, d(4), a(4), alpha(4)], 'standard');

% Joint limits (elderly care scenario constraints)
L(1).qlim = deg2rad([-90  90]);       % Base rotation: ±90°
L(2).qlim = deg2rad([-120 20]);       % Shoulder joint: -120° to 20°
L(3).qlim = deg2rad([-30  120]);      % Elbow joint: -30° to 120°
L(4).qlim = deg2rad([-180 180]);      % Wrist rotation: ±180°

arm = SerialLink(L, 'name', 'ElderlyCareArm');

% End-effector horizontal constraint: q4 = -(q2 + q3)
% This ensures the end-effector remains horizontal regardless of q2 and q3
keep_horizontal = @(q) [q(1), q(2), q(3), -(q(2) + q(3))];

% Optional: visualize robot at initial configuration
% figure; arm.plot(keep_horizontal([0 0 0 0]), 'nowrist', 'noname'); grid on; axis equal;

%% 2) Define 4 target end-effector positions (mm)
% Note: With increased arm length (200+180+50), target positions should be within reachable workspace
P1 = [ 250,   50,  300];   % Example: Pick object (bedside/table surface height)
P2 = [ 250,   50,  200];   % Vertical lift
P3 = [ 100,  300,  200];   % Horizontal movement
P4 = [ 100,  300,  300];   % Place object

P_list = [P1; P2; P3; P4];

%% 3) Solve analytical IK: compute q1-q3 for each point, then supplement q4

q_wp_3   = zeros(4,3);  % Store (q1, q2, q3) for 4 waypoints
q_wp_4   = zeros(4,4);  % Store (q1, q2, q3, q4) for 4 waypoints

% Reference initial posture for continuous branch selection
% (adjust based on typical care-task posture)
q_guess_3 = deg2rad([0, -30, 60]); 

for i = 1:4
    if i == 1
        q_ref = q_guess_3;
    else
        % Use previous solution as reference for smooth continuity
        q_ref = q_wp_3(i-1,:);
    end

    % Solve analytical IK for position only
    q_wp_3(i,:) = elderlyArm_IK_analytic_pos(P_list(i,:), q_ref, arm);
    
    % Apply horizontal constraint to compute q4
    q_wp_4(i,:) = keep_horizontal(q_wp_3(i,:));
end

disp('=== Joint angles (radians) for 4 waypoints: q1..q4 ===');
disp(q_wp_4);
disp('=== Joint angles (degrees) for 4 waypoints: q1..q4 ===');
disp(rad2deg(q_wp_4));

%% 3.5) Verify 4 waypoints using FK (position accuracy check)
T_wp = arm.fkine(q_wp_4);
p_wp = transl(T_wp);

disp('=== TCP positions from FK (mm) for 4 waypoints ===');
disp(p_wp);

compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

%% 3.6) Error analysis: desired vs. FK positions
err_abs = p_wp - P_list;
err_norm = sqrt(sum(err_abs.^2, 2));
P_norm   = sqrt(sum(P_list.^2, 2));
rel_err  = err_norm ./ max(P_norm, 1e-6);

err_tbl = table( ...
    P_list(:,1), P_list(:,2), P_list(:,3), ...
    p_wp(:,1),   p_wp(:,2),   p_wp(:,3), ...
    err_abs(:,1), err_abs(:,2), err_abs(:,3), ...
    err_norm, rel_err, ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk','Ex','Ey','Ez','ErrNorm','RelErrNorm'});
disp('=== Position errors at each waypoint (mm) ===');
disp(err_tbl);

[maxErr, idxMax] = max(err_norm);
fprintf('Maximum Euclidean error = %.3e mm at waypoint %d.\n', maxErr, idxMax);

%% 4) Joint-space interpolation: P1->P2->P3->P4 using 3 jtraj segments

N = 20; % Number of interpolation points per segment (adjustable)

q_traj_4 = [];
for i = 1:3
    % Generate trajectory segment from waypoint i to i+1
    [q_seg, ~, ~] = jtraj(q_wp_4(i,:), q_wp_4(i+1,:), N);
    if i == 1
        q_traj_4 = q_seg;
    else
        % Concatenate segments, removing duplicate waypoint
        q_traj_4 = [q_traj_4; q_seg(2:end,:)]; %#ok<AGROW>
    end
end

%% 5) Use FK to generate continuous end-effector trajectory and plot

T_fk = arm.fkine(q_traj_4);
p_fk = transl(T_fk);

figure('Name', 'TCP Trajectory');
plot3(p_fk(:,1), p_fk(:,2), p_fk(:,3), 'b-', 'LineWidth', 2); hold on;

% Mark the 4 waypoints
plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'P1 (Pick)');
plot3(P2(1), P2(2), P2(3), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'P2 (Lift)');
plot3(P3(1), P3(2), P3(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'P3 (Move)');
plot3(P4(1), P4(2), P4(3), 'mo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'P4 (Place)');

grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('TCP Trajectory (Analytical IK + Keep-Horizontal + jtraj + FK)');
legend('TCP path','P1','P2','P3','P4', 'Location', 'best');
hold off;

%% 6) Plot joint angular velocities

dt = 1; % Time step for velocity computation (determines velocity curve shape)
qd_traj = diff(q_traj_4) / dt;
t_qd = (1:size(qd_traj,1)) * dt;

figure('Name', 'Joint Velocities');
for j = 1:4
    subplot(4,1,j);
    plot(t_qd, qd_traj(:,j), 'LineWidth', 1.5);
    grid on;
    ylabel(sprintf('\\omega_%d [rad/s]', j));
    title(sprintf('Joint %d Angular Velocity', j));
    if j == 4
        xlabel('Time [s]');
    end
end
sgtitle('Joint Velocity Profiles');

%% 7) Animation demonstration

figure('Name', 'Robot Animation');
arm.plot(q_traj_4, 'trail', {'r', 'LineWidth', 1.5}, 'nowrist', 'noname');
title('ElderlyCareArm: 4-Point Joint-Space Trajectory (Analytical IK)');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local Functions: Analytical IK for Elderly Care Arm (Position-only)
%% + Horizontal End-Effector (q4 supplied externally via keep_horizontal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q_best = elderlyArm_IK_analytic_pos(p, q_ref, arm)
% elderlyArm_IK_analytic_pos
%   
%   Solves analytical inverse kinematics for position only.
%   Orientation constraint (horizontal end-effector) is applied externally.
%
% Inputs:
%   p     : 1x3, target TCP position [x y z] (mm)
%   q_ref : 1x3, reference joint angles [q1 q2 q3] (rad) for branch selection
%   arm   : SerialLink object (used to read a2, a3, a4, d1, joint limits)
%
% Output:
%   q_best: 1x3, [q1 q2 q3] (rad)
%
% Geometric assumptions (matching horizontal end-effector constraint):
% - a4 (tool length) extends along horizontal radial direction: a4 * [cos(q1), sin(q1), 0]
% - Therefore, IK first subtracts a4, then solves planar 2R analytical solution for (a2, a3)
%
% This approach is commonly used in care tasks: end-effector orientation is maintained
% horizontal via q4 = -(q2 + q3), while position is freely controlled by q1, q2, q3.

    x = p(1); y = p(2); z = p(3);

    % Extract DH parameters from robot
    d1 = arm.links(1).d;   % Base height: 80 mm
    a2 = arm.links(2).a;   % Link 2 length: 200 mm
    a3 = arm.links(3).a;   % Link 3 length: 180 mm
    a4 = arm.links(4).a;   % Tool length: 50 mm

    % 1) q1: Base yaw (directly from TCP x-y projection)
    q1 = atan2(y, x);

    % 2) Project TCP to work plane and subtract horizontal tool extension (a4)
    r_tcp = hypot(x, y);
    r = r_tcp - a4;        % Subtract horizontal tool extension
    z2 = z - d1;           % Subtract base height

    % Reachability check (2R planar case)
    d = hypot(r, z2);
    if d > (a2 + a3) + 1e-6 || d < abs(a2 - a3) - 1e-6
        error('Target (%.1f, %.1f, %.1f) is outside workspace for this IK model.', x, y, z);
    end

    % 3) q3: Two solutions from law of cosines (elbow-up and elbow-down)
    c3 = (r^2 + z2^2 - a2^2 - a3^2) / (2*a2*a3);
    c3 = max(min(c3, 1), -1);  % Clamp to [-1, 1] to avoid numerical errors
    s3_pos =  sqrt(1 - c3^2);
    s3_neg = -sqrt(1 - c3^2);

    q3_a = atan2(s3_pos, c3);  % Elbow-up solution
    q3_b = atan2(s3_neg, c3);  % Elbow-down solution

    % 4) q2: Derived from geometric relationships
    q2_a = atan2(z2, r) - atan2(a3*sin(q3_a), a2 + a3*cos(q3_a));
    q2_b = atan2(z2, r) - atan2(a3*sin(q3_b), a2 + a3*cos(q3_b));

    % Collect candidate solutions
    cand = [q1, q2_a, q3_a;
            q1, q2_b, q3_b];

    % 5) Filter by joint limits and select closest to reference posture
    valid = true(2,1);
    for k = 1:2
        qk = cand(k,:);

        % Check q1 limits
        if ~isempty(arm.links(1).qlim)
            if qk(1) < arm.links(1).qlim(1)-1e-6 || qk(1) > arm.links(1).qlim(2)+1e-6
                valid(k) = false;
            end
        end
        % Check q2 limits
        if ~isempty(arm.links(2).qlim)
            if qk(2) < arm.links(2).qlim(1)-1e-6 || qk(2) > arm.links(2).qlim(2)+1e-6
                valid(k) = false;
            end
        end
        % Check q3 limits
        if ~isempty(arm.links(3).qlim)
            if qk(3) < arm.links(3).qlim(1)-1e-6 || qk(3) > arm.links(3).qlim(2)+1e-6
                valid(k) = false;
            end
        end
    end

    idx = find(valid);
    if isempty(idx)
        warning('IK: both branches violate joint limits; selecting closest to q_ref anyway.');
        idx = (1:2)';
    end

    if isempty(q_ref) || any(isnan(q_ref))
        q_best = cand(idx(1),:);
        return;
    end

    % Select the branch closest to reference posture
    bestIdx = idx(1);
    bestDist = norm( angdiff_vec(q_ref, cand(bestIdx,:)) );

    for ii = 2:numel(idx)
        k = idx(ii);
        dtmp = norm( angdiff_vec(q_ref, cand(k,:)) );
        if dtmp < bestDist
            bestDist = dtmp;
            bestIdx = k;
        end
    end

    q_best = cand(bestIdx,:);
end

function d = angdiff_vec(q1, q2)
% Compute element-wise angular difference, wrapped to [-pi, pi]
    d = atan2(sin(q2 - q1), cos(q2 - q1));
end
