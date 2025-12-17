clc; clear; close all;

%% DH Parameters (Standard DH Convention)
% Link parameters for a 4-DOF elderly care robotic arm
d     = [80,   0,   0,   0   ];      % Link offset (mm)
a     = [0, 200, 180,  50   ];       % Link length (mm)
alpha = [-pi/2, 0, 0, -pi/2];        % Link twist angle (rad)

% Create robot links using DH parameters
L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');
L(4) = Link([0, d(4), a(4), alpha(4)], 'standard');

% Create serial link robot (4R configuration)
arm = SerialLink(L, 'name', 'ElderlyCareArm');
arm.base = eye(4);

% Home configuration (all joints at 0 degrees)
qz = [0 0 0 0];

%% Plot the robotic arm
figure('Color','white');
arm.plot(qz, ...
    'workspace', [-300 300 -300 300 0 500], ...
    'scale', 0.8, ...
    'jointdiam', 1, ...
    'nobase', 'noname', 'delay', 0, ...
    'noshadow', ...           % Turn off shadow effect
    'notiles');               % Remove floor tiles (key setting)
hold on;

%% Compute homogeneous transformation matrices for each joint frame
T{1} = arm.A(1,qz);
T{2} = T{1} * arm.A(2,qz);
T{3} = T{2} * arm.A(3,qz);
T{4} = T{3} * arm.A(4,qz);

% Collect all transformation matrices (including base frame 0)
T0 = eye(4);
T_all = {T0, T{1}, T{2}, T{3}, T{4}};
frames = {'0','1','2','3','4'};

%% Plot coordinate frames at each joint
% Display frames without X/Y/Z labels, only arrows
for i = 1:5
    trplot(T_all{i}, ...
       'frame', frames{i}, 'text_opts', {'FontSize',12,'FontWeight','bold'}, ...
       'length', 250, 'arrow', 'rgb');  % RGB: Red=X, Green=Y, Blue=Z
end

%% Configure plot appearance
grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

title('Joint Coordinate Frames (Standard DH Convention, q = [0,0,0,0])');
