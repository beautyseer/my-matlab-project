clear; clc; close all;


% 初始化机械臂模型
angle = pi/180;
theta1 = 0;   D1 = 0.13156;  A1 = 0;     alpha1 = 0;     offset1 = pi;
theta2 = 0;   D2 = 0.06639;  A2 = 0;     alpha2 = -pi/2; offset2 = -pi/2;
theta3 = 0;   D3 = 0;        A3 = 0.1104; alpha3 = pi;    offset3 = 0;
theta4 = 0;   D4 = 0;        A4 = 0.096;  alpha4 = pi;    offset4 = -pi/2;
theta5 = 0;   D5 = 0.07318;  A5 = 0;     alpha5 = -pi/2;  offset5 = pi/2;
theta6 = 0;   D6 = 0.0436;   A6 = 0;     alpha6 = pi/2;   offset6 = 0;

L(1) = Link([theta1, D1, A1, alpha1, 0, offset1], 'modified');
L(2) = Link([theta2, D2, A2, alpha2, 0, offset2], 'modified');
L(3) = Link([theta3, D3, A3, alpha3, 0, offset3], 'modified');
L(4) = Link([theta4, D4, A4, alpha4, 0, offset4], 'modified');
L(5) = Link([theta5, D5, A5, alpha5, 0, offset5], 'modified');
L(6) = Link([theta6, D6, A6, alpha6, 0, offset6], 'modified');

arm = SerialLink(L, 'name', 'NanoB01');

% 加载机械臂模型（用你之前定义的 SerialLink，如 arm_NanoB01）

% 初始姿态
q_init = [-pi/2, pi/2, pi/2, pi/2, 0, 0];

% 移动距离（单位：米）
dx = 0.1;  % 沿X移动
dy = -0.01; % 沿Y移动

% 沿X轴移动
[q_result_x ,xyz_positions]= move_along_x(arm, q_init, dx);
figure(1);
plot(1:100, xyz_positions(1,:), 'r', 'LineWidth', 2); hold on;
plot(1:100, xyz_positions(2,:), 'g', 'LineWidth', 2);
plot(1:100, xyz_positions(3,:), 'b', 'LineWidth', 2);
legend('X', 'Y', 'Z');
xlabel('迭代步数');
ylabel('位置 (m)');
title('末端三维位置变化');
grid on;
figure(2)
arm.plot(q_init)
figure(3)
arm.plot(q_result_x)
T1 = arm.fkine(q_init);
q1 = transl(T1);
T2 = arm.fkine(q_result_x);
q2 = transl(T2);
disp(q2-q1);

