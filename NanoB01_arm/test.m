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
% 初始化机器人模型（和之前一样，略）

% 初始化姿态和目标
q = [-pi/2, pi/2, pi/2, pi/2, 0, 0];  % 初始关节角度
T0 = arm.fkine(q);                   % 初始位姿
R0 = T0.R;
p0 = T0.t;

% 在末端坐标系中移动 delta（即末端 X 正方向）
delta_local = [0.2; 0; 0];          % 在末端坐标系中移动 1cm

% 设置迭代参数
max_iter = 100;
tolerance = 1e-6;
step_size = 0.5;

q_history = q;
err_history = [];

for i = 1:max_iter
    T = arm.fkine(q);
    p = T.t;
    R = T.R;

    % 计算当前误差（用当前末端姿态转换到末端坐标系）
    p_target = p0 + R0 * delta_local;
    err_world = p_target - p;

    % 把误差从世界坐标系变换到当前末端坐标系下
    err_local = R' * err_world;

    % 获取末端坐标系下的雅可比矩阵
    J = arm.jacobe(q);
    J_pos_local = J(1:3, :);  % 只使用位置部分

    % 计算关节角变化量
    dq = pinv(J_pos_local) * err_local * step_size;
    q = q + dq';

    q_history = [q_history; q];
    err_history(end+1) = norm(err_world);  % 用 world 坐标下误差评估是否收敛

    if err_history(end) < tolerance
        fprintf('收敛于第 %d 次迭代，误差 %.6f m\n', i, err_history(end));
        break;
    end
end

% 动画显示
figure;
arm.plot(q_history(1,:), 'workspace', [-0.5 0.5 -0.5 0.5 0 0.5]);
hold on;
for i = 2:10:size(q_history,1)
    arm.plot(q_history(i,:),'delay',0.8);
end
arm.plot(q_history(end,:));



% 绘制目标点
plot3(p_target(1), p_target(2), p_target(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
title('基于末端坐标系的雅可比控制（沿自身X轴移动）');


% 绘制误差收敛曲线
figure;
semilogy(err_history, 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('位置误差 (m)');
title('误差收敛曲线');

