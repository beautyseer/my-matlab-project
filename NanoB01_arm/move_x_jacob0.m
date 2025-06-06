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

% 初始关节角度（注意是去 offset 后的值）
q = [-pi/2, pi/2, pi/2, pi/2, 0, 0]; % rad

% 设置位移目标
delta = 0.1; % m，末端自身 X 方向平移 1cm
T0 = arm.fkine(q);
R0 = T0.R;
p0 = T0.t;

% 设置目标位置
p_target = p0 + R0 * [delta; 0; 0];

% 迭代参数
max_iter = 100;
tolerance = 1e-5;
step_size = 0.1;
q_history = q;
err_history = [];

for i = 1:max_iter
    T = arm.fkine(q);
    R = T.R;
    p = T.t;
    
    pos_err = p_target - p;
    err_norm = norm(pos_err);
    err_history(end+1) = err_norm;
    
    if err_norm < tolerance
        fprintf('收敛于第 %d 次迭代，误差 %.6f m\n', i, err_norm);
        break;
    end

    J = arm.jacob0(q);     % 雅可比矩阵（基坐标系下）
    J_pos = J(1:3, :);     % 只取位置部分
    dq = pinv(J_pos) * pos_err * step_size;
    q = q + dq';           % 注意 dq 是列向量，q 是行向量
    q_history = [q_history; q]; 
end

% 画图
figure;
arm.plot(q_history(1,:), 'workspace', [-0.5 0.5 -0.5 0.5 0 0.5]);
hold on;
for i = 2:10:size(q_history,1)
    arm.plot(q_history(i,:));
end
arm.plot(q_history(end,:));

% 绘制目标点
plot3(p_target(1), p_target(2), p_target(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
title('机械臂末端沿其自身X轴移动0.01m');

% 误差曲线
figure;
semilogy(err_history, 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('位置误差 (m)');
title('误差收敛曲线');
