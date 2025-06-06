clear
clc

% 角度转换
angle = pi/180;  % 转化为弧度制

% D-H参数表
theta1 = 0;   D1 = 0.13156;  A1 = 0;     alpha1 = 0;     offset1 = pi;
theta2 = 0;   D2 = 0.06639;  A2 = 0;     alpha2 = -pi/2; offset2 = -pi/2;
theta3 = 0;   D3 = 0;       A3 = 0.1104; alpha3 = pi;    offset3 = 0;
theta4 = 0;   D4 = 0;       A4 = 0.096; alpha4 = pi;    offset4 = -pi/2;
theta5 = 0;   D5 = 0.07318;  A5 = 0;     alpha5 = -pi/2; offset5 = pi/2;
theta6 = 0;   D6 = 0.0436;   A6 = 0;     alpha6 = pi/2;  offset6 = 0;

% DH法建立模型
L(1) = Link([theta1, D1, A1, alpha1, 0, offset1], 'modified');
L(2) = Link([theta2, D2, A2, alpha2, 0, offset2], 'modified');
L(3) = Link([theta3, D3, A3, alpha3, 0, offset3], 'modified');
L(4) = Link([theta4, D4, A4, alpha4, 0, offset4], 'modified');
L(5) = Link([theta5, D5, A5, alpha5, 0, offset5], 'modified');
L(6) = Link([theta6, D6, A6, alpha6, 0, offset6], 'modified');

% 定义关节范围
L(1).qlim = [-168*angle, 168*angle];
L(2).qlim = [-135*angle, 135*angle];
L(3).qlim = [-150*angle, 150*angle];
L(4).qlim = [-145*angle, 145*angle];
L(5).qlim = [-165*angle, 165*angle];
L(6).qlim = [-180*angle, 180*angle];

% 创建机械臂模型
arm_NanoB01 = SerialLink(L, 'name', 'NanoBO1');

%% 设置末端速度并计算关节速度
% 1. 设置初始关节角度（弧度）
q_initial = [0, pi/4, pi/2, 0, pi/4, 0];

% 2. 设置末端坐标系下的速度（目标：沿X轴正方向移动）
% [vx, vy, vz, wx, wy, wz] - 线速度(m/s)和角速度(rad/s)
v_tool = [0.1, 0, 0, 0, 0, 0];  % 仅沿末端X轴移动，速度0.1m/s

% 3. 计算相对于末端坐标系的雅可比矩阵
J_tool = arm_NanoB01.jacobe(q_initial);
J_tool2 = arm_NanoBO1_jacobe(q_initial);

% 4. 计算关节速度（使用伪逆矩阵求解）
qd = pinv(J_tool) * v_tool';  % 注意：必须使用矩阵乘法(*)而非点乘(.*)
qd2 = pinv(J_tool2) * v_tool';
% 5. 显示结果
fprintf('目标末端速度 (m/s, rad/s):\n');
fprintf('线速度: [%.4f, %.4f, %.4f]\n', v_tool(1:3));
fprintf('角速度: [%.4f, %.4f, %.4f]\n\n', v_tool(4:6));

fprintf('计算得到的关节速度 (rad/s):\n');
fprintf('q1: %.4f, q2: %.4f, q3: %.4f\n', qd(1), qd(2), qd(3));
fprintf('q4: %.4f, q5: %.4f, q6: %.4f\n\n', qd(4), qd(5), qd(6));

fprintf('公式计算得到的关节速度 (rad/s):\n');
fprintf('q1: %.4f, q2: %.4f, q3: %.4f\n', qd2(1), qd2(2), qd2(3));
fprintf('q4: %.4f, q5: %.4f, q6: %.4f\n\n', qd2(4), qd2(5), qd2(6));

v_t = J_tool*qd

%% 绘制轨迹

%% 轨迹模拟参数
dt = 0.05;              % 时间步长（秒）
T_total = 0.2;            % 模拟总时间（秒）
steps = T_total / dt;   % 步数

q = q_initial;          % 初始关节角度
trajectory = zeros(steps, 3);  % 用于记录末端位置轨迹

for i = 1:steps
    % 计算末端位姿
    T = arm_NanoB01.fkine(q);       % 末端变换矩阵
    pos = transl(T);                % 提取位置向量
    trajectory(i, :) = pos';        % 存储轨迹点

    % 更新关节角度：q(t+1) = q(t) + qd * dt
    q = q + (qd' * dt);             % 注意转置 qd
    arm_NanoB01.plot(q);
    drawnow;
end

%% 绘制轨迹
figure;
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'b-', 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('机械臂末端轨迹');
axis equal;

% 可选：画出机械臂当前姿态
figure;
arm_NanoB01.plot(q_initial);  % 初始姿态

T1 = arm_NanoB01.fkine(q_initial);
T2 = arm_NanoB01.fkine(q);
