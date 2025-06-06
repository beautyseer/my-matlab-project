clear
clc

% ========== 1. 构建机械臂模型 ==========
angle = pi/180;

% 定义 D-H 参数
L(1) = Link([0, 0.13156, 0, 0, 0, pi], 'modified');
L(2) = Link([0, 0.06639, 0, -pi/2, 0, -pi/2], 'modified');
L(3) = Link([0, 0, 0.1104, pi, 0, 0], 'modified');
L(4) = Link([0, 0, 0.096, pi, 0, -pi/2], 'modified');
L(5) = Link([0, 0.07318, 0, -pi/2, 0, pi/2], 'modified');
L(6) = Link([0, 0.0436, 0, pi/2, 0, 0], 'modified');

% 限制范围
L(1).qlim = [-168*angle, 168*angle];
L(2).qlim = [-135*angle, 135*angle];
L(3).qlim = [-150*angle, 150*angle];
L(4).qlim = [-145*angle, 145*angle];
L(5).qlim = [-165*angle, 165*angle];
L(6).qlim = [-180*angle, 180*angle];

% 建立机械臂对象
arm = SerialLink(L, 'name', 'NanoB01');

% ========== 2. 初始状态与参数 ==========
q = [-1.46607657167524 0.883136601509129 0.546288055874225 -0.211184839491315 1.52716309549504 -8.88178419700125e-16];    % 初始关节角度
dt = 0.05;                          % 时间步长
v_tool = [0.1, 0, 0, 0, 0, 0];      % 末端在自身X方向移动，速度0.1m/s
distance_goal = 0.2;               % 移动总距离 (m)

% 获取初始末端位置
T_start = arm.fkine(q);
p_start = transl(T_start);         % 初始末端位置

% 用于存储轨迹
trajectory = p_start;
q_traj = q;

% ========== 3. 主循环 ==========
while true
    % 计算雅可比矩阵（末端基于自身坐标系）
    J_tool = arm.jacobe(q);
    
    % 计算关节速度
    qd = pinv(J_tool) * v_tool';

    % 更新关节角度
    q = q + (qd' * dt);

    % 更新末端位置
    T_now = arm.fkine(q);
    p_now = transl(T_now);
    
    % 存储轨迹
    trajectory = [trajectory; p_now];
    q_traj = [q_traj; q];
    
    % 判断是否达到目标距离
    moved_distance = norm(p_now - p_start);
    if moved_distance >= distance_goal
        break;
    end
end

% ========== 4. 绘制轨迹 ==========
figure;
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r-', 'LineWidth', 2);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('末端沿自身X轴移动轨迹');

% ========== 5. 可选：播放动画 ==========
figure;
for i = 1:size(q_traj, 1)
    arm.plot(q_traj(i,:));
    drawnow;
end
