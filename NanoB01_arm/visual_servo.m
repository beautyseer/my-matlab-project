% 基于雅可比矩阵的机械臂控制仿真   其正运动学使用的是标准建模
clear ; close all; clc;

%% 机械臂DH参数
dh_params = [
    0,      0,      0.13156,  pi;       % 关节1
    -pi/2,  0,      0.06639,  -pi/2;    % 关节2
    pi,     0.1104, 0,        0;        % 关节3
    pi,     0.096,  0,        -pi/2;    % 关节4
    -pi/2,  0,      0.07318,  pi/2;     % 关节5
    pi/2,   0,      0.0486,   0];       % 关节6
%% 初始关节角度 (零位)
q = zeros(6, 1);

%% 目标位移 (沿末端X轴移动0.01m)
delta_x = 0.01;  % 位移量 (m)
delta_y = -0.01;
task_space_delta = [delta_x; 0; 0; 0; 0; 0];  % 任务空间位移 [dx, dy, dz, droll, dpitch, dyaw]

%% 参数设置
max_iterations = 100;
tolerance = 1e-6;  % 收敛容差
step_size = 0.1;   % 步长因子

%% 记录初始位置
[T_initial, ~] = forward_kinematics(q,dh_params);
ee_position_initial = T_initial(1:3, 4);
ee_orientation_initial = T_initial(1:3, 1:3);

%% 目标位置 (保持姿态不变，仅沿X轴平移)
ee_position_target = ee_position_initial + ee_orientation_initial * [delta_x; 0; 0];

%% 迭代控制过程
q_history = q;
error_history = zeros(max_iterations, 1);

for i = 1:max_iterations
    % 计算当前位姿
    [T_current, ~] = forward_kinematics(q, dh_params);
    ee_position_current = T_current(1:3, 4);
    
    % 计算位置误差
    position_error = ee_position_target - ee_position_current;
    error_history(i) = norm(position_error);
    
    % 检查收敛性
    if error_history(i) < tolerance
        fprintf('迭代 %d 次后收敛，最终误差: %.6f m\n', i, error_history(i));
        break;
    end
    
    % 计算当前雅可比矩阵
    J = jacobian(q,dh_params);
    
    % 截取位置部分的雅可比矩阵 (3x6)
    J_position = J(1:3, :);
    
    % 使用伪逆计算关节角度增量
    dq = pinv(J_position) * position_error * step_size;
    
    % 更新关节角度
    q = q + dq;
    q_history = [q_history, q];
    
    % 如果达到最大迭代次数
    if i == max_iterations
        fprintf('达到最大迭代次数，最终误差: %.6f m\n', error_history(i));
    end
end

%% 可视化
figure;
hold on;
grid on;

% 绘制初始位形
plot_robot(q_history(:,1), dh_params, 'b', '初始位形');

% 绘制中间位形 (每隔10步绘制一次)
for i = 10:10:size(q_history, 2)-1
    plot_robot(q_history(:,i), dh_params, [0.7, 0.7, 0.7], '');
end

% 绘制最终位形
plot_robot(q_history(:,end), dh_params, 'r', '最终位形');

% 绘制目标点
plot3(ee_position_target(1), ee_position_target(2), ee_position_target(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% 绘制移动轨迹
ee_positions = zeros(3, size(q_history, 2));
for i = 1:size(q_history, 2)
    [T, ~] = forward_kinematics(q_history(:,i), dh_params);
    ee_positions(:,i) = T(1:3, 4);
end
% plot3(ee_positions(1,:), ee_positions(2,:), ee_positions(3,:), 'k--', 'LineWidth', 1);
% 
% % 设置图形属性
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% title(['机械臂沿末端X轴移动' num2str(delta_x*100) 'cm']);
% legend;
% axis equal;
view(3);

%% 绘制误差收敛曲线
figure;
semilogy(1:size(error_history, 1), error_history, 'b-', 'LineWidth', 2);
grid on;
xlabel('迭代次数');
ylabel('位置误差 (m)');
title('误差收敛曲线');
set(gca, 'XLim', [1, max_iterations]);

%% 函数定义 
% 正运动学函数
function [T, joint_positions] = forward_kinematics(q, dh_params)
    n = size(dh_params, 1);
    T = eye(4);
    joint_positions = zeros(n+1, 3);
    
    for i = 1:n
        alpha = dh_params(i, 1);
        a = dh_params(i, 2);
        d = dh_params(i, 3);
        theta_offset = dh_params(i, 4);
        
        theta = q(i) + theta_offset;
        
        % 计算齐次变换矩阵
        T_i = [
            cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0, sin(alpha), cos(alpha), d;
            0, 0, 0, 1
        ];
        
        T = T * T_i;
        joint_positions(i+1, :) = T(1:3, 4)';
    end
end

% 计算雅可比矩阵
function J = jacobian(q, dh_params)
    % 使用数值微分法计算雅可比矩阵
    delta = 1e-6;
    J = zeros(6, length(q));
    
    % 当前位姿
    [T, ~] = forward_kinematics(q, dh_params);
    position = T(1:3, 4);
    orientation = T(1:3, 1:3);
    
    % 对每个关节施加微小扰动
    for i = 1:length(q)
        q_delta = q;
        q_delta(i) = q_delta(i) + delta;
        
        % 计算扰动后的位姿
        [T_delta, ~] = forward_kinematics(q_delta, dh_params);
        position_delta = T_delta(1:3, 4);
        orientation_delta = T_delta(1:3, 1:3);
        
        % 计算位置雅可比列
        J(1:3, i) = (position_delta - position) / delta;
        
        % 计算姿态雅可比列 (使用旋转向量)
        R_error = orientation_delta * orientation';
        omega = rotation_matrix_to_axis_angle(R_error);
        J(4:6, i) = omega / delta;
    end
end

% 旋转矩阵转旋转向量
function omega = rotation_matrix_to_axis_angle(R)
    theta = acos((trace(R) - 1) / 2);
    
    if theta < 1e-10
        omega = [0; 0; 0];
    else
        omega = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)] / (2 * sin(theta)) * theta;
    end
end

% 绘制机械臂
function plot_robot(q, dh_params, color, label)
    [~, joint_positions] = forward_kinematics(q, dh_params);
    
    % 绘制连杆
    for i = 1:size(joint_positions, 1)-1
        plot3([joint_positions(i,1), joint_positions(i+1,1)], ...
              [joint_positions(i,2), joint_positions(i+1,2)], ...
              [joint_positions(i,3), joint_positions(i+1,3)], ...
              'Color', color, 'LineWidth', 2);
    end
    
    % 绘制关节
    plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), ...
          'o', 'Color', color, 'MarkerSize', 6, 'MarkerFaceColor', color);
    
    % 绘制末端坐标系
    [T, ~] = forward_kinematics(q, dh_params);
    position = T(1:3, 4);
    orientation = T(1:3, 1:3);
    
    % X轴
    plot3([position(1), position(1) + 0.03*orientation(1,1)], ...
          [position(2), position(2) + 0.03*orientation(2,1)], ...
          [position(3), position(3) + 0.03*orientation(3,1)], ...
          'r', 'LineWidth', 2);
    
    % Y轴
    plot3([position(1), position(1) + 0.03*orientation(1,2)], ...
          [position(2), position(2) + 0.03*orientation(2,2)], ...
          [position(3), position(3) + 0.03*orientation(3,2)], ...
          'g', 'LineWidth', 2);
    
    % Z轴
    plot3([position(1), position(1) + 0.03*orientation(1,3)], ...
          [position(2), position(2) + 0.03*orientation(2,3)], ...
          [position(3), position(3) + 0.03*orientation(3,3)], ...
          'b', 'LineWidth', 2);
    
    % 添加图例
    if ~isempty(label)
        plot3(position(1), position(2), position(3), 'Color', color, 'LineStyle', 'none', 'Marker', 'o', 'DisplayName', label);
    end
end