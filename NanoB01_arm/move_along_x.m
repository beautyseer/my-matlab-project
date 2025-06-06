function [q_final,xyz_positions] = move_along_x(arm, q_init, dx)
    % 初始化参数
    q = q_init(:);   % 保证 q 是列向量
    max_iterations = 100;
    tolerance = 1e-6;
    step_size = 0.1;
    xyz_positions = zeros(3, max_iterations);
    
    % 记录初始末端位姿及末端坐标系
    T0 = arm.fkine(q);
    p0 = T0.t;         % 初始末端位置（世界坐标）
    R0 = T0.R;         % 初始末端姿态（旋转矩阵）
    
    % 计算目标位置：在初始末端坐标系中沿 X 轴移动 dx
    p_target = p0 + R0(:,1) * dx;
    
    % 迭代控制更新
    for i = 1:max_iterations
        T_current = arm.fkine(q);
        p_current = T_current.t;
        R_current = T_current.R;
        xyz_positions(:, i) = T_current.t;
        % 位姿误差 in world坐标
        err_world = p_target - p_current;
        
        % 收敛判断（用世界坐标下的误差范数）
        if norm(err_world) < tolerance
            fprintf('move_along_x 在第 %d 次迭代收敛，误差 %.6f m\n', i, norm(err_world));
            break;
        end
        
        % 将世界坐标下的误差转换到当前末端坐标系中
        % （这里使用当前的姿态 R_current）
        err_local = R_current' * err_world;
        
        % 获取末端坐标系下的雅可比
        % 末端坐标系的雅可比直接给出速度在当前末端坐标系下的表达
        J_local = arm.jacobe(q);
        J_pos_local = J_local(1:3, :);  % 仅取位置部分
        
        % 计算关节角度增量
        dq = pinv(J_pos_local) * (err_local * step_size);
        
        % 更新 q（注意 dq 为列向量，q 为列向量）
        q = q + dq;
    end
    
    % 用动画显示最终位形（可根据需要加延时）
    %arm.plot(q');
    q_final = q';
end
