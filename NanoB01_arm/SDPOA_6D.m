function [t, q, v, a] = SDPOA_6D(q_array, v_array, a_array, t_array, K)
    % 输入说明：
    %   q_array: 6×n 矩阵，每一行对应一个维度的位置序列
    %   v_array: 6×n 矩阵，每一行对应一个维度的速度序列
    %   a_array: 6×n 矩阵，每一行对应一个维度的加速度序列
    %   t_array: 1×n 时间节点序列
    %   K: 6×1 向量，每个维度对应一个 K 值
    % 输出：
    %   t: 1×N 时间序列
    %   q: 6×N 轨迹位置
    %   v: 6×N 轨迹速度
    %   a: 6×N 轨迹加速度

    % 初始化
    num_dims = 6;  % 固定6维
    t = t_array(1);
    q = q_array(:, 1);  % 6×1
    v = v_array(:, 1);  % 6×1
    a = a_array(:, 1);  % 6×1

    % 遍历每个维度
    for dim = 1:num_dims
        % 提取当前维度的数据和对应的 K
        q_dim = q_array(dim, :);
        v_dim = v_array(dim, :);
        a_dim = a_array(dim, :);
        K_dim = K(dim);  % 当前维度的 K 值
        
        % 初始化当前维度轨迹
        t_dim = t_array(1);
        q_dim_traj = q_dim(1);
        v_dim_traj = v_dim(1);
        a_dim_traj = a_dim(1);
        
        % 遍历时间段
        for i = 1:length(q_dim)-1
            T = t_array(i+1) - t_array(i);
            q0 = q_dim(i);
            v0 = v_dim(i);
            a0 = a_dim(i);
            q1 = q_dim(i+1);
            v1 = v_dim(i+1);
            a1 = a_dim(i+1);
            
            % 使用当前维度的 K_dim 计算系数
            a3 = (20*q1 - 20*q0 - (8*v1 + 12*v0)*T - (3*a0 - a1)*T^2 - 2*K_dim*T^6) / (2*T^3);
            a4 = (30*q0 - 30*q1 + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T^2 + 6*K_dim*T^6) / (2*T^4);
            a5 = (12*q1 - 12*q0 - (6*v1 + 6*v0)*T - (a0 - a1)*T^2 - 6*K_dim*T^6) / (2*T^5);
            
            % 生成时间序列
            ti = t_array(i):0.05:t_array(i+1);
            tau = ti - t_array(i);
            
            % 计算轨迹（使用当前维度的 K_dim）
            qi = q0 + v0*tau + 0.5*a0*tau.^2 + a3*tau.^3 + a4*tau.^4 + a5*tau.^5 + K_dim*tau.^6;
            vi = v0 + a0*tau + 3*a3*tau.^2 + 4*a4*tau.^3 + 5*a5*tau.^4 + 6*K_dim*tau.^5;
            ai = a0 + 6*a3*tau + 12*a4*tau.^2 + 20*a5*tau.^3 + 30*K_dim*tau.^4;
            
            % 拼接轨迹
            t_dim = [t_dim, ti(2:end)];
            q_dim_traj = [q_dim_traj, qi(2:end)];
            v_dim_traj = [v_dim_traj, vi(2:end)];
            a_dim_traj = [a_dim_traj, ai(2:end)];
        end
        
        % 存入输出矩阵
        if dim == 1
            % 初始化输出矩阵（根据第一个维度长度）
            q = zeros(num_dims, length(t_dim));
            v = zeros(num_dims, length(t_dim));
            a = zeros(num_dims, length(t_dim));
            t = t_dim;
        end
        q(dim, :) = q_dim_traj;
        v(dim, :) = v_dim_traj;
        a(dim, :) = a_dim_traj;
    end
end