
%% 机械臂设置
clear;
close all;
clc;
%角度转换
angle=pi/180;  %转化为角度制
%D-H参数表
theta1 = 0;   D1 = 0.13156;  A1 = 0;     alpha1 = 0;     offset1 = pi;
theta2 = 0;   D2 = 0.06639;  A2 = 0;     alpha2 = -pi/2; offset2 = -pi/2;
theta3 = 0;   D3 = 0;       A3 = 0.1104; alpha3 = pi;    offset3 = 0;
theta4 = 0;   D4 = 0;       A4 = 0.096; alpha4 = pi;    offset4 = -pi/2;
theta5 = 0;   D5 = 0.07318;  A5 = 0;     alpha5 = -pi/2; offset5 = pi/2;
theta6 = 0;   D6 = 0.0436;   A6 = 0;     alpha6 = pi/2;  offset6 = 0;

% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'standard'：建立标准型D-H参数
L(1) = Link([theta1, D1, A1, alpha1,0,offset1], 'modified');
L(2) = Link([theta2, D2, A2, alpha2,0, offset2], 'modified');
L(3) = Link([theta3, D3, A3, alpha3,0, offset3], 'modified');
L(4) = Link([theta4, D4, A4, alpha4,0, offset4], 'modified');
L(5) = Link([theta5, D5, A5, alpha5,0, offset5], 'modified');
L(6) = Link([theta6, D6, A6, alpha6,0, offset6], 'modified');
 
% 定义关节范围
L(1).qlim =[-168*angle, 168*angle];
L(2).qlim =[-135*angle, 135*angle];
L(3).qlim =[-150*angle, 150*angle];
L(4).qlim =[-145*angle, 145*angle];
L(5).qlim =[-165*angle, 165*angle];
L(6).qlim =[-180*angle, 180*angle];



% 显示机械臂（把上述连杆“串起来”）
arm_NanoB01= SerialLink(L,'name','Jetson Nano B01');

%角度转换
T1=transl(0.268,-0.015,0.214);%两个位置，绕过障碍物  R = -142.2 P = -74.1 Y = -57.4
T2=transl(-0.017,0.176,0.368);  % R = -7.9 P = 20.2 Y = 81.6

%正运动学参数
DHtable=[1 D1   A1     alpha1;
         2 D2   A2     alpha2;
         3 D3   A3     alpha3;
         4 D4   A4     alpha4;
         5 D5   A5     alpha5;
         6 D6   A6     alpha6];
q0=[pi,-pi/2,0,-pi/2,pi/2,0]; %初始角度

%% 障碍物

% 定义长方体的一个顶点、长、宽、高
R = 0.025;       %R为机械臂最大半径
vertex = [0.1, 0.1, 0.17]; % 顶点坐标    
length = 0.085+2*R;         % 长度    0.085 0.126 0.600
width = 0.126+2*R;          % 宽度
height = 0.6+2*R;         % 高度

% 调用函数   输出为长方体在xy，xz，yz面上的投影,左下角和右上角
%[a,b,c]=project_ju_xing(vertex, length, width, height);

%%  给出三维坐标，逆运动学   或者直接给出六个角度



 %q1= arm_NanoB01.ikunc(T1);
% q2 =arm_NanoB01.ikunc(T2);
q1 = [0 -78.8 6.8 72.5 89.4 0]*angle;  %改为弧度制 
q2 = [-84 50.6 31.3 -12.1 87.5 0]*angle;

q_array = [q1;q2];
q_array = q_array';     % 初末角度
v_array = zeros(6, 3);  % 速度初始化为0
a_array = zeros(6, 3);  % 加速度初始化为0
t_array = [0,2];
%K = [0.01;0.01;0.01;0.01;0.01;0.01];
% 粒子群
[gbest,gbest_value,gbest_t,gbest_all] = pso_6d(q_array,v_array,a_array,t_array,DHtable,vertex,length,width,height,R);
plot(gbest_t,gbest_all);



%% 粒子群函数
function [gbest, gbest_value,gbest_t,gbest_all] = pso_6d(q_array,v_array,a_array,t_array,DHtable,vertex,length,width,height,R)
    % 参数设置
    n_particles = 60;       % 粒子数量
    n_iterations = 200;     % 最大迭代次数
    w = 0.7;                % 惯性权重
    c1 = 1.5;               % 个体学习因子
    c2 = 1.5;               % 社会学习因子
    D = 6;                  % 参数维度（6 维列向量）
    lb = -30* ones(D, 1);  % 参数下界（6 维列向量）
    ub = 30 * ones(D, 1);   % 参数上界（6 维列向量）
    %K_orig = [   -0.0517    0.3555    0.2383   -0.0209    0.0214    0.0141]';
    % 初始化粒子位置和速度（每个粒子是 6 维列向量）
    particles = repmat(lb, 1, n_particles) + repmat(ub - lb, 1, n_particles) .* rand(D, n_particles);
    velocities = zeros(D, n_particles);  % 速度矩阵

    % 初始化个体最优和全局最优
    pbest = particles;  % 每个粒子的历史最优位置
    pbest_values = arrayfun(@(i) objective_function(particles(:,i),q_array,v_array,a_array,t_array,DHtable,vertex,length,width,height,R), 1:n_particles);
    [gbest_value, gbest_index] = min(pbest_values);
    gbest = particles(:, gbest_index);  % 全局最优位置
    %x_zhou = zeros(1,n_iterations);
    gbest_all = zeros(1,n_iterations);
    % 迭代优化
    for iter = 1:n_iterations
        for i = 1:n_particles
            % 更新速度
            r1 = rand(D, 1);
            r2 = rand(D, 1);
            velocities(:,i) = w * velocities(:,i) ...
                + c1 * r1 .* (pbest(:,i) - particles(:,i)) ...
                + c2 * r2 .* (gbest - particles(:,i));

            % 更新位置
            particles(:,i) = particles(:,i) + velocities(:,i);

            % 边界约束处理
            particles(:,i) = max(particles(:,i), lb);
            particles(:,i) = min(particles(:,i), ub);

            % 计算当前适应度
            current_value = objective_function(particles(:,i),q_array,v_array,a_array,t_array,DHtable,vertex,length,width,height,R);

            % 更新个体最优
            if current_value < pbest_values(i)
                pbest(:,i) = particles(:,i);
                pbest_values(i) = current_value;
            end
        end

        % 更新全局最优
        [new_min, new_min_index] = min(pbest_values);
        if new_min < gbest_value
            gbest = pbest(:, new_min_index);
            gbest_value = new_min;
        end

        % 输出迭代信息
        fprintf('Iteration %d: Best Value = %f\n', iter, gbest_value);
        gbest_all(iter) =gbest_value; 
        gbest_t(iter) = iter;
      
    end

    % 输出最终结果
    fprintf('\nOptimization Completed!\n');
    fprintf('Best Parameters:\n');
    disp(gbest');
    fprintf('Best Objective Value: %f\n', gbest_value);
end

%%  适应度函数
function  fitness = objective_function(K,q_array,v_array,a_array,t_array,DHtable,vertex,length,width,height,R)
%设置适应度函数
%因素包括碰撞+角度距离  ！不是位移
    
    [t,q_jtraj,v_traj,a_traj] = SDPOA_6D(q_array,v_array,a_array,t_array,K); %SDPOA

    
    % 平滑性计算
    smoothness = sum(a_traj(:).^2);
    penalties(1) = smoothness;
    
    %关节限制检查
    %joint_limits_2 = ones(6,1)*175;
    joint_limits_2 = [168;135;150;145;165;180]*pi/180;
    joint_limits_1 = [-168;-135;-150;-145;-165;-180]*pi/180;
    limit_violation = sum(max(0, q_jtraj - joint_limits_2) + ...
                        max(0, joint_limits_1 - q_jtraj));
    limit_violation = sum(limit_violation);              
    penalties(2) = limit_violation;
    
    % 避障
    q0=[pi,-pi/2,0,-pi/2,pi/2,0]; %初始角度
    collision = 0;
    collision_self = 0;
    joint_T = cell(1,6);
    joint_T = cell2mat(joint_T);
    q_jtraj_T = q_jtraj'; %这里输入的规格不同，需要转置
    % 找到各个关节相对于基坐标系的变换矩阵
    for i = 1:size(q_jtraj_T,1)   % 该输入为n×6
        last_joint = eye(4);
        q_set = q_jtraj_T(i,:);%设置角度

        q_now = q0+q_set;
        [~,H_i]=NanoBO1_fkin(DHtable,q_now);
        for y = 1:6
        joint_T(:,:,y) =last_joint * H_i{y};   %joint_T 为各个关节相对基坐标系的变换矩阵
        last_joint = joint_T(:,:,y);
        end

       % 从变换矩阵中提炼出三维坐标
       location_0 = [0 0 0]; %基坐标系
       location_1 = joint_T(:,:,1);location_2 = joint_T(:,:,2);location_3 = joint_T(:,:,3);
       location_4 = joint_T(:,:,4);location_5 = joint_T(:,:,5);location_6 = joint_T(:,:,6);

       location_1 = location_1(1:3,4);location_2 = location_2(1:3,4);location_3 = location_3(1:3,4);
       location_4 = location_4(1:3,4);location_5 = location_5(1:3,4); location_6 = location_6(1:3,4);

       joint_total = horzcat(location_1,location_2,location_3,location_4,location_5,location_6);


       joint_total_T = joint_total'; %三维坐标的转置

        for j = 1:5               %判断和长方体是否碰撞
            intersect = segment_cuboid_intersection(joint_total(:,j)',joint_total(:,j+1)',vertex,length,width,height);
            if intersect == 1
                collision = collision+1;  %判断碰撞次数
            end
        end
        
        %自碰撞
%         for k = 1:3 %机械臂之间的碰撞
% 
%             [dis_1,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+2,:),joint_total_T(k+3,:));
%             if dis_1 < 2*R
%                 collision_self = 1;
%                 break;
%             end
%             if k < 3
%             [dis_2,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+3,:),joint_total_T(k+4,:));
%                 if dis_2 < 2*R
%                     collision_self = 1;
%                     break;
%                 end
%             end
%             if k < 2
%             [dis_3,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+4,:),joint_total_T(k+5,:));
%             if dis_3 < 2*R
%                 collision_self = 1;
%                 break;
%             end
%             end
% 
%         end
%         
        
        


    end
    penalties(3) = collision+collision_self;
    
    weights = [0.000001 0.01 1];
    fitness = dot(weights,penalties);

    
    
end
