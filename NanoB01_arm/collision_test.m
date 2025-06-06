
clear;
close all;
clc;
 K = [  -0.0249   -0.1572    0.0171   -0.0680    0.0549    0.0981]';
 
 

T1=transl(0.268,-0.015,0.214);%两个位置，绕过障碍物  R = -142.2 P = -74.1 Y = -57.4
T2=transl(-0.017,0.176,0.368);  % R = -7.9 P = 20.2 Y = 81.6
%角度转换
angle=pi/180;  %转化为角度制
 
% %D-H参数表
% theta1 = 0;   D1 = 0.0985;  A1 = 0;     alpha1 = 0;     offset1 = pi;
% theta2 = 0;   D2 = 0.1215;  A2 = 0;     alpha2 = -pi/2; offset2 = -pi/2;
% theta3 = 0;   D3 = 0;       A3 = 0.408; alpha3 = pi;    offset3 = 0;
% theta4 = 0;   D4 = 0;       A4 = 0.376; alpha4 = pi;    offset4 = -pi/2;
% theta5 = 0;   D5 = 0.1025;  A5 = 0;     alpha5 = -pi/2; offset5 = 0;
% theta6 = 0;   D6 = 0.094;   A6 = 0;     alpha6 = pi/2;  offset6 = 0;



%D-H参数表
theta1 = 0;   D1 = 0.13156;  A1 = 0;     alpha1 = 0;     offset1 = pi;
theta2 = 0;   D2 = 0.06639;  A2 = 0;     alpha2 = -pi/2; offset2 = -pi/2;
theta3 = 0;   D3 = 0;       A3 = 0.1104; alpha3 = pi;    offset3 = 0;
theta4 = 0;   D4 = 0;       A4 = 0.096; alpha4 = pi;    offset4 = -pi/2;
theta5 = 0;   D5 = 0.07318;  A5 = 0;     alpha5 = -pi/2; offset5 = pi/2;
theta6 = 0;   D6 = 0.0436;   A6 = 0;     alpha6 = pi/2;  offset6 = 0;
 
% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'standard'：建立标准型D-H参数
L(1) = Link([theta1, D1, A1, alpha1,0, offset1], 'modified');
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
arm_NanoB01 = SerialLink(L,'name','NanoBO1');

DHtable=[1 0.0985 0 0;
         2 0.1215 0 -pi/2;
         3 0 0.408 pi;
         4 0 0.376 pi;
         5 0.1025 0 -pi/2;
         6 0.094 0 pi/2];
%%
% q1= arm_NanoB01.ikunc(T1);    %返回弧度制
% q2 =arm_NanoB01.ikunc(T2);
%%
q1 = [0 -78.8 6.8 72.5 89.4 0]*angle;  %改为弧度制 
q2 = [-84 50.6 31.3 -12.1 87.5 0]*angle;



q_array = [q1;q2];
q_array = q_array';     % 初末角度
v_array = zeros(6, 2);  % 速度初始化为0
a_array = zeros(6, 2);  % 加速度初始化为0
t_array = [0,2];




[t_, q, v, a] = SDPOA_6D(q_array, v_array, a_array, t_array, K);  %输入输出均为6×n


R = 0.025;       %R为机械臂最大半径
vertex = [0.1, 0.1, 0.17]; % 顶点坐标    
length = 0.085+2*R;         % 长度    0.085 0.126 0.600
width = 0.126+2*R;          % 宽度
height = 0.6+2*R; 




q0=[pi,-pi/2,0,-pi/2,pi/2,0]; %初始角度
collision = 0;
collision_self = 0;

joint_T = cell(1,6);
joint_T = cell2mat(joint_T);
q_jtraj = q';

% 找到各个关节相对于基坐标系的变换矩阵
for i = 1:size(q_jtraj,1)
    last_joint = eye(4);
    q_set = q_jtraj(i,:);%设置角度
    
    q_now = q0+q_set;
    [H_t2o,H_i]=aubo_fkin(DHtable,q_now);
    for y = 1:6
        joint_T(:,:,y) =last_joint * H_i{y};   %joint_T 为各个关节相对基坐标系的变换矩阵
        last_joint = joint_T(:,:,y);
    end
   
   % 从变换矩阵中提炼出三维坐标
   location_1 = joint_T(:,:,1);location_2 = joint_T(:,:,2);location_3 = joint_T(:,:,3);
   location_4 = joint_T(:,:,4);location_5 = joint_T(:,:,5);location_6 = joint_T(:,:,6);
   
   location_1 = location_1(1:3,4);location_2 = location_2(1:3,4);location_3 = location_3(1:3,4);
   location_4 = location_4(1:3,4);location_5 = location_5(1:3,4); location_6 = location_6(1:3,4);
    
   joint_total = horzcat(location_1,location_2,location_3,location_4,location_5,location_6);
    
    
    joint_total_T = joint_total'; %三维坐标的转置
     
    for j = 1:5               %判断和长方体是否碰撞
        intersect = segment_cuboid_intersection(joint_total(:,j)',joint_total(:,j+1)', vertex,length, width, height);%collision_detect(joint_total(:,j),joint_total(:,j+1),vertex,length,width,height);
        if intersect == 1
            collision = collision+1;  %判断碰撞次数
        end
    end
    
%     for k = 1:3 %机械臂之间的碰撞
%         
%         [dis_1,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+2,:),joint_total_T(k+3,:));
%         if k < 3
%         [dis_2,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+3,:),joint_total_T(k+4,:));
%         end
%         if k < 2
%         [dis_3,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+4,:),joint_total_T(k+5,:));
%         end
%         if dis_1 < 2*R || dis_2 < 2*R || dis_3 < 2*R
%             collision_self = collision_self + 1;
%         end
%     end


%         自碰撞
        for k = 1:3 %机械臂之间的碰撞

            [dis_1,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+2,:),joint_total_T(k+3,:));
            if dis_1 < 2*R
                collision_self = 1;
                break;
            end
            if k < 3
            [dis_2,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+3,:),joint_total_T(k+4,:));
                if dis_2 < 2*R
                    collision_self = 1;
                    break;
                end
            end
            if k < 2
            [dis_3,~,~,~,~] = distance_between_lines(joint_total_T(k,:),joint_total_T(k+1,:),joint_total_T(k+4,:),joint_total_T(k+5,:));
            if dis_3 < 2*R
                collision_self = 1;
                break;
            end
            end

        end
%         
end


%% 绘制动画
    x0 = vertex(1);
    y0 = vertex(2);
    z0 = vertex(3);

    % 计算长方体的 8 个顶点坐标
    vertices = [
        x0, y0, z0;
        x0 + length, y0, z0;
        x0 + length, y0 + width, z0;
        x0, y0 + width, z0;
        x0, y0, z0 + height;
        x0 + length, y0, z0 + height;
        x0 + length, y0 + width, z0 + height;
        x0, y0 + width, z0 + height
        ];
    
 %绘制长方体
    figure(1);
    hold on;
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('长方体及其投影');

    % 绘制长方体
    faces = [
        1, 2, 3, 4;  % 底面
        5, 6, 7, 8;  % 顶面
        1, 2, 6, 5;  % 前面
        2, 3, 7, 6;  % 右面
        3, 4, 8, 7;  % 后面
        4, 1, 5, 8   % 左面
    ];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'cyan', 'FaceAlpha', 0.5);



arm_NanoB01.plot(q_jtraj);

figure(2);
subplot(3,1,1),plot(t_,q,'r'),xlabel('t'),ylabel('position');
hold on;
%plot(t_array,q_array,'color','g'),grid on;
subplot(3,1,2),plot(t_,v,'b'),xlabel('t'),ylabel('velocity');
hold on;
%plot(t_array,v_array,'color','y'),grid on;
subplot(3,1,3),plot(t_,a,'g'),xlabel('t'),ylabel('accelerate');
hold on;
%plot(t_array,a_array,'color','r'),grid on;


disp("障碍物碰撞为：")
disp(collision)
disp("自碰撞为：")
disp(collision_self)

dlmwrite('D:\zhang_jie_code\myrobot_arm\q_jtraj.csv',q_jtraj,',')


dlmwrite('D:\zhang_jie_code\myrobot_arm\q_jtraj.csv',q,',')
dlmwrite('D:\zhang_jie_code\myrobot_arm\v_jtraj.csv',v,',')
dlmwrite('D:\zhang_jie_code\myrobot_arm\t_jtraj.csv',t_,',')
