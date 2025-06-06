
clear;
close all;
clc;
 
%角度转换
angle=pi/180;  %转化为角度制

%D-H参数表   nanoBO1参数，csdn结构
theta1 = 0;   D1 = 0.13156;  A1 = 0;     alpha1 = 0;     offset1 = pi;
theta2 = 0;   D2 = 0.06639;  A2 = 0;     alpha2 = -pi/2; offset2 = -pi/2;
theta3 = 0;   D3 = 0;       A3 = 0.1104; alpha3 = pi;    offset3 = 0;
theta4 = 0;   D4 = 0;       A4 = 0.096; alpha4 = pi;    offset4 = -pi/2;
theta5 = 0;   D5 = 0.07318;  A5 = 0;     alpha5 = -pi/2; offset5 = pi/2;
theta6 = 0;   D6 = 0.0486;   A6 = 0;     alpha6 = pi/2;  offset6 = 0;

 
%D-H参数表
% theta1 = 0;   D1 = -0.13156;  A1 = 0;      alpha1 = pi;     offset1 = 0;
% theta2 = 0;   D2 = -0.06639;  A2 = 0;      alpha2 = pi/2; offset2 = -pi/2;
% theta3 = 0;   D3 = 0;         A3 = 0.1104; alpha3 = pi;    offset3 = 0;
% theta4 = 0;   D4 = 0;         A4 = 0.096;  alpha4 = pi;    offset4 = pi/2;
% theta5 = 0;   D5 = -0.07318;  A5 = 0;      alpha5 = -pi/2; offset5 = 0;    %在实际机械臂中，末端Z轴与基坐标系X轴正方向相同；X轴与基坐标系Y轴负方向相同
% theta6 = 0;   D6 = 0;         A6 = 0;      alpha6 = -pi/2;  offset6 = 0;


% % AUBO  D-H参数表
% theta1 = 0;   D1 = -0.0985;  A1 = 0;     alpha1 = pi;     offset1 = 0;
% theta2 = 0;   D2 = -0.1215;  A2 = 0;     alpha2 = pi/2; offset2 = -pi/2;
% theta3 = 0;   D3 = 0;       A3 = 0.408; alpha3 = pi;    offset3 = 0;
% theta4 = 0;   D4 = 0;       A4 = 0.376; alpha4 = pi;    offset4 = pi/2;
% theta5 = 0;   D5 = -0.1025;  A5 = 0;     alpha5 = -pi/2; offset5 = 0;
% theta6 = 0;   D6 = 0;   A6 = 0;     alpha6 = pi/2;  offset6 = 0;
 
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
 %theta0 = [0 0 0 0 0 0];				%初始关节角度 figure(1)

% robot_auboi5.plot(theta0);
q1 = [25.92, 94.39, -0.96, -85.69, 55.54, 134.56];
q2 = [25.92, 70.48, -1.23, -74.00, 56.51, 133.94];
q3 = [26.54, 94.30, -0.96, -95.36, 56.95, 124.01];
q4 = [89.12, 70.66, -1.05, -72.15, 7.29, 115.75];

axis([-0.6 0.6 -0.6 0.6 -0.6 0.6]); 
arm_NanoB01.teach

arm_NanoB01.fkine(q1)

%% 正运动学参数
DHtable=[1 D1   A1     alpha1;
         2 D2   A2     alpha2;
         3 D3   A3     alpha3;
         4 D4   A4     alpha4;
         5 D5   A5     alpha5;
         6 D6   A6     alpha6];
%q0=[0,-pi/2,0,pi/2,0,0]; %初始角度
q0 = [pi,-pi/2,0,-pi/2,pi/2,0];
q_set = [0,0,0,0,0,0];%设置角度
q_now = q0+q_set;
[H_t2o,H_i]=aubo_fkin(DHtable,q_now);%此为用具体角度进行求值
H_t2o

%% 绘制障碍物

hold on
% 定义长方体的一个顶点、长、宽、高
R = 0.05;       %R为机械臂最大半径
vertex = [0.1, 0.1, 0.17]; % 顶点坐标    
length = 0.085+2*R;         % 长度    0.085 0.126 0.600
width = 0.126+2*R;          % 宽度
height = 0.6+2*R;         % 高度

% 调用函数   输出为长方体在xy，xz，yz面上的投影,左下角和右上角
[a,b,c]=project_ju_xing_plot(vertex, length, width, height);
