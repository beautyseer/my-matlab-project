clear
clc

% % 使用符号工具箱定义符号变量
syms theta1 theta2 theta3 theta4 theta5 theta6 real
syms a0 a1 a2 a3 a4 a5 real
syms d1 d2 d3 d4 d5 d6 real
syms c1 c2 c3 c4 c5 c6 s1 s2 s3 s4 s5 s6
% 
% 定义余弦和正弦符号变量
c1 = cos(theta1); s1 = sin(theta1);
c2 = cos(theta2); s2 = sin(theta2);
c3 = cos(theta3); s3 = sin(theta3);
c4 = cos(theta4); s4 = sin(theta4);
c5 = cos(theta5); s5 = sin(theta5);
c6 = cos(theta6); s6 = sin(theta6);


%syms c1 c2 c3 c4 c5 c6 s1 s2 s3 s4 s5 s6
% 定义齐次变换矩阵
T1_0 = [c1, -s1, 0, a0;
        s1,  c1, 0, 0;
        0,    0, 1, d1;
        0,    0, 0, 1];

T2_1 = [c2, -s2, 0, a1;
        0,    0, 1, d2;
       -s2, -c2, 0, 0;
        0,    0, 0, 1];

T3_2 = [c3, -s3, 0, a2;
       -s3, -c3, 0, 0;
        0,    0, -1, -d3;
        0,    0, 0, 1];

T4_3 = [c4, -s4, 0, a3;
       -s4, -c4, 0, 0;
        0,    0, -1, -d4;
        0,    0, 0, 1];

T5_4 = [c5, -s5, 0, a4;
        0,    0, 1, d5;
       -s5, -c5, 0, 0;
        0,    0, 0, 1];

T6_5 = [c6, -s6, 0, a5;
        0,    0, -1, -d6;
        s6,  c6, 0, 0;
        0,    0, 0, 1];

% 计算从基座到末端执行器的总变换矩阵   
T6_0 = simplify(T1_0 * T2_1 * T3_2 * T4_3 * T5_4 * T6_5);

%% 参考论文的公式和坐标系构建
clear 
clc
syms a0 a1 a2 a3 a4 a5 real
syms d1 d2 d3 d4 d5 d6 real
syms c1 c2 c3 c4 c5 c6 s1 s2 s3 s4 s5 s6
syms theta1 theta2 theta3 theta4 theta5 theta6 real
% 定义余弦和正弦符号变量
c1 = cos(theta1); s1 = sin(theta1);
c2 = cos(theta2); s2 = sin(theta2);
c3 = cos(theta3); s3 = sin(theta3);
c4 = cos(theta4); s4 = sin(theta4);
c5 = cos(theta5); s5 = sin(theta5);
c6 = cos(theta6); s6 = sin(theta6);


% 定义齐次变换矩阵
T1_0 = [c1, -s1, 0, a0;
        s1,  c1, 0, 0;
        0,    0, 1, d1;
        0,    0, 0, 1];

T2_1 = [c2, -s2, 0, a1;
        0,    0, 1, d2;
       -s2, -c2, 0, 0;
        0,    0, 0, 1];

T3_2 = [c3, -s3, 0, a2;
       -s3, -c3, 0, 0;
        0,    0, -1, -d3;
        0,    0, 0, 1];

T4_3 = [c4, -s4, 0, a3;
       -s4, -c4, 0, 0;
        0,    0, -1, -d4;
        0,    0, 0, 1];

T5_4 = [c5, -s5, 0, a4;
        0,    0, 1, d5;
       -s5, -c5, 0, 0;
        0,    0, 0, 1];

T6_5 = [c6, -s6, 0, a5;
        0,    0, -1, -d6;
        s6,  c6, 0, 0;
        0,    0, 0, 1];

% 计算从基座到末端执行器的总变换矩阵   
T6_0 = simplify(T1_0 * T2_1 * T3_2 * T4_3 * T5_4 * T6_5)

%%  雅可比矩阵计算过程
T6_5 = simplify(T6_5);
T6_4 = simplify(T5_4*T6_5);
T6_3 = simplify(T4_3 * T5_4 * T6_5);
T6_2 = simplify(T3_2 * T4_3 * T5_4 * T6_5);
T6_1 = simplify(T2_1 * T3_2 * T4_3 * T5_4 * T6_5);

%求J1
P1 = [ T6_0(1,4)
    T6_0(2,4)
   ];
noa_z = [T6_0(3,1)
    T6_0(3,2)
    T6_0(3,3)
    ];
N_xy = [T6_0(1,1)
    T6_0(2,1)
    ];
O_xy = [T6_0(1,2)
    T6_0(2,2)
    ];
A_xy = [T6_0(1,3)
    T6_0(2,3)
    ];
J1 = [ cross2d(P1,N_xy)
    cross2d(P1,O_xy)
    cross2d(P1,A_xy)
    noa_z
    ];

%求J2
P1 = [ T6_1(1,4)
    T6_1(2,4)
    ];
noa_z = [T6_1(3,1)
    T6_1(3,2)
    T6_1(3,3)
    ];
N_xy = [T6_1(1,1)
    T6_1(2,1)
    ];
O_xy = [T6_1(1,2)
    T6_1(2,2)
    ];
A_xy = [T6_1(1,3)
    T6_1(2,3)
    ];
J2 = [ cross2d(P1,N_xy)
    cross2d(P1,O_xy)
    cross2d(P1,A_xy)
    noa_z
    ];

%J3
P1 = [ T6_2(1,4)
    T6_2(2,4)
   ];
noa_z = [T6_2(3,1)
    T6_2(3,2)
    T6_2(3,3)
    ];
N_xy = [T6_2(1,1)
    T6_2(2,1)
    T6_2(3,1)];
O_xy = [T6_2(1,2)
    T6_2(2,2)
    T6_2(3,2)];
A_xy = [T6_2(1,3)
    T6_2(2,3)
    T6_2(3,3)];
N1 = cross2d(P1,N_xy);
O1 = cross2d(P1,O_xy);
Z1 = cross2d(P1,A_xy);
J3 = [ N1
     O1
     Z1
     noa_z
    ];

%J4
P1 = [ T6_3(1,4)
    T6_3(2,4)
    ];
noa_z = [T6_3(3,1)
    T6_3(3,2)
    T6_3(3,3)
    ];
N_xy = [T6_3(1,1)
    T6_3(2,1)
    T6_3(3,1)];
O_xy = [T6_3(1,2)
    T6_3(2,2)
    T6_3(3,2)];
A_xy = [T6_3(1,3)
    T6_3(2,3)
    T6_3(3,3)];
N1 = cross2d(P1,N_xy);
O1 = cross2d(P1,O_xy);
Z1 = cross2d(P1,A_xy);
J4 = [ N1
     O1
     Z1
     noa_z
    ];



%J5
P1 = [ T6_4(1,4)
    T6_4(2,4)
    ];
noa_z = [T6_4(3,1)
    T6_4(3,2)
    T6_4(3,3)
    ];
N_xy = [T6_4(1,1)
    T6_4(2,1)
   ];
O_xy = [T6_4(1,2)
    T6_4(2,2)
    ];
A_xy = [T6_4(1,3)
    T6_4(2,3)
    ];
N1 = cross2d(P1,N_xy);
O1 = cross2d(P1,O_xy);
Z1 = cross2d(P1,A_xy);
J5 = [ N1
     O1
     Z1
     noa_z
    ];






%J6  
P1 = [ T6_5(1,4)
    T6_5(2,4)
    ];
noa_z = [T6_5(3,1)
    T6_5(3,2)
    T6_5(3,3)
    ];
N_xy = [T6_5(1,1)
    T6_5(2,1)
   ];
O_xy = [T6_5(1,2)
    T6_5(2,2)
    ];
A_xy = [T6_5(1,3)
    T6_5(2,3)
    ];
N1 = cross2d(P1,N_xy);
O1 = cross2d(P1,O_xy);
Z1 = cross2d(P1,A_xy);
J6 = [ N1
     O1
     Z1
     noa_z
    ];
J_last = [0
    0
    0
    0
    0
    1];

J = [simplify(J2) simplify(J3) simplify(J4) simplify(J5) simplify(J6) J_last];%有问题，最后一列应为000001,






%% 雅可比矩阵  CSDN代码

% 给定关节角度
q=[15*(pi/180) -15*(pi/180) 45*(pi/180) -15*(pi/180) 15*(pi/180) 15*(pi/180)];
 
% IRB2600 MDH
th(1) = q(1);               d(1) = -0.13156;             a(1) = 0;              alp(1) = 0;
th(2) = q(2)-pi/2;          d(2) = -0.06639;               a(2) = 0;            alp(2) = pi/2;   
th(3) = q(3);               d(3) = 0;               a(3) = 0.1104;            alp(3) = pi;
th(4) = q(4)+pi/2;               d(4) = 0;             a(4) = 0.096;            alp(4) = pi;
th(5) = q(5);               d(5) = -0.07318;               a(5) = 0;              alp(5) = -pi/2;
th(6) = q(6);            d(6) = 0;              a(6) = 0;              alp(6) = pi/2;
 
% 初始化矩阵T
T=zeros(4,4,6);
 
% 计算各关节间的齐次变换矩阵
for i=1:6
    T(:,:,i)=[cos(th(i))                  -sin(th(i))                    0                              a(i);
              sin(th(i))*cos(alp(i))       cos(th(i))*cos(alp(i))       -sin(alp(i))       -d(i)*sin(alp(i));
              sin(th(i))*sin(alp(i))       cos(th(i))*sin(alp(i))        cos(alp(i))        d(i)*cos(alp(i));
              0                0                             0                                            1];
end
T01 = T(:,:,1);
T12 = T(:,:,2);
T23 = T(:,:,3);
T34 = T(:,:,4);
T45 = T(:,:,5);
T56 = T(:,:,6);
T06=T01*T12*T23*T34*T45*T56;

 
% J1
R01=[T01(1,1) T01(1,2) T01(1,3)
     T01(2,1) T01(2,2) T01(2,3)
     T01(3,1) T01(3,2) T01(3,3)];
 z1=[ T01(1,3)
      T01(2,3)
      T01(3,3)];
 T16=T12*T23*T34*T45*T56;
 p1=[T16(1,4)
     T16(2,4) 
     T16(3,4)];
 P1=R01*p1;
 Z1=cross(z1,P1);
 J1=[Z1
     z1];
 
% J2
T02=T01*T12;
R02=[T02(1,1) T02(1,2) T02(1,3)
     T02(2,1) T02(2,2) T02(2,3)
     T02(3,1) T02(3,2) T02(3,3)];
 z2=[T02(1,3)
     T02(2,3)
     T02(3,3)];
 T26=T23*T34*T45*T56;
 p2=[T26(1,4)
     T26(2,4)
     T26(3,4)];
 P2=R02*p2;
 Z2=cross(z2,P2);
 J2=[Z2
     z2];
 
 % J3
T03=T01*T12*T23;
R03=[T03(1,1) T03(1,2) T03(1,3)
     T03(2,1) T03(2,2) T03(2,3)
     T03(3,1) T03(3,2) T03(3,3)];
 z3=[T03(1,3)
     T03(2,3)
     T03(3,3)];
 T36=T34*T45*T56;
 p3=[T36(1,4)
     T36(2,4)
     T36(3,4)];
 P3=R03*p3;
 Z3=cross(z3,P3);
 J3=[Z3
     z3];
 
 % J4
T04=T01*T12*T23*T34;
R04=[T04(1,1) T04(1,2) T04(1,3)
     T04(2,1) T04(2,2) T04(2,3)
     T04(3,1) T04(3,2) T04(3,3)];
 z4=[T04(1,3)
     T04(2,3)
     T04(3,3)];
 T46=T45*T56;
 p4=[T46(1,4)
     T46(2,4)
     T46(3,4)];
 P4=R04*p4;
 Z4=cross(z4,P4);
 J4=[Z4
     z4];
 
 % J5
T05=T01*T12*T23*T34*T45;
R05=[T05(1,1) T05(1,2) T05(1,3)
     T05(2,1) T05(2,2) T05(2,3)
     T05(3,1) T05(3,2) T05(3,3)];
 z5=[T05(1,3)
     T05(2,3)
     T05(3,3)];
 p5=[T56(1,4)
     T56(2,4)
     T56(3,4)];
 P5=R05*p5;
 Z5=cross(z5,P5);
 J5=[Z5
     z5];
 
 % J6
R06=[T06(1,1) T06(1,2) T06(1,3)
     T06(2,1) T06(2,2) T06(2,3)
     T06(3,1) T06(3,2) T06(3,3)];
 z6=[T06(1,3)
     T06(2,3)
     T06(3,3)];
 p6=[0
     0 
     0];
 P6=R06*p6;
 Z6=cross(z6,P6);
 J6=[Z6
     z6];
 
J=[J1 J2 J3 J4 J5 J6];
%%

function result = cross2d(a, b)
    % cross2d2D 计算二维向量的标量叉积
    % 输入：a, b - 二维向量 [x, y]
    % 输出：标量叉积 a_x*b_y - a_y*b_x
    result = a(1)*b(2) - a(2)*b(1);
end

