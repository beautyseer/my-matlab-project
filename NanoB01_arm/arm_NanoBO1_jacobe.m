function J = arm_NanoBO1_jacobe(q_now)

%雅可比矩阵，带入具体角度即可输出
q0 = [pi,-pi/2,0,-pi/2,pi/2,0];
q_now = q_now+q0;
D1 = 0.13156;  A1 = 0;     alpha1 = 0;     offset1 = pi;
 D2 = 0.06639;  A2 = 0;     alpha2 = -pi/2; offset2 = -pi/2;
 D3 = 0;       A3 = 0.1104; alpha3 = pi;    offset3 = 0;
 D4 = 0;       A4 = 0.096; alpha4 = pi;    offset4 = -pi/2;
 D5 = 0.07318;  A5 = 0;     alpha5 = -pi/2; offset5 = pi/2;
 D6 = 0.0486;   A6 = 0;     alpha6 = pi/2;  offset6 = 0;
d1 = D1;
d2 = D2;
d3 = D3;
d4 = D4;
d5 = D5;
d6 = D6;
a0 = A1;
a1 = A2;
a2 = A3;
a3 = A4;
a4 = A5;
a5 = A6;
%输入各个角度的值
theta1 = q_now(1);
theta2 = q_now(2);
theta3 = q_now(3);
theta4 = q_now(4);
theta5 = q_now(5);
theta6 = q_now(6);
 
J1 = [ 
- (cos(theta6)*(sin(theta1)*sin(theta5) + cos(theta2 - theta3 + theta4)*cos(theta1)*cos(theta5)) - sin(theta2 - theta3 + theta4)*cos(theta1)*sin(theta6))*(d2*cos(theta1) - d3*cos(theta1) + d4*cos(theta1) + a1*sin(theta1) + a4*cos(theta2 - theta3 + theta4)*sin(theta1) - d5*sin(theta2 - theta3 + theta4)*sin(theta1) + d6*cos(theta1)*cos(theta5) + a2*cos(theta2)*sin(theta1) - a5*cos(theta1)*sin(theta5) + a3*cos(theta2 - theta3)*sin(theta1) + d6*cos(theta2 - theta3 + theta4)*sin(theta1)*sin(theta5) + a5*cos(theta2 - theta3 + theta4)*cos(theta5)*sin(theta1)) - (cos(theta6)*(cos(theta1)*sin(theta5) - cos(theta2 - theta3 + theta4)*cos(theta5)*sin(theta1)) + sin(theta2 - theta3 + theta4)*sin(theta1)*sin(theta6))*(a0 + a1*cos(theta1) - d2*sin(theta1) + d3*sin(theta1) - d4*sin(theta1) + a4*cos(theta2 - theta3 + theta4)*cos(theta1) - d5*sin(theta2 - theta3 + theta4)*cos(theta1) + a2*cos(theta1)*cos(theta2) - d6*cos(theta5)*sin(theta1) + a5*sin(theta1)*sin(theta5) + a3*cos(theta2 - theta3)*cos(theta1) + d6*cos(theta2 - theta3 + theta4)*cos(theta1)*sin(theta5) + a5*cos(theta2 - theta3 + theta4)*cos(theta1)*cos(theta5))
(sin(theta6)*(sin(theta1)*sin(theta5) + cos(theta2 - theta3 + theta4)*cos(theta1)*cos(theta5)) + sin(theta2 - theta3 + theta4)*cos(theta1)*cos(theta6))*(d2*cos(theta1) - d3*cos(theta1) + d4*cos(theta1) + a1*sin(theta1) + a4*cos(theta2 - theta3 + theta4)*sin(theta1) - d5*sin(theta2 - theta3 + theta4)*sin(theta1) + d6*cos(theta1)*cos(theta5) + a2*cos(theta2)*sin(theta1) - a5*cos(theta1)*sin(theta5) + a3*cos(theta2 - theta3)*sin(theta1) + d6*cos(theta2 - theta3 + theta4)*sin(theta1)*sin(theta5) + a5*cos(theta2 - theta3 + theta4)*cos(theta5)*sin(theta1)) + (sin(theta6)*(cos(theta1)*sin(theta5) - cos(theta2 - theta3 + theta4)*cos(theta5)*sin(theta1)) - sin(theta2 - theta3 + theta4)*cos(theta6)*sin(theta1))*(a0 + a1*cos(theta1) - d2*sin(theta1) + d3*sin(theta1) - d4*sin(theta1) + a4*cos(theta2 - theta3 + theta4)*cos(theta1) - d5*sin(theta2 - theta3 + theta4)*cos(theta1) + a2*cos(theta1)*cos(theta2) - d6*cos(theta5)*sin(theta1) + a5*sin(theta1)*sin(theta5) + a3*cos(theta2 - theta3)*cos(theta1) + d6*cos(theta2 - theta3 + theta4)*cos(theta1)*sin(theta5) + a5*cos(theta2 - theta3 + theta4)*cos(theta1)*cos(theta5))
 (cos(theta1)*cos(theta5) + cos(theta2 - theta3 + theta4)*sin(theta1)*sin(theta5))*(a0 + a1*cos(theta1) - d2*sin(theta1) + d3*sin(theta1) - d4*sin(theta1) + a4*cos(theta2 - theta3 + theta4)*cos(theta1) - d5*sin(theta2 - theta3 + theta4)*cos(theta1) + a2*cos(theta1)*cos(theta2) - d6*cos(theta5)*sin(theta1) + a5*sin(theta1)*sin(theta5) + a3*cos(theta2 - theta3)*cos(theta1) + d6*cos(theta2 - theta3 + theta4)*cos(theta1)*sin(theta5) + a5*cos(theta2 - theta3 + theta4)*cos(theta1)*cos(theta5)) + (cos(theta5)*sin(theta1) - cos(theta2 - theta3 + theta4)*cos(theta1)*sin(theta5))*(d2*cos(theta1) - d3*cos(theta1) + d4*cos(theta1) + a1*sin(theta1) + a4*cos(theta2 - theta3 + theta4)*sin(theta1) - d5*sin(theta2 - theta3 + theta4)*sin(theta1) + d6*cos(theta1)*cos(theta5) + a2*cos(theta2)*sin(theta1) - a5*cos(theta1)*sin(theta5) + a3*cos(theta2 - theta3)*sin(theta1) + d6*cos(theta2 - theta3 + theta4)*sin(theta1)*sin(theta5) + a5*cos(theta2 - theta3 + theta4)*cos(theta5)*sin(theta1))
 - cos(theta2 - theta3 + theta4)*sin(theta6) - sin(theta2 - theta3 + theta4)*cos(theta5)*cos(theta6)
 sin(theta2 - theta3 + theta4)*cos(theta5)*sin(theta6) - cos(theta2 - theta3 + theta4)*cos(theta6)
 -sin(theta2 - theta3 + theta4)*sin(theta5)];
 
J2 = [

(sin(theta2 - theta3 + theta4)*sin(theta6) - cos(theta2 - theta3 + theta4)*cos(theta5)*cos(theta6))*(d2 - d3 + d4 + d6*cos(theta5) - a5*sin(theta5)) - cos(theta6)*sin(theta5)*(a1 + a4*cos(theta2 - theta3 + theta4) - d5*sin(theta2 - theta3 + theta4) + a2*cos(theta2) + a3*cos(theta2 - theta3) + a5*cos(theta2 - theta3 + theta4)*cos(theta5) + d6*cos(theta2 - theta3 + theta4)*sin(theta5))
(sin(theta2 - theta3 + theta4)*cos(theta6) + cos(theta2 - theta3 + theta4)*cos(theta5)*sin(theta6))*(d2 - d3 + d4 + d6*cos(theta5) - a5*sin(theta5)) + sin(theta5)*sin(theta6)*(a1 + a4*cos(theta2 - theta3 + theta4) - d5*sin(theta2 - theta3 + theta4) + a2*cos(theta2) + a3*cos(theta2 - theta3) + a5*cos(theta2 - theta3 + theta4)*cos(theta5) + d6*cos(theta2 - theta3 + theta4)*sin(theta5))
                                                                      cos(theta5)*(a1 + a4*cos(theta2 - theta3 + theta4) - d5*sin(theta2 - theta3 + theta4) + a2*cos(theta2) + a3*cos(theta2 - theta3) + a5*cos(theta2 - theta3 + theta4)*cos(theta5) + d6*cos(theta2 - theta3 + theta4)*sin(theta5)) - cos(theta2 - theta3 + theta4)*sin(theta5)*(d2 - d3 + d4 + d6*cos(theta5) - a5*sin(theta5))
                                                                                                                                                                                                                                                                                               - cos(theta2 - theta3 + theta4)*sin(theta6) - sin(theta2 - theta3 + theta4)*cos(theta5)*cos(theta6)
                                                                                                                                                                                                                                                                                                 sin(theta2 - theta3 + theta4)*cos(theta5)*sin(theta6) - cos(theta2 - theta3 + theta4)*cos(theta6)
                                                                                                                                                                                                                                                                                                                                                        -sin(theta2 - theta3 + theta4)*sin(theta5)];
                                                                                                                                                                                                                                                                                                                                                    
J3 = [
    
a4*sin(theta6) - d5*cos(theta5)*cos(theta6) + a3*cos(theta4)*sin(theta6) + a5*cos(theta5)*sin(theta6) + d6*sin(theta5)*sin(theta6) + a2*cos(theta3)*cos(theta4)*sin(theta6) + a3*cos(theta5)*cos(theta6)*sin(theta4) + a2*sin(theta3)*sin(theta4)*sin(theta6) + a2*cos(theta3)*cos(theta5)*cos(theta6)*sin(theta4) - a2*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta3)
a4*cos(theta6) + a3*cos(theta4)*cos(theta6) + a5*cos(theta5)*cos(theta6) + d5*cos(theta5)*sin(theta6) + d6*cos(theta6)*sin(theta5) + a2*cos(theta3)*cos(theta4)*cos(theta6) + a2*cos(theta6)*sin(theta3)*sin(theta4) - a3*cos(theta5)*sin(theta4)*sin(theta6) - a2*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta6) + a2*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6)
                                                                                                                                                                                                                                                                                                           -sin(theta5)*(d5 - a3*sin(theta4) + a2*sin(theta3 - theta4))
                                                                                                                                                                                                                                                                                                                                               -cos(theta6)*sin(theta5)
                                                                                                                                                                                                                                                                                                                                                sin(theta5)*sin(theta6)
                                                                                                                                                                                                                                                                                                                                                            cos(theta5)
 ];
J4 = [
     
- (sin(theta4)*sin(theta6) - cos(theta4)*cos(theta5)*cos(theta6))*(d5*cos(theta4) + a4*sin(theta4) + a5*cos(theta5)*sin(theta4) + d6*sin(theta4)*sin(theta5)) - (cos(theta4)*sin(theta6) + cos(theta5)*cos(theta6)*sin(theta4))*(a3 + a4*cos(theta4) - d5*sin(theta4) + a5*cos(theta4)*cos(theta5) + d6*cos(theta4)*sin(theta5))
- (cos(theta6)*sin(theta4) + cos(theta4)*cos(theta5)*sin(theta6))*(d5*cos(theta4) + a4*sin(theta4) + a5*cos(theta5)*sin(theta4) + d6*sin(theta4)*sin(theta5)) - (cos(theta4)*cos(theta6) - cos(theta5)*sin(theta4)*sin(theta6))*(a3 + a4*cos(theta4) - d5*sin(theta4) + a5*cos(theta4)*cos(theta5) + d6*cos(theta4)*sin(theta5))
                                                                                                                                                                                                                                                                                               sin(theta5)*(d5 - a3*sin(theta4))
                                                                                                                                                                                                                                                                                                         cos(theta6)*sin(theta5)
                                                                                                                                                                                                                                                                                                        -sin(theta5)*sin(theta6)
                                                                                                                                                                                                                                                                                                                    -cos(theta5)
 ];
J5 = [
     
sin(theta6)*(a4 + a5*cos(theta5) + d6*sin(theta5)) - d5*cos(theta5)*cos(theta6)
cos(theta6)*(a4 + a5*cos(theta5) + d6*sin(theta5)) + d5*cos(theta5)*sin(theta6)
                                                                -d5*sin(theta5)
                                                       -cos(theta6)*sin(theta5)
                                                        sin(theta5)*sin(theta6)
                                                                    cos(theta5)
 ];
J6 = [
     
 d6*cos(theta6)
-d6*sin(theta6)
            -a5
    sin(theta6)
    cos(theta6)
              0
 ];
J_last = [0
    0
    0
    0
    0
    1];
    
 J = [J2 J3 J4 J5 J6 J_last];
