R = rotx(pi/2)*roty(pi/2);
trplot(R);
tranimate(R)
R1=rpy2tr(0.1,0.2,0.3)  %四维矩阵   RPY:翻滚俯仰偏航
R2=rpy2r(0.1,0.2,0.3)   %横滚-俯仰-偏航角    三维矩阵
R3=eul2r(0.1,0.2,0.3)  %等价 rotz(0.)*roty(0.2)*rotz(0.3)  欧拉角
q=UnitQuaternion(rpy2tr(1,2,3));%将传递的参数转化为四元数
T=transl(1,0,0);%创建有平移无旋转的相对位姿 
%T表示二维位姿变换，R表示三维位姿变换？
via = [4,1;4,4;5,2;2,5];
q = mstraj(via,[2,1],[],[4,1],0.05,0)  %轨迹

R0=rotz(-1)*roty(-1);
R1=rotz(1)*roty(1); 
rpy0=tr2rpy(R0);
rpy1=tr2rpy(R1);
rpy=mtraj(@tpoly,rpy0,rpy1,50);
tranimate(rpy2tr(rpy));


L(1)=Link([0 0 1 0]);
L(2)=Link([0 0 1 0]);
two_link=SerialLink(L,'name','two link');%创建Link对象  P136
mdl_planar2 %将机器人定义在工作空间
two_link.fkine([0 0])%正向运动学
two_link.plot([0 0])%画机器人图形
two_link.plot([pi/4 -pi/4])%画机器人图形




mdl_puma560 %载入机器人模型
p560
p560.plot(qz)
p560.fkine(qz)
p560.base=transl(0,0,30*0.0254);
p560.plot()


T=p560.fkine(qn);
qi=p560.ikine6s(T)%逆运动学解
qi=p560.ikine6s(T,'ru');%右手位形解

T=p560.fkine(qn);%数值解   
 
T1=transl(0.4,0.2,0)*trotx(pi);
T2=transl(0.4,-0.2,0)*trotx(pi/2);
q1=p560.ikine6s(T1);
q2=p560.ikine6s(T2);
t=[0:0.05:2];
%q=jtraj(q1,q2,t);%相当于具有tpoly插值的mtraj，它对多轴进行了优化   P148
q=p560.jtraj(T1,T2,t)    %关节空间运动
p560.plot(q)
plot(t,q(:,2))

Ts = ctraj(T1,T2,length(t));%笛卡尔空间运动  --直线
plot(t,transl(Ts));
plot(t,tr2rpy(Ts));

T1=transl(0.5,0.3,0.44)*troty(pi/2);%奇异位形运动
T2=transl(0.5,-0.3,0.44)*troty(pi/2);
Ts = ctraj(T1,T2,length(t));%笛卡尔空间运动  --直线
qc=p560.ikine6s(Ts);
qplot(t,qc)
m=p560.maniplty(qc);%可操作性


T=transl(0.4,0.2,0)*trotx(pi);
qr=p560.ikine6s(T,'ru');
ql=p560.ikine6s(T,'lu');
q=jtraj(qr,ql,t);
p560.plot(q)
qplot(t,q)

%%对DH参数进行调整
DHtable=[1 98.5 0 0;
         2 121.5 0 -pi/2;
         3 0 408 pi;
         4 0 376 pi;
         5 102.5 0 -pi/2;
         6 94 0 pi/2];
q1 =[0,0,0,0,0,0]; %pi*rand(1,6); 
q0=[pi,-pi/2,0,-pi/2,0,0];%零位时关节角度
q=q1+q0;

[H_t2o,H_i]=aubo_fkin(DHtable,q);%auboi5正运动学求解
%%
T1=transl(0.4,0.2,0)*trotx(pi);
T2=transl(0.4,-0.2,0)*trotx(pi);








     