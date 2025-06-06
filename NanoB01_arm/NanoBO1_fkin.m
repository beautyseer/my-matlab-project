function [H, H_i] = NanoBO1_fkin(DHtable,q)
% H 为输出，末端到基座的 4X4 齐次变换矩阵；
% H_i 为输出，相邻连杆的 4X4 齐次变换矩阵，由于有 n 个连杆，因此 H_i 应该为包含 n 个元素的 cell；
% DHtable 为输入，是一个 nX4 的矩阵，满足 DH 改进约定的 DH 表
% q 为输入，是一个 1xN 的向量，表示关节坐标
 
%%%！！！！注：机械臂零位的角度应先加到q中,而DH表不含有零位角度！！！！！！！%%
 
%%相邻连杆的 4X4 齐次变换矩阵%%
    first=1;
    last=size(DHtable,1);%%计算出有多少个连杆
    T1=ones(4,4,6);
 for i=first:last
    theta=q(i);
    d=DHtable(i,2);
    a=DHtable(i,3);
    alpha=DHtable(i,4);
  T1(:,:,i)=[ cos(theta)                 -sin(theta)                 0                    a;
              sin(theta)*cos(alpha)      cos(theta)*cos(alpha)     -sin(alpha)       -d*sin(alpha);
              sin(theta)*sin(alpha)      cos(theta)*sin(alpha)      cos(alpha)       d*cos(alpha);
                0               0                       0           1];
 T(:,:,i)=double(T1(:,:,i));
 end
 %%
 %%计算末端到基座的 4X4 齐次变换矩阵和将相邻连杆的 4X4 齐次变换矩阵放置到元胞数组%%
  H1=eye(4);
  H_i=cell(1,last);  %%%预先分配内存
for i=first:last
    H1=H1* T(:,:,i);
    H_i{i}=T(:,:,i);
end
 H_i;
 H=double(H1);
 
 