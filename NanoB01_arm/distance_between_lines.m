function [dis,P1,Q1,P2,Q2] = distance_between_lines(A,B,C,D)

%求两线段之间的最短距离
%输入为横向量！！！！！
%[dis,P1,Q1,P2,Q2]=segmentDistance(A,B,C,D)
%A,B为第一条线段的首末端点 C,D为第二条线段的首末短线
%dis为最短距离 P2 Q2为最短距离的首末端点
%P1,Q1为直线AB、CD的最短距离首末端点
%P2,Q2为线段AB、CD的最短距离首末端点

syms s t
%s取0~1时P点在线段AB上滑动，s>1 P点在B端点外，s<1 P点在A端点外
P=A+s.*(B-A);
%t取0~1时Q点在线段CD上滑动，t>1 Q点在D端点外，s<1 P点在C端点外
Q=C+t.*(D-C);
%求P点到Q点的距离的平方
Fst=sum((P-Q).^2);
%求偏导
eqns(1)=(diff(Fst,s)==0);
eqns(2)=(diff(Fst,t)==0);
%解二元一次方程
[s,t]=solve(eqns,s,t);
P1=subs(P);
Q1=subs(Q);
%如果两线段最短距离不在线段上 进行判断
if s>1
    s=1;
    [dis,P2,Q2]=pointSegmentDistance(C,D,subs(P));
elseif s<0
    s=0;
    [dis,P2,Q2]=pointSegmentDistance(C,D,subs(P));
elseif t>1
    t=1;
    [dis,P2,Q2]=pointSegmentDistance(A,B,subs(Q));
elseif t<0
    t=0;
    [dis,P2,Q2]=pointSegmentDistance(A,B,subs(Q));
else
    dis=norm(subs(P)-subs(Q));
    P2=P1;
    Q2=Q1;
end
end

function [dis,P2,Q2]=pointSegmentDistance(A,B,C)
%求空间一点到一线段的最短距离 
%[dis,P2,Q2]=pointSegmentDistance(A,B,C)
%A B为线段首末端点，C为空间一点
%dis为最短距离 P2 Q2为最短距离的首末端点

syms s
%s取0~1时P点在线段AB上滑动，s>1 P点在B端点外，s<1 P点在A端点外
P=A+s.*(B-A);
Fs=sum((P-C).^2);
eqns=(diff(Fs,s)==0);
s=solve(eqns,s);
if s>1
    s=1;
    dis=norm(subs(P)-C);
elseif s<0
    s=0;
    dis=norm(subs(P)-C);
else
    dis=norm(subs(P)-C);
end
P2=subs(P);
Q2=C;
end

