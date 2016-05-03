function [ eta_n1, etadot_n1 ] = model( tau, eta_n, etadot_n, T, deltam, r2)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

%%********************%%
%eta[X, Y, Z, roll, pitch, yaw], external坐标系下系统状态
%niu[u, v, w, wx, wy, wz], body坐标系下速度角速度向量
%Mrb惯性矩阵， Ma附加质量
%Crb科氏力矩阵，Ca流体科氏力矩阵
%D阻尼矩阵，D=Dp+Ds+Dw+Dm
%tau[Fx, Fy, Fz, Mx, My, Mz], Body FORCE向量
%deltam 增加质量
%r2 质量增加位置
%r1 rg 机体的浮心和整体重心位置
%%********************%%

mbody=0.1;%kg
%deltam=0;
m=mbody+deltam;
%r2=[0 0 0]';
rg=deltam/(deltam+m)*r2;
g=10;
B=mbody*g;
r1=[0 0 -0.01]';

Ix=0.001;
Iy=0.002;
Iz=0.004;
% TEMP(1,1)=mat2cell(diag([m m m]));
% TEMP(1,2)=mat2cell(-m*vec2skewsym(rg));
% TEMP(2,1)=mat2cell(m*vec2skewsym(rg));
% TEMP(2,2)=mat2cell(diag([Ix Iy Iz])+deltam*vec2skewsym(rg)*vec2skewsym(rg));
TEMP(1,1)={diag([m m m])};
TEMP(1,2)={-m*vec2skewsym(rg)};
TEMP(2,1)={m*vec2skewsym(rg)};
TEMP(2,2)={diag([Ix Iy Iz])+deltam*vec2skewsym(rg)*vec2skewsym(rg)};
Mrb=cell2mat(TEMP);%Mrb惯性矩阵
%Mrb=diag([m m m Ix Iy Iz]);%Mrb惯性矩阵
Ma=zeros(6);
Ma(1,1)=0.0005;Ma(2,2)=0.0005;Ma(3,3)=0.0005;Ma(4,4)=0.0002;Ma(5,5)=0.0001;Ma(6,6)=0.0002;
Ma(5,1)=0.0005;Ma(1,5)=Ma(5,1);
Ma(4,2)=0.0005;Ma(2,4)=Ma(4,2);%Ma附加质量
M=Mrb+Ma;%惯量阵

J=RotationM(eta_n);
niu=inv(J)*etadot_n;
TEMP=mat2cell(J,[3 3],[3 3]);
R=cell2mat(TEMP(1,1));

TEMP=mat2cell(Mrb,[3 3],[3 3]);
M11=cell2mat(TEMP(1,1));
M12=cell2mat(TEMP(1,2));
M21=cell2mat(TEMP(2,1));
M22=cell2mat(TEMP(2,2));
TEMP=mat2cell(niu,[3 3]);
niu1=cell2mat(TEMP(1));
niu2=cell2mat(TEMP(2));
Cniu11=zeros(3);
Cniu12=-vec2skewsym(M11*niu1+M12*niu2);
Cniu21=-vec2skewsym(M11*niu1+M12*niu2);
Cniu22=-vec2skewsym(M21*niu1+M22*niu2);
Crb=cell2mat({Cniu11,Cniu12; Cniu21, Cniu22});%Crb科氏力矩阵
TEMP=mat2cell(Ma,[3 3],[3 3]);
A11=cell2mat(TEMP(1,1));
A12=cell2mat(TEMP(1,2));
A21=cell2mat(TEMP(2,1));
A22=cell2mat(TEMP(2,2));
TEMP=mat2cell(niu,[3 3]);
niu1=cell2mat(TEMP(1));
niu2=cell2mat(TEMP(2));
Caniu11=zeros(3);
Caniu12=-vec2skewsym(A11*niu1+A12*niu2);
Caniu21=-vec2skewsym(A11*niu1+A12*niu2);
Caniu22=-vec2skewsym(A21*niu1+A22*niu2);
Ca=cell2mat({Caniu11,Caniu12; Caniu21, Caniu22});%Ca流体科氏力矩阵
C=Crb+Ca;%科氏力阵

Xu=0.01;Yv=0.01;Zw=0.01;Kp=0.001;Mq=0.002;Nr=0.003;
Xuu=0;Yvv=0;Zww=0;Kpp=0;Mqq=0;Nrr=0;
D=diag([Xu Yv Zw Kp Mq Nr])+diag([Xuu*abs(niu(1)) Yvv*abs(niu(2)) Zww*abs(niu(3)) Kpp*abs((niu(4))) Mqq*abs(niu(5)) Nrr*abs(niu(6))]);%阻尼阵

l=0.05;
%Mbuoyancy=inv(R)*(cross(R*[0 0 l]', [0 0 B]'));
RestoringFORCE=inv(R)*[0, 0, m*g-B]';
Mbuoyancy=cross(r1,inv(R)*[0 0 -B]');
Mdeltam=cross(r2, inv(R)*[0 0 deltam*g]');
G=[RestoringFORCE(1) RestoringFORCE(2) RestoringFORCE(3) Mbuoyancy(1)+Mdeltam(1) Mbuoyancy(2)+Mdeltam(2) Mbuoyancy(3)+Mdeltam(3)]';
%G=[0 0 0 Mbuoyancy(1) Mbuoyancy(2) Mbuoyancy(3)]';

niudot=inv(M)*(tau+G-C*niu-D*niu);
niun1=niu+niudot*T;
etadot_n1=J*niun1;
eta_n1=eta_n+etadot_n*T;
end

