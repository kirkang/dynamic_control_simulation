clear all;
% close all;
%% 仿真参数设置
N=10000-1;
T=0.001;

%% 配重初始化
rdeltam=[0.02 0.01 0]';
deltam=0;
%% 推力配置,推力数组Thrust初始化
%ri 第i个推进器方向单位列向量
%pi 第i个推进器位置向量
%Ti 第i个推进器推力，标量
%A  tau=A*[T1,T2,...,Tn]'
%Arot
%Atrans
Thrust=[0,0,0]';
r1=[0,0,1]';p1=[0,-0.05,0]';%右，向上
r2=[0,0,1]';p2=[0,0.05,0]';%左，向上
r3=[0,0,1]';p3=[0.1,0,0]';%前，向上
r4=[0,0,1]';p4=[-0.1,0,0]';%后，向上
r5=[1,0,0]';p5=[0,-0.05,0]';
r6=[1,0,0]';p6=[0,0.05,0]';
r7=[0,1,0]';p7=[-0.05,0,0]';
r8=[0,1,0]';p8=[0.05,0,0]';
rraw=[r1,r2,r3,r4,r5,r6,r7,r8];
praw=[p1,p2,p3,p4,p5,p6,p7,p8];
c=cell(2,8);
for i=1:8
    if length(nonzeros(rraw(:,i)))==0 && length(nonzeros(praw(:,i)))==0
        n=i-1;
        break
    end
end
index=[
    1 
    2 
    3 
    4 
    %5 
    %6 
    %7 
    %8
    ];
for i=1:length(index)
%     c(1,i)=mat2cell(rraw(:,index(i)));
%     c(2,i)=mat2cell(cross(praw(:,index(i)),rraw(:,index(i))));
    c(1,i)={rraw(:,index(i))};
    c(2,i)={cross(praw(:,index(i)),rraw(:,index(i)))};
end
A=cell2mat(c);
B=mat2cell(A,[3 3]);
Atrans=cell2mat(B(1));
Arot=cell2mat(B(2));
% Arotinv=Arot'*inv(Arot*Arot');
% Atransinv=Atrans'*inv(Atrans*Atrans');
Arotinv=pinv(Arot);
Atransinv=pinv(Atrans);
Tt=zeros(length(index),N);%推力序列

%% PID参数
rp1_2=0.15;rp3_4=0.3;
ri1_2=0.0005;ri3_4=0.0005;
rd1_2=0.015;rd3_4=0.025;
tp1_2=1.2;tp3_4=1.2;
ti1_2=0.0015;ti3_4=0.0015;
td1_2=0.3;td3_4=0.3;
pidrot=[[rp1_2 ri1_2 rd1_2];%1
        [rp1_2 ri1_2 rd1_2];%2
        [rp3_4 ri3_4 rd3_4];%3
        [rp3_4 ri3_4 rd3_4];%4
        [2 0.002 0.08];%5
        [2 0.002 0.08];%6
        [2 0.002 0.08];%7
        [2 0.002 0.08];%8
       ];%[5 0.000001 0.2];%[0.15 0.0001 0.01];
pidtrans=[[tp1_2 ti1_2 td1_2];%1
          [tp1_2 ti1_2 td1_2];%2
          [tp3_4 ti3_4 td3_4];%3
          [tp3_4 ti3_4 td3_4];%4
          [5 0.005 0.5];%5
          [5 0.005 0.5];%6
          [5 0.005 0.5];%7
          [5 0.005 0.5];%8
         ];%[5 0.00001 1.5];%[1 0.0001 0.5];
%% 初始化网络
max_angle=10;
min_angle=-10;
max_trans=5;
min_trans=-5;
max_min=[[max_trans min_trans]' [max_angle min_angle]' [max_angle min_angle]' ...
%             [max_trans min_trans]' [max_angle min_angle]' [max_angle min_angle]' ...
%             [max_trans min_trans]' [max_angle min_angle]' [max_angle min_angle]' ...
            ];%z,roll,pitch
resolution=0.05;
R=2*max_angle/resolution;
C=ceil(R/99);
a=CMACcontroller(max_min , C, 3);
Asimple(1,:)=A(3,:);
Asimple(2,:)=A(4,:);
Asimple(3,:)=A(5,:);%用于网络的简化版A阵
Asimpleinv=pinv(Asimple);

%% 状态初始化
eta=zeros(6,N);
etadot=zeros(6,N);
etam=zeros(6,N);

for i=1:N+1
eta_target(:,i)=[0, 0, 0, (0)*pi/180, (0)*pi/180, 0]';
% eta_target(:,i)=[0, 0, 0, (5)*pi/180*sin(pi*i/1000), (5)*pi/180*cos(pi*i/1000), 0]';
% if i>100
%     eta_target(:,i)=[0, 0, 0, (5)*pi/180, (5)*pi/180, 0]';
% end
% if i>3000
%    eta_target(:,i)=[0, 0, 0, (0)*pi/180, (0)*pi/180, 0]'; 
% end
end
eta(:,1)=[0,0,0,0,0,0]';
eta(:,2)=eta(:,1);
tau=[0 0 0 0 0 0]';

Erot=zeros(length(index),1);%=[0 0 0 0 0 0 0 0]';
Etrans=zeros(length(index),1);%=[0 0 0 0 0 0 0 0]';
Eroti=zeros(length(index),1);%=[0 0 0 0 0 0 0 0]';
Etransi=zeros(length(index),1);%=[0 0 0 0 0 0 0 0]';
%% 产生白噪声
n={0.025*wgn(3,N,1);0.05*wgn(3,N,1)};
noise=cell2mat(n);
%% 仿真循环
for i=3:N-2
    if i>1000 
        deltam=0.05;
    end
    if i>6000 
        deltam=0;
    end
    [ eta(:,i+1), etadot(:,i+1) ]=model(tau, eta(:,i), etadot(:,i), T, deltam, rdeltam);
    %etam(:,i+1)=observer(eta(:,i+1),i,N);
    etam(:,i+1)=eta(:,i+1)+noise(:,i)/100;
    if i==1000
        ;
    end
    [tau, Erot, Etrans, Eroti, Etransi]=controller( eta_target(:,i+1), eta(:,i+1), Erot, Etrans, Eroti, Etransi, A, pidrot, pidtrans, T);
    
% %     [a, FM(:,i)] = CMACrecaller(a, [eta_target(3, i+1) eta_target(4, i+1) eta_target(5, i+1) eta_target(3, i) eta_target(4, i) eta_target(5, i) eta_target(3, i-1) eta_target(4, i-1) eta_target(5, i-1)], [0 0 0], false);
%     [a, FM(:,i)] = CMACrecaller(a, [eta_target(3, i+2)-2*eta_target(3, i+1)+eta_target(3, i) eta_target(4, i+2)-2*eta_target(4, i+1)+eta_target(4, i) eta_target(5, i+2)-2*eta_target(5, i+1)+eta_target(5, i)], [0 0 0], false);
% %     [a, FM(:,i)] = CMACrecaller(a, [0 ...
% %                                     0 ...
% %                                     0 ...
% %                                     eta_target(3, i+1)-eta(3, i) ...
% %                                     eta_target(4, i+1)-eta(4, i) ...
% %                                     eta_target(5, i+1)-eta(5, i) ...
% %                                     eta_target(3, i)-eta(3, i-1) ...
% %                                     eta_target(4, i)-eta(4, i-1) ...
% %                                     eta_target(5, i)-eta(5, i-1)], [0 0 0], false);
% 
%     tau=tau+[0 0 FM(:,i)' 0]';    
%     motorforce=pinv(A)*tau;
%     motorforce
%       %% 电机限幅
%       for ii=1:length(motorforce)
%         if motorforce(ii)>0.2 
%             motorforce(ii)=0.2;end
%         if motorforce(ii)<-0.2 
%             motorforce(ii)=-0.2;end
%       
%       tau=A*motorforce;
%    end
    TAU(:,i)=tau;
% %     [a, z] = CMACrecaller(a, [eta(3, i) eta(4, i) eta(5, i) eta(3, i-1) eta(4, i-1) eta(5, i-1) eta(3, i-2) eta(4, i-2) eta(5, i-2)], TAU(3:5,i-2), true);
%       [a, z] = CMACrecaller(a, [eta(3, i)-2*eta(3, i-1)+eta(3, i-2) eta(4, i)-2*eta(4, i-1)+eta(4, i-2) eta(5, i)-2*eta(5, i-1)+eta(5, i-2)], TAU(3:5,i-2), true);
% %     [a, z] = CMACrecaller(a, [eta_target(3, i+1)-eta(3, i) ...
% %                               eta_target(4, i+1)-eta(4, i) ...
% %                               eta_target(5, i+1)-eta(5, i) ...
% %                               eta_target(3, i)-eta(3, i-1) ...
% %                               eta_target(4, i)-eta(4, i-1) ...
% %                               eta_target(5, i)-eta(5, i-1) ...
% %                               eta_target(3, i-1)-eta(3, i-2) ...
% %                               eta_target(4, i-1)-eta(4, i-2) ...
% %                               eta_target(5, i-1)-eta(5, i-2)], TAU(3:5,i-2), true);
%     Etau(:,i-2)=TAU(:,i-2)-[0 0 z 0]';
%     TAU(:,i-2)-[0 0 z 0]'
    
    i
end



%% 绘图
% figure
% plot3(eta(1,1:N-2),eta(2,1:N-2),-eta(3,1:N-2));
% grid on

figure
subplot(3,1,1);
plot(eta(1,1:N-2));
grid on
subplot(3,1,2);
plot(eta(2,1:N-2));
grid on
subplot(3,1,3);
plot(-eta(3,1:N-2));
grid on

figure
subplot(3,1,1);
plot(eta(4,1:N-2)*180/pi);
hold on
plot(eta_target(4,1:N-2)*180/pi,'r');
grid on
subplot(3,1,2);
plot(eta(5,1:N-2)*180/pi);
hold on
plot(eta_target(5,1:N-2)*180/pi,'r');
grid on
subplot(3,1,3);
plot(eta(6,1:N-2)*180/pi);
hold on
plot(eta_target(6,1:N-2)*180/pi,'r');
grid on

% figure
% subplot(3,1,1);
% plot(TAU(4,1:N-2));
% grid on
% subplot(3,1,2);
% plot(FM(1,1:N-2));
% grid on
% subplot(3,1,3);
% plot(TAU(4,1:N-2)-FM(1,1:N-2));
% grid on
% 
% figure
% subplot(3,1,1);
% plot(Etau(3,1:N-5));
% grid on
% subplot(3,1,2);
% plot(Etau(4,1:N-5));
% grid on
% subplot(3,1,3);
% plot(Etau(5,1:N-5));
% grid on