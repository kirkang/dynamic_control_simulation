function [tau, Erot, Etrans, Eroti, Etransi]= controller( eta_target, eta_n, Erot1, Etrans1, Eroti, Etransi, A, pidrot, pidtrans, T)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
%% Arot, Atrans, Atransinv, Arotinv
B=mat2cell(A,[3 3]);
Atrans=cell2mat(B(1));
Arot=cell2mat(B(2));
% Arotinv=Arot'*inv(Arot*Arot');
% Atransinv=Atrans'*inv(Atrans*Atrans');
Arotinv=pinv(Arot);
Atransinv=pinv(Atrans);

%% Rcmd
J=RotationM(eta_n);
TEMP=mat2cell(J,[3 3],[3 3]);
Ris=cell2mat(TEMP(1,1));
J=RotationM(eta_target);
TEMP=mat2cell(J,[3 3],[3 3]);
Rgoal=cell2mat(TEMP(1,1));
%Rcmd=inv(inv(Rgoal)*Ris);
Rcmd=Rgoal*inv(Ris);
Ris
inv(Ris)
%% axis r and angle w of rotation, cmdroll/cmdpitch/cmdyaw
flag=true;
w=acos((trace(Rcmd)-1)/2);
if w==0 
    flag=false;
else
    r=[Rcmd(3,2)-Rcmd(2,3),Rcmd(1,3)-Rcmd(3,1),Rcmd(2,1)-Rcmd(1,2)]'/(2*sin(w));
end

%% Erot
if flag
Erot=Arotinv*(w*r);
else Erot=Erot1;
end
%% Etrans
Etrans=Atransinv*[0 0 eta_target(3)-eta_n(3)]';
%% output
Eroti=Eroti+Erot;
Etransi=Etransi+Etrans;
for i=1:length(Eroti)
if Eroti(i)>10 Eroti(i)=0;end%清积分饱和
if Etransi(i)>100 Etransi(i)=0;end
end
% Thrustrot=pidrot(1)*Erot+pidrot(2)*(Eroti+Erot)+pidrot(3)*(Erot-Erot1)/T;
% Thrusttrans=pidtrans(1)*Etrans+pidtrans(2)*(Etransi+Etrans)+pidtrans(3)*(Etrans-Etrans1)/T;
Thrustrot=zeros(length(Erot),1);
Thrusttrans=zeros(length(Erot),1);
for i=1:length(Erot)%pid计算
Thrustrot(i)=pidrot(i,1)*Erot(i)+pidrot(i,2)*(Eroti(i)+Erot(i))+pidrot(i,3)*(Erot(i)-Erot1(i))/T;
Thrusttrans(i)=pidtrans(i,1)*Etrans(i)+pidtrans(i,2)*(Etransi(i)+Etrans(i))+pidtrans(i,3)*(Etrans(i)-Etrans1(i))/T;
end
Ttoatl=Thrustrot+Thrusttrans;
for i=1:length(Ttoatl)
if Ttoatl(i)>0.2 
    Ttoatl(i)=0.2;end
if Ttoatl(i)<-0.2 
    Ttoatl(i)=-0.2;end
end
tau=A*(Ttoatl);
end

