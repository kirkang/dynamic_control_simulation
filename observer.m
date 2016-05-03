function etam_n = observer( eta_n,i,N )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
n={0.001*wgn(3,N,1);0.00001*wgn(3,N,0.5)};
noise=cell2mat(n);
etam_n=eta_n+noise(:,i);
end

