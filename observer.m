function etam_n = observer( eta_n,i,N )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
n={0.001*wgn(3,N,1);0.00001*wgn(3,N,0.5)};
noise=cell2mat(n);
etam_n=eta_n+noise(:,i);
end

