function A = vec2skewsym( vec3 )
%UNTITLED5 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
A=zeros(3);
A(1,2)=-vec3(3);A(2,1)=-A(1,2);
A(1,3)=vec3(2);A(3,1)=-A(1,3);
A(2,3)=-vec3(1);A(3,2)=-A(2,3);
end

