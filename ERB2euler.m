function vec = ERB2euler( R )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
pitch=asin(-R(3,1));
roll=asin(R(3,2)/cos(pitch));
yaw=asin(R(2,1)/cos(pitch));
vec=[roll pitch yaw]';
end

