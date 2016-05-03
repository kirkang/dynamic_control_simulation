function vec = ERB2euler( R )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
pitch=asin(-R(3,1));
roll=asin(R(3,2)/cos(pitch));
yaw=asin(R(2,1)/cos(pitch));
vec=[roll pitch yaw]';
end

