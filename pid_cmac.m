function [ out, error_i, E ] = pid_cmac( in_target, in, pid, error_i, error_d, T)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
E=in_target-in;
error_i=error_i+E;
if error_i>pi/2 
    error_i=pi/2;
end
if error_i<-pi/2 
    error_i=-pi/2;
end
out=pid(1)*E+pid(2)*error_i+pid(3)*(E-error_d)/T;
end

