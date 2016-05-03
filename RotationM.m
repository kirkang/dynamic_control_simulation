function J = RotationM( eta )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
%eta[X, Y, Z, roll, pitch, yaw], external坐标系下系统状态
%niu[u, v, w, wx, wy, wz], body坐标系下速度角速度向量
roll=eta(4);
pitch=eta(5);
yaw=eta(6);
J=[
    [cos(yaw)*cos(pitch),cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),sin(yaw)*sin(roll)+cos(yaw)*cos(roll)*sin(pitch)],[0,0,0];
    [sin(yaw)*cos(pitch),cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw),sin(pitch)*sin(yaw)*cos(roll)-cos(yaw)*sin(roll)],[0,0,0];
    [-sin(pitch),cos(pitch)*sin(roll),cos(pitch)*cos(roll)],[0,0,0];
    [0,0,0],[1,sin(roll)*tan(pitch),cos(roll)*tan(pitch)];
    [0,0,0],[0,cos(roll),-sin(roll)];
    [0,0,0],[0,sin(roll)/cos(pitch),cos(roll)/cos(pitch)]
  ];
end

