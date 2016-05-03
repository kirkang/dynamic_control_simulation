function J = RotationM( eta )
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
%eta[X, Y, Z, roll, pitch, yaw], external����ϵ��ϵͳ״̬
%niu[u, v, w, wx, wy, wz], body����ϵ���ٶȽ��ٶ�����
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

