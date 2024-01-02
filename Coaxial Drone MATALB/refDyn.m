function [xRefDot, xRefcmd] = refDyn(xref, xcmd, controlParam, dt)

%% global I_term define 
global IzdotRef IpRef IqRef IrRef;

%% gain define 
K_D                 =   controlParam(1);
K_velD              =   controlParam(2);
K_roll              =   controlParam(3);
K_pitch             =   controlParam(4);
K_yaw               =   controlParam(5);
K_p                 =   controlParam(6);
K_q                 =   controlParam(7);
K_r                 =   controlParam(8);
Igain_velD          =   controlParam(9);
Igain_p             =   controlParam(10);
Igain_q             =   controlParam(11);
Igain_r             =   controlParam(12);  

%% state define
D                   =   xref(3);
u                   =   xref(4);
v                   =   xref(5);
w                   =   xref(6);
phi                 =   xref(7);
theta               =   xref(8);
psi                 =   xref(9);
p                   =   xref(10);
q                   =   xref(11);
r                   =   xref(12);

euler               =   [phi; theta; psi];
R                   =   EulerDCM(euler);
velNED              =   R * [u;v;w];

%% command define 
dCmd                =   xcmd(3);
phiCmd              =   xcmd(7);
thetaCmd            =   xcmd(8);
psiCmd              =   xcmd(9);

%% Attitude & Altitude controller 
dDotCmd             =   K_D * (dCmd - D);
pCmd                =   K_roll * (phiCmd - phi);
qCmd                =   K_pitch * (thetaCmd - theta);
rCmd                =   K_yaw * (psiCmd - psi);

IzdotRef            =   IzdotRef + Igain_velD * (dDotCmd - velNED(3)) * dt;    
IpRef               =   IpRef + Igain_p * (pCmd - p) * dt;
IqRef               =   IqRef + Igain_q * (qCmd - q) * dt;
IrRef               =   IrRef + Igain_r * (rCmd - r) * dt;

az                  =   K_velD * (dDotCmd - velNED(3)) + IzdotRef;
pdot                =   K_p * (pCmd - p) + IpRef;
qdot                =   K_q * (qCmd - q) + IqRef;
rdot                =   K_r * (rCmd - r) + IrRef;

xRefDot             =   [0; 0; dDotCmd; 0; 0; az; pCmd; qCmd; rCmd; pdot; qdot; rdot];
xRefcmd(6,1)        =   dDotCmd; 
xRefcmd(10,1)       =   pCmd; 
xRefcmd(11,1)       =   qCmd; 
xRefcmd(12,1)       =   rCmd;
end

%% Functions
function R = EulerDCM(euler)
    roll        =   euler(1);
    pitch       =   euler(2);
    yaw         =   euler(3);
    
    R(1,1)      =   cos(yaw) * cos(pitch);
    R(1,2)      =   -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll); 
    R(1,3)      =   sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
    R(2,1)      =   sin(yaw)*cos(pitch);
    R(2,2)      =   cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
    R(2,3)      =   -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
    R(3,1)      =   -sin(pitch);
    R(3,2)      =   cos(pitch)*sin(roll);
    R(3,3)      =   cos(pitch)*cos(roll);
end