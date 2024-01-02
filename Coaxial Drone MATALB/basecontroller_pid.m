function [U, InnerXcmd] = basecontroller_pid(xcmd, x, dt, param, controlParam)

%% global define 
global Izdot Ip Iq Ir;

%% parameter define 
m                   =   param(1);
g                   =   param(2);
dz                  =   param(3);
d                   =   param(4);
finA                =   param(5);
Ix                  =   param(6);
Iy                  =   param(7);
Iz                  =   param(8);
thrustMax           =   param(9);
cT                  =   param(10);
cM                  =   param(11);
cL                  =   param(12);
lprop               =   param(13);
rprop               =   param(14);
finMax              =   param(15);

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
pos                 =   x(1:3);                                               %N, E, D
velB                =   x(4:6);                                               %u, v, w
euler               =   x(7:9);                                               %phi, the, psi
rateB               =   x(10:12);                                             %p, q, r   

%% body parameter define 
u                   =   velB(1);
v                   =   velB(2);
w                   =   velB(3);
phi                 =   euler(1);
theta               =   euler(2);
psi                 =   euler(3);
p                   =   rateB(1);
q                   =   rateB(2);
r                   =   rateB(3);

%% frame rotation matrix define 
R                   =   EulerDCM(euler);

velNED              =   R * velB;

%% command define 
dCmd                =   xcmd(3);

phiCmd              =   xcmd(7);
thetaCmd            =   xcmd(8);
psiCmd              =   xcmd(9);

%% Altitude control
Fz_trim             =   -(m*g) / (cos(theta) * cos(phi));
errorD              =   (dCmd - pos(3));
velDCmd             =   K_D * errorD;

errorVelD           =   velDCmd - velNED(3);
Izdot               =   Izdot + Igain_velD * (errorVelD) * dt;
FzControl           =   (m / (cos(theta)*cos(phi))) * (K_velD * (errorVelD) + Izdot) + Fz_trim;
FzControl           =   max(min(FzControl,0.0), -thrustMax *2);
%% Atti control 
P_trim              =   -(sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r);
Q_trim              =   r*sin(phi);
R_trim              =   -q*tan(phi);

errorPhi            =   phiCmd - phi;
errorTheta          =   thetaCmd - theta;
errorPsi            =   psiCmd - psi;

pCmd                =   K_roll * errorPhi + P_trim;
qCmd                =   (K_pitch / cos(phi)) * errorTheta + Q_trim;
rCmd                =   (K_yaw * cos(theta)/cos(phi)) * errorPsi + R_trim;

%% Rate control
L_trim              =   (Iz-Iy)*q*r;
M_trim              =   (Ix-Iz)*p*r;
N_trim              =   (Iy-Ix)*q*p;

errorP              =   pCmd - p;
errorQ              =   qCmd - q;
errorR              =   rCmd - r;

Ip                  =   Ip + Igain_p * errorP * dt;
Iq                  =   Iq + Igain_q * errorQ * dt;
Ir                  =   Ir + Igain_r * errorR * dt;

LControl            =   Ix * ((K_p * errorP) + Ip) + L_trim;
MControl            =   Iy * ((K_q * errorQ) + Iq) + M_trim;
NControl            =   Iz * ((K_r * errorR) + Ir) + N_trim;

%% allocation 
Vs                  =   sqrt(-FzControl / (2*1.225*pi*rprop^2));
Q                   =   0.5*1.225*(2*Vs)^2;
k1                  =   cL * finA * Q * dz;
k2                  =   cL * finA * Q * d;
kQ                  =   cM / cT;
                
CA                  =   [-1,  -1,  0,  0,  0,  0;
                          0,   0, -k1, 0,  k1, 0;
                          0,   0,  0,  k1, 0, -k1;
                         -kQ, kQ, 0,  0,  0,  0];

% CA                  =   [-1,  -1,  0,  0,  0,  0;
%                           0,   0, -k1, 0,  k1, 0;
%                           0,   0,  0,  k1, 0, -k1;
%                          -kQ, kQ, k2,  k2,  k2,  k2];
         
U                   =   pinv(CA) * [FzControl; LControl; MControl; NControl];

%Constraint Thrust
U(1)                =   min(max(U(1), 0.1), thrustMax);
U(2)                =   min(max(U(2), 0.1), thrustMax);

%Constraint fin angle
U(3)                =   finSaturation(finMax, U(3));
U(4)                =   finSaturation(finMax, U(4));
U(5)                =   finSaturation(finMax, U(5));
U(6)                =   finSaturation(finMax, U(6));

InnerXcmd(6,1)      =   (1 / (cos(theta)*cos(phi))) * velDCmd; 
InnerXcmd(10,1)     =   pCmd; 
InnerXcmd(11,1)     =   qCmd; 
InnerXcmd(12,1)     =   rCmd; 

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

function angle = finSaturation(finMax, alpha)
    
    if (alpha >= finMax)
        angle = finMax;

    elseif (alpha <= -finMax)
        angle = -finMax;
    
    else
        angle = alpha;
    
    end

end