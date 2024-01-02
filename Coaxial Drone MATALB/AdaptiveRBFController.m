function [U, InnerXcmd, deltaFM] =AdaptiveRBFController(xCmd, x, controldt, param, controlParam)
%% global define 
global Izdot Ip Iq Ir;
global thetaVelD thetaR;

deltaFM             =   zeros(2,1);
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
dCmd                =   xCmd(3);

phiCmd              =   xCmd(7);
thetaCmd            =   xCmd(8);
psiCmd              =   xCmd(9);

%% Altitude control with RBF Adaptive control
% Outer
Fz_trim             =   -(m*g) / (cos(theta) * cos(phi));
errorD              =   (dCmd - pos(3));
velDCmd             =   K_D * errorD;

% RBF Adaptive
deltaFM(1)          =   AdaptiveLaw_z(velDCmd, velNED(3), param, controldt);

% Inner 
errorVelD           =   velDCmd - velNED(3);
Izdot               =   Izdot + Igain_velD * (errorVelD) * controldt;
FzControl           =   (m / (cos(theta)*cos(phi))) * (K_velD * (errorVelD) + 0*Izdot) + Fz_trim - deltaFM(1);
FzControl           =   max(min(FzControl,0.0), -thrustMax *2);

%% Atti & Rate control(Yaw) with RBF Adaptive control
% Outer
R_trim              =   -q*tan(phi);
errorPsi            =   psiCmd - psi;
rCmd                =   (K_yaw * cos(theta)/cos(phi)) * errorPsi + R_trim;

% RBF Adaptive
deltaFM(2)          =   AdaptiveLaw_yaw(rCmd, r, param, controldt);

% Inner
N_trim              =   (Iy-Ix)*q*p;
errorR              =   rCmd - r;
Ir                  =   Ir + Igain_r * errorR * controldt;
NControl            =   Iz * ((K_r * errorR) + 0*Ir) + N_trim - deltaFM(2);

%% Atti & Rate control(Roll, Pitch)
% Atti control
P_trim              =   -(sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r);
Q_trim              =   r*sin(phi);

errorPhi            =   phiCmd - phi;
errorTheta          =   thetaCmd - theta;

pCmd                =   K_roll * errorPhi + P_trim;
qCmd                =   (K_pitch / cos(phi)) * errorTheta + Q_trim;

% Rate control
L_trim              =   (Iz-Iy)*q*r;
M_trim              =   (Ix-Iz)*p*r;

errorP              =   pCmd - p;
errorQ              =   qCmd - q;

Ip                  =   Ip + Igain_p * errorP * controldt;
Iq                  =   Iq + Igain_q * errorQ * controldt;

LControl            =   Ix * ((K_p * errorP) + Ip) + L_trim;
MControl            =   Iy * ((K_q * errorQ) + Iq) + M_trim;

%% Allocation
Vs                  =   sqrt(-FzControl / (2*1.225*pi*rprop^2));
Q                   =   0.5*1.225*(2*Vs)^2;
k1                  =   cL * finA * Q * dz;
k2                  =   cL * finA * Q * d;
kQ                  =   cM / cT;
                
CA                  =   [-1,  -1,  0,  0,  0,  0;
                          0,   0, -k1, 0,  k1, 0;
                          0,   0,  0,  k1, 0, -k1;
                         -kQ, kQ, 0,  0,  0,  0];

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
function deltaFz = AdaptiveLaw_z(xCmd, x, param, controldt)
%% parameter define 
global thetaVelD 
m                   =   param(1);

%% RBF basis parameters
c               =   0.02;
d               =   2;
sigma           =   1;



for i = 1:11
    norm_z      =   sqrt((x - c*(i-1) + d)^2);
    rbf(i,1)    =   exp(-sigma^2 * norm_z);
end

%% AdaptiveLaw
A               =   1;
Q               =   1;

P               =   lyap(A,Q);
B               =   1/m;
e               =   x - xCmd;

Gamma           = 100 * eye(11,11);
thetaDot        = Gamma * rbf * e' * P * B; 
thetaVelD       = thetaVelD + thetaDot * controldt;

deltaFz         = -thetaVelD' * rbf;
end

function deltaN = AdaptiveLaw_yaw(xCmd, x, param, controldt)
%% parameter define 
global thetaR
Iz                  =   param(8);

%% RBF basis parameters
c               =   0.02; % 0.2
d               =   2; % 2
sigma           =   1;

for i = 1:11
    norm_z      =   sqrt((x - c*(i-1) + d)^2);
    rbf(i,1)    =   exp(-sigma^2 * norm_z);
end

%% AdaptiveLaw
A               =   1;
Q               =   1;

P               =   lyap(A,Q);
B               =   1/Iz;
e               =   x - xCmd;

Gamma           = 0.02 * eye(11,11); % 0.01
thetaDot        = Gamma * rbf * e' * P * B;
thetaR          = thetaR + thetaDot * controldt;

deltaN          = -thetaR' * rbf;  

end

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

function W = pqrDCM(euler)
    roll        =   euler(1);
    pitch       =   euler(2);
    yaw         =   euler(3);

    W(1,1)      =   1;
    W(1,2)      =   sin(roll)*tan(pitch);
    W(1,3)      =   cos(roll)*tan(pitch);
    W(2,1)      =   0;
    W(2,2)      =   cos(roll);
    W(2,3)      =   -sin(roll);
    W(3,1)      =   0;
    W(3,2)      =   sin(roll)/cos(pitch);
    W(3,3)      =   cos(roll)/cos(pitch);
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