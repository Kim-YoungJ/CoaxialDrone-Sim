function [xDot] = Dyn6Dof(x, Fb, Mb, param)

%% parameter define 
m                   = param(1);
g                   = param(2);
dz                  = param(3);
d                   = param(4);
finA                = param(5);
Ix                  = param(6);
Iy                  = param(7);
Iz                  = param(8);
thrustMax           = param(9);
cT                  = param(10);
cM                  = param(11);
cL                  = param(12);
lprop               = param(13);
rprop               = param(14);
finMax              = param(15);

%% state define 
pos                 = x(1:3);                                               %N, E, D
velB                = x(4:6);                                               %u, v, w
euler               = x(7:9);                                               %phi, the, psi
rateB               = x(10:12);                                             %p, q, r    

%% body parameter define 
u                   = velB(1);
v                   = velB(2);
w                   = velB(3);
phi                 = euler(1);
theta               = euler(2);
psi                 = euler(3);
p                   = rateB(1);
q                   = rateB(2);
r                   = rateB(3);

%% frame rotation matrix define 
R                   = EulerDCM(euler);
W                   = pqrDCM(euler);

% dynamics 
PosDot              = R * velB;

velBDot             = [r*v - w*q + Fb(1)*(1/m);
                       -u*r + w*p + Fb(2)*(1/m);
                       u*q - v*p + Fb(3)*(1/m)];

eulerDot            = W * rateB;                   

rateBDot            = [(1/Ix)*(Mb(1) + (Iy-Iz)*q*r);
                       (1/Iy)*(Mb(2) + (Iz-Ix)*r*q);
                       (1/Iz)*(Mb(3) + (Ix-Iy)*p*q)];
                   

xDot                = [PosDot; velBDot; eulerDot; rateBDot];

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
