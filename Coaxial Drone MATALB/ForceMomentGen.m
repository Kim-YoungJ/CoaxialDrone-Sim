function [Fb, Mb] = ForceMomentGen(x, u, param, groundEffectModelNum)

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

kQ                  =   cM / cT;

%% state define 
pos                 =   x(1:3);                                               %N, E, D
velB                =   x(4:6);                                               %u, v, w
euler               =   x(7:9);                                               %phi, the, psi
rateB               =   x(10:12);                                             %p, q, r   

%% frame rotation matrix define 
R                   =   EulerDCM(euler);
velNED              =   R * velB;

%% control input define 
T1                  =   u(1);                                                 % upper rotor 
T2                  =   u(2);                                                 % lower rotor
alpha1              =   u(3);                                                 % control fin 1
alpha2              =   u(4);                                                 % control fin 2
alpha3              =   u(5);                                                 % control fin 3
alpha4              =   u(6);                                                 % control fin 4

%% gravity 
FbGravity           =   R' * [0; 0; m*g];
MbGravity           =   [0; 0; 0];

%% thrust 
Area                =   pi * rprop^2;
rho                 =   1.225;

Vu                  =   sqrt(T1 / (2*rho*Area));
Vl                  =   sqrt(T2 / (2*rho*Area));
Vs                  =   (Vl+Vu) + sqrt((Vl+Vu)^2 - (Vu^2) * (2-(Vu/(Vu+Vl))));

Tu                  =   T1;
Tl                  =   rho*Area*(Vu+Vl)*Vs - rho*Area*Vu^2;

% thrust 
FbThrust            =   [0;
                         0;
                         -Tu-Tl];   
% FbThrust            =   [0;
%                          0;
%                          -T1-T2];
% fprintf("nominal : %f\n", FbThrust(3));
% 
MbThrust            =   [0;
                         0;
                         -Tu*kQ + kQ*Tl];   
% MbThrust            =   [0;
%                          0;
%                          -T1*kQ + kQ*T2];   
                                      
%% aero dyn
q                   =   0.5*1.225*(2*Vs)^2;


FbAero              =   [cL*finA*q*(alpha2-alpha4);
                         cL*finA*q*(alpha1-alpha3);
                         0];

MbAero              =   [cL*finA*q*(-alpha1+alpha3)*dz;
                         cL*finA*q*(alpha2-alpha4)*dz;
                         cL*finA*q*(alpha1+alpha2+alpha3+alpha4)*d];

if (groundEffectModelNum  ~= 0)
    FbThrust(3)     =   GroundEffectModel(groundEffectModelNum, rprop, pos(3), FbThrust(3));
    % fprintf("ground effect : %f\n", FbThrust(3));
else 
    % pass 
end                   

%% Total Force & Moment
Fb                  =   FbGravity + FbThrust + FbAero;
Mb                  =   MbGravity + MbThrust + MbAero;

end

%% Functions
function R = EulerDCM(euler)
    roll    =   euler(1);
    pitch   =   euler(2);
    yaw     =   euler(3);
    
    R(1,1)  =   cos(yaw) * cos(pitch);
    R(1,2)  =   -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll); 
    R(1,3)  =   sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
    R(2,1)  =   sin(yaw)*cos(pitch);
    R(2,2)  =   cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
    R(2,3)  =   -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
    R(3,1)  =   -sin(pitch);
    R(3,2)  =   cos(pitch)*sin(roll);
    R(3,3)  =   cos(pitch)*cos(roll);
end


function effect = GroundEffectModel(model_number, rprop, z, thrust)

    if model_number == 1
        rho    = 5;
        effect = (1 / (( 1 - rho * (1.0*rprop / (4 * max(-z, 0.2)))^2))) * thrust;
        
    elseif model_number == 2
         effect = (0.9926 + 0.03794 * (2 * rprop / max(-z,0.01))^2) * thrust;
    
    elseif model_number == 3
        Ca = 1.7;
        Cb = 0.44;
        effect = (Ca * (exp(-Cb/rprop)*max(-z,0.01)) + 1) * thrust;

    end

end