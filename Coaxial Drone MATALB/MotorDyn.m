function Uactual = MotorDyn(Ucmd, Uactual, param, dt)
    
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

%% Motor dynamics(1st system)
UactualDot          =   zeros(6,1);
tauMotor            =   0.05;

UactualDot(1)       =   (1/tauMotor)*(Ucmd(1)-Uactual(1));
UactualDot(2)       =   (1/tauMotor)*(Ucmd(2)-Uactual(2));


Uactual             =   Uactual + UactualDot*dt;

Uactual(1)          =   min(max(Uactual(1), 0.1), thrustMax);
Uactual(2)          =   min(max(Uactual(2), 0.1), thrustMax);

end 