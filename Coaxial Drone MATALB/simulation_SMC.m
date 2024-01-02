clear; clc; 
close all;

%% UNIT
R2D                 =   180/pi;
D2R                 =   pi/180;

%% Parameter Settings  
nx                  =   12;                                                 % number of state variable
nu                  =   6;                                                  % number of control variable

m                   =   1.585;                                              % mass[kg]
g                   =   9.81;                                               % gravity coefficient[m/s^2]
dz                  =   0.2;                                                % center of gravity offset to control fin z direction[m]
d                   =   0.1;                                                % center of gravity offset to control fin x,y direction[m]
finA                =   9.282 * 10^(-2);                                    % area of control fin[m^2]

Ix                  =   1.08 * 10^(-2);                                     % moment of inertia x
Iy                  =   1.08 * 10^(-2);                                     % moment of inertia y
Iz                  =   3.05 * 10^(-2);                                     % momnet of inertia z

thrustMax           =   10.0;                                               % propeller thrust max[N]
cT                  =   1.1859 * 10^(-7);                                   % propeller lift coefficient
cM                  =   2.4167 * 10^(-9);                                   % propeller moment coefficient
cL                  =   2 * pi;                                             % control fin lift coefficient

lprop               =   0.10;                                               % distance between coaixal rotors[m]
rprop               =   0.2794;                                             % propeller radius[m]
finMax              =   30 * D2R;                                           % maximum fin angle deflection[rad]

param               =   [m; g; dz; d; finA; Ix; Iy; Iz; 
                       thrustMax; cT; cM; cL; lprop; rprop; finMax];

%% Initial value settings 
% Inertial NED Frame
N0                  =   0;
E0                  =   0;   
D0                  =   -0.8;

% Body Frame(Vel)
u0                  =   0;
v0                  =   0;
w0                  =   0;

% Body Frame(Euler)
phi0                =   0 * D2R;
the0                =   0 * D2R;
psi0                =   0 * D2R;

% Body Frame(Euler Rate)
p0                  =   0;
q0                  =   0;
r0                  =   0;

%% Datas
x                   =   [N0;E0;D0;u0;v0;w0;phi0;the0;psi0;p0;q0;r0];
xDot                =   zeros(nx,1);
xRef                =   [N0;E0;D0;u0;v0;w0;phi0;the0;psi0;p0;q0;r0];

u                   =   [m*g/2; m*g/2;
                         0; 0; 0; 0];
uReal               =   [m*g/2; m*g/2;
                         0; 0; 0; 0];

Fb                  =   zeros(3,1);
Mb                  =   zeros(3,1);

%% Controller Settings 
controlParam        =   ControlSettings_pid();

%% Main Simulation 

% ground effect option
groundEffectModel   =   1;

% Save Data name option
SaveOption          =   false;

% simulation initialize
t0                  =   0.0;
tf                  =   40;                                                %  [sec]
dt                  =   0.001;
nStep               =   fix(tf / dt);

counter             =   0;
controlHz           =   200;
time                =   0;

% Reference Generation
refNum              =   2;
xCmd                =   referenceCmd(nx, nStep, dt, refNum);
xRefCmd             =   xCmd;

for i = 1:nStep

   % Saving Data
   outSim.time(:,i)      =  time;

   outSim.x(:,i)         =  x;
   outSim.xCmd(:,i)      =  xCmd(:,i);
   outSim.xDot(:,i)      =  xDot;
   
   outSim.xRef(:,i)      =  xRef;
   outSim.xRefCmd(:,i)   =  xRefCmd(:,i);

   outSim.u(:,i)         =  u;
   outSim.uReal(:,i)     =  uReal;

   outSim.Fb(:,i)        =  Fb;
   outSim.Mb(:,i)        =  Mb;
   
   % Controller
   if (rem(counter, 5) == 0)

       [u, innerCmd]    =   basecontroller_smc(xCmd(:,i), x, 1/controlHz, param, controlParam);
       counter          =   0;
   else
       counter          =   counter + 1;
   end
    
   outSim.xCmd(6,i)        =   innerCmd(6,1); 
   outSim.xCmd(10,i)       =   innerCmd(10,1); 
   outSim.xCmd(11,i)       =   innerCmd(11,1); 
   outSim.xCmd(12,i)       =   innerCmd(12,1);
   
   % Reference Dynamics 
   [xRefDot, innerRefCmd]   =  refDyn(xRef, xCmd(:,i), controlParam, dt);
  
   % Dynamics 
   uReal                =   MotorDyn(u,uReal, param, dt);
   [Fb, Mb]             =   ForceMomentGen(x, uReal, param, groundEffectModel);
   [xDot]               =   Dyn6Dof(x, Fb, Mb, param);
   
    
   % Integration 
   xRef             =   xRef + xRefDot * dt;
   x                =   x + xDot * dt;
   time             =   time + dt;

   
end

%% Save Datas
if SaveOption == true
    pidResult = outSim;
    save PID_yaw.mat pidResult
end

%% Plot (control results)
figure(1) 
plot(outSim.time, -outSim.xCmd(3,:), '--', 'linewidth', 2)
hold on 
grid on
plot(outSim.time, -outSim.x(3,:), 'linewidth', 2)
title("Height", 'FontSize',14)
legend("Cmd", "State", 'FontSize',18)
ylabel("Z [m]", 'FontSize',14)
xlabel("time [s]", 'FontSize',14)

figure(2)
sgtitle('Attitude', 'FontSize',14)
subplot(3,1,1);
plot(outSim.time, R2D*outSim.xCmd(7,:), '--', 'linewidth', 2)
hold on 
grid on
plot(outSim.time, R2D*outSim.x(7,:), 'linewidth', 2)
legend("Cmd", "State", 'FontSize',14)
ylabel("Roll [deg]", 'FontSize',14)
ylim([-10,10])

subplot(3,1,2);
plot(outSim.time, R2D*outSim.xCmd(8,:), '--', 'linewidth', 2)
hold on 
grid on
plot(outSim.time, R2D*outSim.x(8,:), 'linewidth', 2)
legend("Cmd", "State", 'FontSize',18)
ylabel("Pitch [deg]", 'FontSize',14)
ylim([-10,10])

subplot(3,1,3);
plot(outSim.time, R2D*outSim.xCmd(9,:), '--', 'linewidth', 2)
hold on 
grid on
plot(outSim.time, R2D*outSim.x(9,:), 'linewidth', 2)
legend("Cmd", "State", 'FontSize',18)
ylabel("Yaw [deg]", 'FontSize',14)
xlabel("time [s]", 'FontSize',14)


figure(3)
sgtitle('Rate', 'FontSize',14)
subplot(3,1,1);
plot(outSim.time, R2D*outSim.xCmd(10,:), '--', 'linewidth', 2)
hold on 
grid on
plot(outSim.time, R2D*outSim.x(10,:), 'linewidth', 2);
legend("Cmd", "State", 'FontSize',18)
ylabel("p [deg/s]", 'FontSize',14)
ylim([-10,10])

subplot(3,1,2);
plot(outSim.time, R2D*outSim.xCmd(11,:), '--', 'linewidth', 2)
hold on 
grid on
plot(outSim.time, R2D*outSim.x(11,:), 'linewidth', 2);
legend("Cmd", "State", 'FontSize',18)
ylabel("q [deg/s]", 'FontSize',14)
ylim([-10,10])

subplot(3,1,3);
plot(outSim.time, R2D*outSim.xCmd(12,:), '--', 'linewidth', 2)
hold on 
grid on
plot(outSim.time, R2D*outSim.x(12,:), 'linewidth', 2);
legend("Cmd", "State", 'FontSize',18)
ylabel("r [deg/s]", 'FontSize',14)
xlabel("time [s]", 'FontSize',14)

figure(4) 
plot(outSim.time, outSim.u(1,:), 'linewidth', 2)
hold on;
grid on
plot(outSim.time, outSim.u(2,:), 'linewidth', 2)
legend("T1","T2", 'FontSize',18);
title('Thrust', 'FontSize',14)
xlabel('time [s]', 'FontSize',14)
ylabel('Thrust [N]', 'FontSize',14)

figure(5)
plot(outSim.time, outSim.u(3,:), 'linewidth', 2)
hold on;
grid on
plot(outSim.time, outSim.u(4,:), 'linewidth', 2)
plot(outSim.time, outSim.u(5,:), 'linewidth', 2)
plot(outSim.time, outSim.u(6,:), 'linewidth', 2)
legend("Fin1","Fin2","Fin3","Fin4", 'FontSize',18)
title("Fin Angle", 'FontSize',14)
ylabel("Fin Angle [deg]", 'FontSize',14)
ylim([-10,10])
xlabel("time [s]", 'FontSize',14)

%% reference vs nominal
figure(6)
subplot(2,1,1)
plot(outSim.time, -outSim.xCmd(3,:), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, -outSim.xRef(3,:), "-.", 'linewidth', 2.5);
plot(outSim.time, -outSim.x(3,:), '-', 'linewidth', 2.5);
subtitle("z", 'FontSize',14)
ylabel("Height [m]", 'FontSize',14)
legend("Command","reference", "response", 'FontSize',14)


subplot(2,1,2)
plot(outSim.time, -outSim.xCmd(6,:), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, -outSim.xRef(6,:), "-.", 'linewidth', 2.5);
plot(outSim.time, -outSim.x(6,:), '-', 'linewidth', 2.5);
subtitle("velbodyD", 'FontSize',14)
ylabel("velD [m/s]", 'FontSize',14)
legend("Command","reference", "response", 'FontSize',14)

figure(7)
subplot(3,1,1)
plot(outSim.time, R2D*(outSim.xCmd(7,:)), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, R2D*(outSim.xRef(7,:)), "-.", 'linewidth', 2.5);
plot(outSim.time, R2D*(outSim.x(7,:)), '-', 'linewidth', 1.0);
subtitle("roll", 'FontSize',14)
ylabel("roll [deg]", 'FontSize',14)
legend({'Command','reference', 'response'}, 'FontSize',14, 'FontSize',14)
ylim([-1,1])

subplot(3,1,2)
plot(outSim.time, R2D*(outSim.xCmd(8,:)), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, R2D*(outSim.xRef(8,:)), "-.", 'linewidth', 2.5);
plot(outSim.time, R2D*(outSim.x(8,:)), '-', 'linewidth', 2.5);
subtitle("pitch", 'FontSize',14)
ylabel("pitch [deg]", 'FontSize',14)
ylim([-1,1])
legend("Command","reference", "response", 'FontSize',14)


subplot(3,1,3)
plot(outSim.time, R2D*(outSim.xCmd(9,:)), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, R2D*(outSim.xRef(9,:)), "-.", 'linewidth', 2.5);
plot(outSim.time, R2D*(outSim.x(9,:)), '-', 'linewidth', 2.5);
subtitle("yaw", 'FontSize',14)
ylabel("yaw [deg]", 'FontSize',14)
legend("Command","reference", "response", 'FontSize',14)

figure(8)
subplot(3,1,1)
plot(outSim.time, R2D*(outSim.xCmd(10,:)), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, R2D*(outSim.xRef(10,:)), "-.", 'linewidth', 2.5);
plot(outSim.time, R2D*(outSim.x(10,:)), '-', 'linewidth', 2.5);
subtitle("p", 'FontSize',14)
ylabel("p [deg/s]", 'FontSize',14)
ylim([-1,1])
legend("Command","reference", "response", 'FontSize',14)

subplot(3,1,2)
plot(outSim.time, R2D*(outSim.xCmd(11,:)), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, R2D*(outSim.xRef(11,:)), "-.", 'linewidth', 2.5);
plot(outSim.time, R2D*(outSim.x(11,:)), '-', 'linewidth', 2.5);
subtitle("q", 'FontSize',14)
ylabel("q [deg/s]", 'FontSize',14)
ylim([-1,1])
legend("Command","reference", "response", 'FontSize',14)

subplot(3,1,3)
plot(outSim.time, R2D*(outSim.xCmd(12,:)), '--', 'linewidth', 2.5)
hold on
grid on
plot(outSim.time, R2D*(outSim.xRef(12,:)), "-.", 'linewidth', 2.5);
plot(outSim.time, R2D*(outSim.x(12,:)), '-', 'linewidth', 2.5);
subtitle("r", 'FontSize',14)
ylabel("r [deg/s]", 'FontSize',14)
legend("Command","reference", "response", 'FontSize',14)


