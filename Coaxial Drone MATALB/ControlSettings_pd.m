function paramSet = ControlSettings_pd()
    
    %% global Variable 
    global Izdot Ip Iq Ir;
    global IzdotRef IpRef IqRef IrRef;
    
    Izdot       =   0;
    Ip          =   0;
    Iq          =   0;
    Ir          =   0;
    IzdotRef    =   0; 
    IpRef       =   0;
    IqRef       =   0;
    IrRef       =   0;
    
    %% Gain Tunning
    %%%%%%% PID Gain %%%%%%
    tau_D       =   0.4;
    tau_velD    =   0.1;
    
    tau_roll    =   0.9;
    tau_p       =   0.3;
    
    tau_pitch   =   0.9;
    tau_q       =   0.3;
    
    tau_yaw     =   0.4;
    tau_r       =   0.1;
    
    K_D         =   1 / tau_D;
    K_velD      =   1 / tau_velD;
    
    K_roll      =   1 / tau_roll;
    K_pitch     =   1 / tau_pitch;
    K_yaw       =   1 / tau_yaw;
    K_p         =   1 / tau_p;
    K_q         =   1 / tau_q;
    K_r         =   1 / tau_r;
    
    Igain_velD  =   (1 / tau_velD) * 0.1;
    Igain_p     =   (1 / tau_p) * 0.1;
    Igain_q     =   (1 / tau_q) * 0.1;
    Igain_r     =   (1 / tau_r) * 0.1;

    paramSet    = [K_D; K_velD; K_roll; K_pitch; K_yaw; K_p; K_q; K_r; Igain_velD; Igain_p; Igain_q; Igain_r];

end