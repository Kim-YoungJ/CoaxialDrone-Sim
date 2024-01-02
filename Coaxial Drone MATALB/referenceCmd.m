%% Reference Test Case
function output = referenceCmd(nx, nStep, dt, Num)
    
    % unit define
    D2R                 =   pi/180;

    switch Num
        case 1       % Various Step Input
            time    = 0;
            refCmd  = zeros(nx,1);
            
            for i = 1 : nStep
                if (time>=0 && time <10) 
                    refCmd(3,i) = -0.2;
                    refCmd(9,i) = 5.0 * D2R;
                    time = time + dt;
                elseif (time>=10 && time <20)
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;                    
                elseif (time>=20 && time <30)
                    refCmd(3,i) = -0.2;
                    refCmd(9,i) = -5.0 * D2R;
                    time = time + dt;    
                else 
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;
                end
            end
        
            output = refCmd;
        
        case 2      % Step Input 
            time    = 0;
            refCmd  = zeros(nx,1);
            
            for i = 1 : nStep
                if (time>=0 && time <10) 
                    refCmd(3,i) = -0.2;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;
                elseif (time>=10 && time <20)
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;                    
                elseif (time>=20 && time <30)
                    refCmd(3,i) = -0.2;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;    
                else 
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;
                end
            end
        
            output = refCmd;

        case 3      % Landing
            time    = 0;
            refCmd  = zeros(nx,1);
            
            for i = 1 : nStep
                if (time>=0 && time <5) 
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;
                elseif (time>=5 && time <35)
                    refCmd(3,i) = 0.6/30*(time-5)-0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;                       
                else 
                    refCmd(3,i) = -0.2;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;
                end
            end
        
            output = refCmd;

        case 4      % yaw step input 
            time    = 0;
            refCmd  = zeros(nx,1);
            
            for i = 1 : nStep
                if (time>=0 && time <10) 
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 5.0 * D2R;
                    time = time + dt;
                elseif (time>=10 && time <20)
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;                    
                elseif (time>=20 && time <30)
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = -5.0 * D2R;
                    time = time + dt;    
                else 
                    refCmd(3,i) = -0.8;
                    refCmd(9,i) = 0.0 * D2R;
                    time = time + dt;
                end
            end
        
            output = refCmd;

        case 5      % sinosodial
            time    = 0;
            refCmd  = zeros(nx,1);
            
            for i = 1 : nStep
                refCmd(3,i) = -0.8 + 0.8*sin((2*pi/30)*time);
                refCmd(9,i) = 0 + 5*(pi/180)*sin((2*pi/30)*time);
                time = time + dt;
            end
        
            output = refCmd;

        case 6      % sinosodial
            time    = 0;
            refCmd  = zeros(nx,1);
            
            for i = 1 : nStep
                refCmd(3,i) = -0.4 + 0.4*sin((2*pi/25)*time);
                refCmd(9,i) = 0 + 3*(pi/180)*sin((2*pi/25)*time);
                time = time + dt;
            end
        
            output = refCmd;

        otherwise
            fprintf("Select refNum 1 to 5");
            
    end

end