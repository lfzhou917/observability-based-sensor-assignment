function [tarx_true, tary_true]=target_motion(tarx_initial, tary_initial, t)
                %simulate the fixed target motion model, here, circle  
                global u_max
                Omega = 50;   
                xc = tarx_initial - u_max*cos(0);
                yc = tary_initial - u_max*sin(0);
                tarx_true = u_max*cos(t/Omega) +xc;
                tary_true = u_max*sin(t/Omega) +yc;
end