%% constantAccelInterp interpolates forr constant accel
% 
% [p, v, a] = constantaccelintep(t, taj, tperc)
%
% output - position, velocity, acceleration
% these are 2D values interpolated according to time steps provided
%
% input - time, tajectoy list, transpercent
% time is the time of interpolation
% traj list is a list of positions with associated times
% transpercent is the the transition percentage
% 
% this solution seemed to be super tricky, because some test cases work
% with one set of equations but not another, and then when I alter
% equations to get other test cases that weren't passed successfully
% previously, the test cases that worked before don't work
%
% Nick Taylor
% 10920730
% MEGN 544
function [p v a] = constAccelInterpx(ti, traj, tau)

%
[n, m] = size(traj);

%
p = zeros(1, m-1);
v = zeros(1, m-1);
a = zeros(1, m-1);

%
time = traj(:, 1);
x = traj(:, 2);
y = traj(:, 3);

%
for i = 1:n-1

    %
    ta = time(i);
    tb = time(i+1);

    %
    if(ti >= ta && ti <= tb)

        %
        if(ti < tb - tau)

            %
            if(ti < ta + tau && i > 1)

                %
                vx = (x(i) - x(i-1)) / (ta - time(i-1));
                vy = (y(i) - y(i-1)) / (ta - time(i-1));

                %
                vx_bc = (x(i+1) - x(i)) / (tb - ta);
                vy_bc = (y(i+1) - y(i)) / (tb - ta);

                %
                a(1) = (vx_bc - vx) / (2*tau); 
                a(2) = (vy_bc - vy) / (2*tau);
                
                %
                P2(1) = x(i) - vx * tau;
                P2(2) = y(i) - vy * tau;
    
                %
                t2 = ta - tau;
        
                %
                v(1) = vx + a(1) * (ti - t2);
                v(2) = vy + a(2) * (ti - t2);
        
                %
                p(1) = P2(1) + vx * (ti - t2) + 0.5 * a(1) * (ti - t2)^2;
                p(2) = P2(2) + vy * (ti - t2) + 0.5 * a(2) * (ti - t2)^2;

            else
    
                %
                vx = (x(i+1) - x(i)) / (tb - ta);
                vy = (y(i+1) - y(i)) / (tb - ta);
        
                %
                p(1) = x(i) + vx * (ti - ta);
                p(2) = y(i) + vy * (ti - ta);
        
                %
                v = [vx, vy];
        
                %
                a = [0, 0];

            end
    
    
        elseif(ti > tb - tau && ti < tb + tau)

            %
            vx = (x(i+1) - x(i)) / (tb - ta);
            vy = (y(i+1) - y(i)) / (tb - ta);

            %
            if(i+2 <= n)

                %
                vx_bc = (x(i+2) - x(i+1)) / (time(i+2) - tb);
                vy_bc = (y(i+2) - y(i+1)) / (time(i+2) - tb);

                %
                a(1) = (vx_bc - vx) / (2*tau); 
                a(2) = (vy_bc - vy) / (2*tau);

            else

                a(1) = 0;
                a(2) = 0;

            end
    
            %
            P2(1) = x(i+1) - vx * tau;
            P2(2) = y(i+1) - vy * tau;

            %
            t2 = tb - tau;
    
            %
            v(1) = vx + a(1) * (ti - t2);
            v(2) = vy + a(2) * (ti - t2);
    
            %
            p(1) = P2(1) + vx * (ti - t2) + 0.5 * a(1) * (ti - t2)^2;
            p(2) = P2(2) + vy * (ti - t2) + 0.5 * a(2) * (ti - t2)^2;
    
        end    

    elseif(ti > time(n))

        %
        p = traj(n, 2:3);
        break;

    end
end



end
