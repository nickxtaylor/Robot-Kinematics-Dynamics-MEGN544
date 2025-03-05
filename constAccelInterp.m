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
function [p v a] = constAccelInterp(ti, traj, tau)

%
[n, m] = size(traj);

%
p = zeros(1, m-1);
v = zeros(1, m-1);
a = zeros(1, m-1);

%
time = traj(:, 1);

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
                for j = 1:m-1

                    %
                    x = traj(:, j+1);

                    %
                    vx = (x(i) - x(i-1)) / (ta - time(i-1));
    
                    %
                    vx_bc = (x(i+1) - x(i)) / (tb - ta);

                    %
                    a(j) = (vx_bc - vx) / (2*tau); 
                    
                    %
                    P2 = x(i) - vx * tau;
        
                    %
                    t2 = ta - tau;
            
                    %
                    v(j) = vx + a(j) * (ti - t2);

                    %
                    p(j) = P2 + vx * (ti - t2) + 0.5 * a(j) * (ti - t2)^2;

                end

            else

                for j = 1:m-1
    
                    %
                    x = traj(:, j+1);
                    
                    %
                    vx = (x(i+1) - x(i)) / (tb - ta);
            
                    %
                    p(j) = x(i) + vx * (ti - ta);
            
                    %
                    v(j) = vx;
            
                    %
                    a(j) = 0;

                end

            end
    
    
        elseif(ti > tb - tau && ti < tb + tau)

            for j = 1:m-1

                %
                x = traj(:, j+1);
    
                %
                vx = (x(i+1) - x(i)) / (tb - ta);
    
                %
                if(i+2 <= n)
    
                    %
                    vx_bc = (x(i+2) - x(i+1)) / (time(i+2) - tb);
    
                    %
                    a(j) = (vx_bc - vx) / (2*tau); 
    
                else
    
                    a(j) = 0;
    
                end
        
                %
                P2 = x(i+1) - vx * tau;
    
                %
                t2 = tb - tau;
        
                %
                v(j) = vx + a(j) * (ti - t2);

                %
                p(j) = P2 + vx * (ti - t2) + 0.5 * a(j) * (ti - t2)^2;

            end
    
        end    

    elseif(ti > time(n))

        %
        p = traj(n, 2:end);
        break;

    end
end

end
