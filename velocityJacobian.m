% function [R] = angleAxis2Rot(Omega) --> change angle axis to rotation
% matrix
%
% Returns the rotation matrix encoded by an angle-
% axis (Omega = theta*k) rotation of theta
% radians about the unit vector k axis
% 
%
% R = rotation matrix translating to Omega
%
% Omega = vector containing magnitude and direction of a given rotation
%
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda
% yadda yadda yadda yadda yadda yadda yadda yadda yaddayadda yadda yadda
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023
function [Jv, Jv_dot] = velocityJacobian(linkList, paramList, paramRateList)

% initate some stuff ya know
n = length(linkList);
Jv = zeros(6, n);
% Jv_dot = zeros(6, n);
w_k = zeros(3, n);
v_0k = zeros(3, n);

%
if(~exist('paramRateList'))

    %
    paramRateList = zeros(n, 1);

end

% get T and d final
T0N = dhFwdKine(linkList, paramList);
d0N = T0N(1:3, 4); 

%
T01 = dhFwdKine(linkList(1), paramList(1));
d01 = T01(1:3, 4);

% build first column of velocity jacobian --> Jv
if(isempty(linkList(1).d)) % prysmatic

    Jv(:, 1) = [0; 0; 1; 0; 0; 0];
    v_0k = paramRateList(1) * [0; 0; 1];

elseif(isempty(linkList(1).theta)) % rotary

    Jv(:, 1) = [cross([0; 0; 1], d0N); [0 0 1]'];
    w_k(:, 1) = paramRateList(1) * [0; 0; 1];
    v_0k(:, 1) = cross(w_k(:, 1), d01);

end

% build the rest of Jv
if(n >= 2)

    %
    for k = 2:n
    
        % T_0:k-1
        T0km1 = dhFwdKine(linkList(1:k-1), paramList(1:k-1));
    
        % d_0:k-1 and d_k-1:n
        d0km1 = T0km1(1:3, 4);
        dkm1N = d0N - d0km1;

        %
        T0k = dhFwdKine(linkList(1:k), paramList(1:k));
        d0k = T0k(1:3, 4);
        dkm1k = d0k - d0km1;

        % choose which structure to build Jv kth column
        if(isempty(linkList(k).d)) % prysmatic

            %
            Jv(:, k) = [T0km1(1:3, 3); 0; 0; 0];
            w_k(:, k) = w_k(:, k-1);
            v_0k(:, k) = v_0k(:, k-1) + cross(w_k(:, k), dkm1k) + paramRateList(k)*T0km1(1:3, 3); 

        elseif(isempty(linkList(k).theta)) % rotary

            %
            Jv(:, k) = [cross(T0km1(1:3, 3), dkm1N); T0km1(1:3, 3)];    
            w_k(:, k) = w_k(:, k-1) + paramRateList(k) * T0km1(1:3, 3);
            v_0k(:, k) = v_0k(:, k-1) + cross(w_k(:, k), dkm1k);

        end    
    end
end

if(paramRateList ~= zeros(n, 1))

    %
    Jv_dot = zeros(6, n);

    % build first column of velocity jacobian --> Jv dot
    if(linkList(1).isRotary == 0) % prysmatic
    
        Jv_dot(1:3, 1) = cross(w_k(:, 1), [0; 0; 1]);
        Jv_dot(4:6, 1) = [0; 0; 0];
    
    
    elseif(linkList(1).isRotary == 1) % rotary
    
        ang_rate = cross(w_k(:, 1), [0; 0; 1]);
        Jv_dot(1:3, 1) = cross(ang_rate, d0N) + cross([0; 0; 1], v_0k(:, n));
        Jv_dot(4:6, 1) = cross(w_k(:, 1), [0; 0; 1]);
    
    end
    
    % now build Jv doot
    if(n >= 2)
        
        for k = 2:n
        
            %T_0:k-1
            T0km1 = dhFwdKine(linkList(1:k-1), paramList(1:k-1));
            
            % d_0:k-1 and d_k-1:n
            z0km1 = T0km1(1:3, 3);
            d0km1 = T0km1(1:3, 4);
            dkm1N = d0N - d0km1;
        
            %
            if(linkList(k).isRotary == 1) % rotary
        
                d_dot = v_0k(:, n) - v_0k(:, k-1);
                ang_rate = cross(w_k(:, k), z0km1);
                Jv_dot(1:3, k) = cross(ang_rate, dkm1N) + cross(z0km1, d_dot);
                Jv_dot(4:6, k) = cross(w_k(:, k), z0km1);
        
            elseif(linkList(k).isRotary == 0) % prysmtaic
        
                Jv_dot(1:3, k) = cross(w_k(:, k), z0km1);
                Jv_dot(4:6, k) = [0; 0; 0];
        
            end
        end
    end

else

    Jv_dot = [];

end

end


%%

%%

%%

%%

%%

%%

%%

%%

%%