function [jointTorques] = newtonEuler( linkList, paramList, paramListDot, paramListDDot, boundry_conditions )
% NEWTONEULER calculates the joint torques, ...
%
% [jointTorques] = newtonEuler( linkList, paramList,
%                      paramListDot, paramListDDot,  boundry_conditions )
%
% Outputs:
%  jointTorques: ...
%
% Inputs:
%   linkList: ...
%
%   Author: ...
%   Last Mod: ...
%

% Number of Joints
numJoints = length(linkList);

% Pre allocate a list of values that need to be stored in memory between
% loops
list = repmat(struct( 'Zlast', zeros(3,1),...   % Z i-1, rotation axis for link i in base frame
    'Woi', zeros(3,1),...     % Angular velocity of origin i in base frame
    'doi', zeros(3,1),...     % Position of Origin i relative to base in base frame
    'Fi', zeros(3,1),...      % Inertial Force on link i in base frame
    'Ni', zeros(3,1),... % Inertial Torque on link i in base frame
    'rii', zeros(3,1),...% Displacement from i-1 to com
    'ri1_i',zeros(3,1) ),...% Displacemenbt from i to com
    numJoints,1);

% Initialize link variables that get propagated forward
Toi = eye(4); % Transform from 0 to joint i
W = boundry_conditions.base_angular_velocity; % Angluar Velocity in joint frame
Wdot = boundry_conditions.base_angular_acceleration; % Angular Acceleration in joint frame
Vdot = boundry_conditions.base_linear_acceleration; % Linear acceleration in joint frame

W_BC = boundry_conditions.base_angular_velocity; %Necessary for Jv and Jvdot

num_static = 0; % number of static links encountered

for i=1:numJoints % begin forward iteration from base to tool
    
    % Calculate link transform from i-1 to i
    if linkList(i).isRotary == 1
        % hint i-num_static is the param index to be on
        Ti = ;
    elseif linkList(i).isRotary == 0
        Ti = ;
    else
        Ti = ;
        num_static = num_static+1;
    end
    
    % extract distance from joint i-1 to joint i
    di = Ti(1:3,4);
    
    % Roi
    Roi = Toi(1:3,1:3);
    
    % extract rotation from joint i to joint i-1 (transpose of i-1 to i
    % rotation)
    Rt = Ti(1:3,1:3)';
    
    % Update joint frame acceleartion, angular Acceleration, and
    % angular velocity.  In the i-i frame (so z is [0;0;1])
    Z = [0;0;1];
    if linkList(i).isRotary == 1
        Wdot = Wdot + ; % update ang accel in joint frame
        W = W + ; % update ang vel
        
        Vdot = Vdot + ; % update accel in joint frame
    elseif linkList(i).isRotary == 0
        Wdot = Wdot; % update ang accel in joint frame
        W = W; % updage ang vel
        
        Vdot = Vdot + ; % update accel in joint frame
    else
        Wdot = Wdot; % update ang accel in joint frame
        W = W; % updage ang vel
        
        Vdot = Vdot + ; % update accel in joint frame
    end
    
    % rotate from i-1 frame to i frame
    Wdot = Rt*Wdot;
    W = Rt*W;
    V = Rt*V;
    Vdot = Rt*Vdot;
    
    % Calculate the displacement from the i-1 frame to the i'th com in
    % the i'th frame
    ri1_i =  Rt*di+linkList(i).com;
    
    % Calculate the Acceleration of the Center of Mass of the link in
    % the ith frame
    Vcdot = Vdot + ;
    
    % Calculate and Save Inertial Force and Torque in the i'th frame
    F = ; % Newton's Equation
    N = ; % Euler's Equation
    
    
    
    % Save values specific to calculating Jv and JvDot that we already know
    list(i).Zlast  = Toi(1:3,3); % Save Zi-1 for Jv and JvDot
    Toi = Toi*Ti; % Update base to link transform
    list(i).doi = Toi(1:3,4); % save distance from base to joint i for Jv and JvDot
    list(i).Woi = Toi(1:3,1:3)*W; % Save Wi i base frame for JvDot
    list(i).Fi = Toi(1:3,1:3)*F; % Inertial Force in base frame
    list(i).Ni = Toi(1:3,1:3)*N; % Inertial Torque in base frame
    list(i).ri1_i = Toi(1:3,1:3)*ri1_i; % Displacement from i-1 to com
    list(i).rii   = Toi(1:3,1:3)*linkList(i).com; % Displacemenbt from i to com
end % End Forward Iterations

% Initialize variables for calculating Jv and JvDot
doN = list(end).doi; % Extract Distance from Base to Tool in base frame

% Initialize variables for force/torque propagation
f = boundry_conditions.distal_force; % Initialize force to external force on the tool in the tool frame
n = boundry_conditions.distal_torque; % Initialize torque to external torque on the tool in the tool frame

% Rotate f & n to base frame
f = ;
n = ;

% preallocate joint torque vector
jointTorques = zeros(numJoints,1); % preallocate for speed

for i = numJoints:-1:1 % From Last joint to base
    
    % displacement from origin i-1 to i in base. Hint: use list(i).doi to help...
    if( i> 1)
        d = ;
    else
        d = list(i).doi;
    end
    
    %First Half or torque update based on applied force
    n = n - ;
    
    % Update Force on joint i in base frame with inertial force from before
    f = f + ;
    
    % Final Update Torque on joint i in base frame with inertial torque and
    % constraint force
    n = n + ;
    
    if linkList(i).isRotary == 1 % Rotational Joint
        % joint i torque is the Z component
        jointTorques(i-num_static) = ;
        
    elseif linkList(i).isRotary == 0 % Prysmatic
        % joint i force is the Z component
        jointTorques(i-num_static) = ;
        
    else
        num_static = num_static-1;
    end
    
end % End Backword Iterations
end % end newtonEular Function definition