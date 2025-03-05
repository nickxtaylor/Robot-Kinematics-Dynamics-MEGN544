% function [R] = angleAxis2Rot(Omega) --> change angle axis to rotation
% matrix
%
% Returns the rotation matrix encoded by an angle-
% axis (Omega = theta*k) rotation of theta
% radians about the unit vector k axis
% 
%
% LinkList
% desired transformation matrix
%
% Guess of a parameter list
%
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023
function [paramList, er] = dhInvKine(linkList, Td, paramListGuess)

% initialize
Tc = dhFwdKine(linkList, paramListGuess);
error = transError(Td, Tc); er = norm(error);
n = length(linkList);
dp = 10^3 * ones(n, 1); paramList = paramListGuess;
tol = 10^(-10); 

% what should tolerance be?
while(er > tol && norm(dp) > tol)

    %
    [Jv, ~] = velocityJacobian(linkList, paramList);
    [U, S, V] = svd(Jv);
    Jv_inv = V * pinv(S) * transpose(U); % Jv_pinv = pinv(U * S * V');
    dp = Jv_inv * error; 
    paramList = paramList + dp;
    Tc = dhFwdKine(linkList, paramList);
    error = transError(Td, Tc);
    er = norm(error);

end

end