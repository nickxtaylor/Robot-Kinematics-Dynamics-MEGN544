function [th] = getTheta(theta, th_last)

%
if(abs(theta(1) - th_last) > abs(theta(2) - th_last))

    th = theta(2);

else

    th = theta(1);

end

end