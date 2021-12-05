function [u] = UnicycleToPoseControl(xTrue, xGoal)
%Computes a control to reach a pose for unicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   xGoal is the goal point
%   u is the control : [v omega]'

% TODO
    K_rho = 15;
    K_alpha = 5;
    K_beta = 15;
    
    alpha_max = pi/4;
    
    error = xGoal - xTrue;
    goalDist = norm(error(1:2));
    AngleToGoal = AngleWrap(atan2(error(2), error(1))-xTrue(3));
    
    u(1) = K_rho*goalDist;
    if abs(AngleToGoal) > alpha_max 
        u(1) = 0;
    end
    
    u(2) = K_alpha*AngleToGoal;
    if goalDist < 0.05
        u(2) = K_beta*error(3);
    end
end

