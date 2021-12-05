function u = BicycleToPathControl(xTrue, Path)
%Computes a control to follow a path for bicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   Path is set of points defining the path : [ x1 x2 ... ;
%                                               y1 y2 ...]
%   u is the control : [v phi]'

% TODO
    % Saves local variables declared in a function to memory for later use by 
    % calling the function. 
    % Persistent variables can only be used by the function they are defined in, 
    % which prevents them from being changed by other functions or other commands.
    persistent id;
    persistent xGoal;
    
    % Set the first goal waypoint on the path when starting the trajectory and record its index
    if xTrue == [0;0;0] %#ok<BDSCA>
        id = 1;
        xGoal = Path(:, id);
    end
    
    % Set a threshold to determine whether the goal waypoint is reached
    threshold = 0.4;

    %% Define current goal
    % Check if the goal waypoint is reached
    error = Path(:, id) - xTrue;
    goalDist = norm(error(1:2));
    if goalDist < threshold
        % Reached
        % Set xGoal to waypoint and update the id
        xGoal = Path(:, id);
        id = id+1;
        id = min(id, size(Path, 2)); % Prevent accesses from exceeding the size of the array

    else
        % Not reached
        % Move goal along line to next waypoint
        
        % Find the unit direction vector to next waypoint
        delta = Path(:, id) - Path(:, id-1);
        delta = delta/norm(delta); 
        
        error = xGoal - xTrue;
        goalDist = norm(error(1:2));
        
        while goalDist < threshold
            xGoal = xGoal + 0.01*delta;
            error = xGoal - xTrue;
            goalDist = norm(error(1:2));
        end
    end

    %% Perform control
    K_rho = 5;
    K_alpha = 8;
 
    error = xGoal - xTrue;
    goalDist = norm(error(1:2));
    alpha = AngleWrap(atan2(error(2), error(1))-xTrue(3));

    u(1) = K_rho * goalDist;
    u(2) = K_alpha * alpha;

end

