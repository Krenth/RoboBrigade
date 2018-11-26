%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular point
%Paul Glotfelter 
%3/24/2016
clear all, close all, clc

N = 5;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
r.step();

% Create a barrier certificate so that the robots don't collide
si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.22);
si_to_uni_dynamics = create_si_to_uni_mapping2();
        
% flag: is the i.c. a evenly spaced circular pattern?
circularInitialConditions = 1;

% Reach initial positions on a circle
if circularInitialConditions        
    circularTargets = 0.6*[ cos( 0:2*pi/N:2*pi*(1- 1/N) ) - 1.7 * ones(1,N) ; sin( 0:2*pi/N:2*pi*(1- 1/N) ) ];
    errorToInitialPos = x(1:2,:) - circularTargets;                % Error
    errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
    while max( errorNorm ) > 0.05
        % Update state variables        
        x = r.get_poses();                            % States of real unicycle robots
        xuni = x(1:2,:);                                    % x-y positions
        
        % Update errors
        errorToInitialPos = xuni - circularTargets;
        errorNorm = [1,1]*(errorToInitialPos.^2);
        
        % Conput control inputs
        u = -0.3.*errorToInitialPos;
        u = si_barrier_certificate(u, x);
        dx = si_to_uni_dynamics(u, x);
        
        % Assing new control inputs to robots
        
        r.set_velocities(1:N, dx);                       % Assign dummy zero velocity
        r.step();                                        % Run robotarium step
    end
    disp('Initial positions reached')  
end

% % We'll make the rotation error huge so that the initialization checker
% % doesn't care about it
% args = {'PositionError', 0.1, 'RotationError', 50};
% init_checker = create_is_initialized(args{:});
% controller = create_si_position_controller();

% while(~init_checker(x, initial_conditions))
% 
%     x = r.get_poses();
%     dxi = controller(x(1:2, :), initial_conditions(1:2, :));
%     dxi = si_barrier_certificate(dxi, x(1:2, :));      
%     dxu = si_to_uni_dynamics(dxi, x);
% 
%     r.set_velocities(1:N, dxu);
%     r.step();   
% end

% %% Create the desired Laplacian
 iterations = 2000;
% l = .53;
% p = 1.5;
% q = .9183681179;
% %Graph laplacian
% 
% L = zeros(N, N);
% A = [0,1,1,1,1;...
%      1,0,1,1,1;...
%      0,0,0,0,0;...
%      1,1,1,0,1;...
%      1,1,1,1,0];
% delta = [4,0,0,0,0;
%          0,4,0,0,0;
%          0,0,0,0,0;
%          0,0,0,4,0
%          0,0,0,0,4];
% L = delta-A;
% weights = [0,l,l*2,q,p;
%            l,0,l,p/2,q;
%            0,0,0,0,0;
%            q,p/2,l,0,l;
%            p,q,l*2,l,0];
%           
% 
%Initialize velocity vector
dxi = zeros(2, N);
% 
% %State for leader
% state = 1;
% 
% These are gains for our formation control algorithm
formation_control_gain = 5;



%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.25, 'AngularVelocityLimit', 7.5);
% Single-integrator barrier certificates
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.22);
% Single-integrator position controller
si_pos_controller = create_si_position_controller();

%waypoints = 0.95*[1.3 0; -1.4 0;]';
%close_enough = 0.05;

theta = 0.7*pi/N;
R = [cos(theta), sin(theta); -sin(theta), cos(theta)];

for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    
    for i = 1:N
        dxi(:, i) = [0 ; 0];
        if i == N
            dxi(:, i) = R*(x(1:2,1) - x(1:2,N));
        else
            dxi(:, i) = R*(x(1:2,i+1) - x(1:2,i));
        end
%         if (i == 3)
%             %%Nothing here
%         else
%         %Zero velocity and get the topological neighbors of agent i
%         dxi(:, i) = [0 ; 0];
%         
%         neighbors = topological_neighbors(L, i);
%         
%         for j = neighbors
%             dxi(:, i) = dxi(:, i) + ...
%                 formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  weights(i,j)^2)*(x(1:2, j) - x(1:2, i));
%         end
%         end
    end
    
%     %% Make the leader travel between waypoints
%     
%     waypoint = waypoints(:, state);
%     
%     switch state        
%         case 1
%             dxi(:, 3) = si_pos_controller(x(1:2, 3), waypoint);
%             if(norm(x(1:2, 3) - waypoint) < close_enough)
%                 state = 2;
%             end
%         case 2
%             dxi(:, 3) = si_pos_controller(x(1:2, 3), waypoint);
%             if(norm(x(1:2, 3) - waypoint) < close_enough)
%                 state = 1;
%             end
%        
%     end
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxi = si_barrier_cert(dxi, x);
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %Iterate experiment
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
