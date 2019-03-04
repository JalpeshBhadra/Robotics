function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent coef_a coef_b coef_c waypoints0 traj_time d0
if nargin > 2
    % setup trajectory segment times
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    % solve for coefficients in x,y,z
    coef_a = getCoef(waypoints0(1,1:end)');
    coef_b = getCoef(waypoints0(2,1:end)');
    coef_c = getCoef(waypoints0(3,1:end)');
    
else
    % provide the trajectory point based on the coefficients
    
    if(t > traj_time(end))
        t = traj_time(end) ;
    end
    
    t_index = find(traj_time >= t,1)-1;
    
    if (t_index == 0)
        t_index = 1;
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = 0*waypoints0(:,1);
        desired_state.acc = 0*waypoints0(:,1);
    else
       
        scale = (t-traj_time(t_index))/d0(t_index);
        
        index = (t_index-1)*8+1:t_index*8;
        
        
        t0 = polyT(8,0,scale)';
        desired_state.pos = [coef_a(index)'*t0; coef_b(index)'*t0; coef_c(index)'*t0];
        
        
        t1 = polyT(8,1,scale)';
        desired_state.vel = [coef_a(index)'*t1; coef_b(index)'*t1; coef_c(index)'*t1].*(1/d0(t_index));
        
        
        t2 = polyT(8,2,scale)';
        desired_state.acc = [coef_a(index)'*t2; coef_b(index)'*t2; coef_c(index)'*t2].*(1/d0(t_index)^2);
    end
    
   
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end

function [T] = polyT(m, q, t)

T = zeros(m,1);
D = zeros(m,1);

for l=1:m
    D(l) = l-1;
    T(l) = 1;
end

% Derivative:
for j=1:q
    for l=1:m
        T(l) = T(l) * D(l);
        if D(l) > 0
            D(l) = D(l) - 1;
        end
    end
end


for l=1:m
    T(l) = T(l) * t^D(l);
end

T = T';

end

function [coef, Z, y] = getCoef(waypoints)


n = size(waypoints,1)-1;


y = zeros(1,8*n);
for i=1:n
    y(1,i) = waypoints(i);
    y(1,i+n) = waypoints(i+1);
end

Z=zeros(8*n,8*n);


for i=1:n
    Z(i,((i-1)*8)+1:i*8) = polyT(8,0,0);
end


for i=1:n
    Z(i+n,((i-1)*8)+1:i*8) = polyT(8,0,1);
end


for k=1:3
    Z(2*n+k,1:8) = polyT(8,k,0);
end


for k=1:3
    Z(2*n+3+k,(end-7):end) = polyT(8,k,1);
end


for i=2:n
    for k=1:6
        Z(2*n+6+(i-2)*6+k, (i-2)*8+1:((i-2)*8+n*n)) = [polyT(8,k,1) -polyT(8,k,0)];
    end
end

coef = Z\y';

end