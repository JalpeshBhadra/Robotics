function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
kdx = 100
kpx = 10
kdy  = 100
kpy = 10
kdz = 1
kpz = 10
kdp=1
kpp=5
kdt=1
kpt=5
kds=1
kps=5


F = (params.mass*params.gravity) + params.mass*(des_state.acc(3) + (kdz*(-state.vel(3)+des_state.vel(3))) + kpz*(-state.pos(3) + des_state.pos(3)));
x = (des_state.acc(1) + (kdx*(-state.vel(1)+des_state.vel(1))) + kpx*(-state.pos(1) + des_state.pos(1)));
y = (des_state.acc(1) + (kdy*(-state.vel(2)+des_state.vel(2))) + kpy*(-state.pos(2) + des_state.pos(2)));

phi = (1/params.gravity)*(x*sin(des_state.yaw) - y*cos(des_state.yaw) );

theta = (1/params.gravity)*(x*cos(des_state.yaw) + y*sin(des_state.yaw) );

M =[ kpp*(phi-state.rot(1)) + kdp*(-state.omega(1));
     kpt*(theta-state.rot(2)) + kdt*(-state.omega(2));
     kps*(des_state.yaw-state.rot(3)) + kds*(des_state.yawdot - state.omega(3))];
% =================== Your code ends here ===================

end
