% Please note that input parameters should be the state data of the full-scale
% KVLCC2 tanker, which is 300,000 t, 320 m long. The output data is a matrix of
% the state parameters.

% For control, the maximum rudder angle is 35 degrees to each side.
% The trial speed of full-scale KVLCC2 is 7.98 m/s and
% the corresponding RPS is 1.54.
% The units of the input and output should be carefully checked.

clc
clear
close all

t_start_FS = 0;    % (s) Start time
t_step_FS  = 0.1;  % (s) Time step
t_end_FS   = 200; % (s) End time

RPS_FS_in       = 1.54; % (Hz)   Initial propeller revolution rate
delta_deg_FS_in = 35; % (deg)   Initial rudder angle

x0G_FS_in       = 0; % (m) Initial coordinate in x-direction
u_FS_in         = 7.98; % (m/s)   Initial velocity in x-direction
y0G_FS_in       = 0; % (m)     Initial coordinate in y-direction
v_FS_in         = 0; % (m/s)   Initial velocity in y-direction
psi_deg_FS_in   = 0; % (deg)   Initial heading angle around z-direction relative to north
r_deg_FS_in     = 0; % (deg/s) Initial turning rate around z-direction


output_FS_hist = simple_sim_without_control_FS(t_start_FS, t_step_FS, t_end_FS, ...
    RPS_FS_in,delta_deg_FS_in, ...
    x0G_FS_in,u_FS_in, ...
    y0G_FS_in,v_FS_in, ...
    psi_deg_FS_in,r_deg_FS_in);

% output_FS_hist has 9 columns. The columns of the output are as follows:
% t_FS_out: Time series
% RPS_FS_out Time series of propeller revolution per second.
% delta_deg_FS_out: Time series of of applied rudder angels in degree.
% x0G_FS_out: Time series of x coordinate in earth-fixed system.
% u_FS_out: Time series of speed in x direction.
% y0G_FS_out: Time series of of y coordinate in earth-fixed system.
% v_FS_out: Time series of speed in y direction.
% psi_deg_out: Time series of heading angle in degree.
% r_deg_out: Time series of of yaw rate in degree.


% For validation
xxx = dlmread('KVLCC2_Turning_star_35_NACA_0018_MARIN.txt','',2,0);

figure
hold on
plot(output_FS_hist(:,6)/320,output_FS_hist(:,4)/320,'b');
plot(xxx(:,4)/320,xxx(:,2)/320,'r');
axis equal
grid on
