function output_FS_hist = simple_sim_without_control_FS(t_start_FS, t_step_FS, t_end_FS, ...
    RPS_FS_in,delta_deg_FS_in, ...
    x0G_FS_in,u_FS_in, ...
    y0G_FS_in,v_FS_in, ...
    psi_deg_FS_in,r_deg_FS_in)
% simple_sim performs simulations

% RPS_FS_in       % (1/s) Input propeller revolution rate
% delta_deg_FS_in % (deg) Input rudder angle
% x0G_FS_in       % (m) Input coordinate in x-direction
% u_FS_in         % (m/s) Input velocity in x-direction
% y0G_FS_in       % (m) Input coordinate in y-direction
% v_FS_in         % (m/s) Input velocity in y-direction
% psi_deg_FS_in   % (rad) Input heading angle around z-direction relative to north
% r_deg_FS_in     % (rad/s) Input turning rate around z-direction


SC  = 45.714; % (-)

% ---------------------------------------------------------------------------- %
%% Simulation
% ---------------------------------------------------------------------------- %
nStep = (t_end_FS - t_start_FS)/t_step_FS + 1;
tSpan = linspace(t_start_FS,t_end_FS,nStep);

[t_hist_FS,state_hist_ms] = ode15s(@(tODE,yODE) ...
    simple_solver(tODE,yODE,RPS_FS_in* sqrt(SC),delta_deg_FS_in),...
    tSpan, ...
    [x0G_FS_in / SC, ...
    u_FS_in / sqrt(SC), ...
    y0G_FS_in / SC, ...
    v_FS_in / sqrt(SC), ...
    deg2rad(psi_deg_FS_in), ...
    deg2rad(r_deg_FS_in) * sqrt(SC)]);

state_hist_FS = [state_hist_ms(:,1) * SC, ... & x0G
    state_hist_ms(:,2) * sqrt(SC), ... % u
    state_hist_ms(:,3) * SC, ... % y0G
    state_hist_ms(:,4) * sqrt(SC), ... % v
    rad2deg(state_hist_ms(:,5)), ... % psi
    rad2deg(state_hist_ms(:,6)) / sqrt(SC)]; % r

data_length = length(t_hist_FS);

delta_deg_FS_in_hist = delta_deg_FS_in * ones(data_length,1);

RPS_FS_in_hist = RPS_FS_in * ones(data_length,1);

output_FS_hist = [t_hist_FS,RPS_FS_in_hist,delta_deg_FS_in_hist,state_hist_FS];

end

function yODEDerive = simple_solver(~,yODE,RPS_FS_in,delta_deg_FS_in)
% Calculate the derivatives of the motion state quantities on midpoint

% x0G_FS_in       % (m) Input coordinate in x-direction
% u_FS_in         % (m/s) Input velocity in x-direction
% y0G_FS_in       % (m) Input coordinate in y-direction
% v_FS_in         % (m/s) Input velocity in y-direction
% psi_deg_FS_in   % (rad) Input heading angle around z-direction relative to north
% r_deg_FS_in     % (rad/s) Input turning rate around z-direction
% RPS_FS_in       % (1/s) Input propeller revolution rate
% delta_deg_FS_in % (deg) Input rudder angle

% Model - scale KVLCC2 (MARIN free running tests)
% Lpp = 7.0 (m)
% Reference:
% SIMMAN 2014 http://www.simman2014.dk/cms/site.aspx?p=13327
% Lee, S.W., Toxopeus, S.L., Quadvlieg, F.H.H.A., 2007. Free Sailing Manoeuvring
% Tests on KVLCC1 and KVLCC2. Wageningen, The Netherlands.

% serviceSpeed = 1.179;  % (m/s)
% serviceRPS   = 10.4;   % [1/s]
% serviceTrMaxDeg  = 15.8;   % (deg/s) ruder turning rate
%
% ---------------------------------------------------------------------------- %
%% Environment
% ---------------------------------------------------------------------------- %
%
rho = 1025; %(kgm^3) Water density
%
% ---------------------------------------------------------------------------- %
% Hull
% ---------------------------------------------------------------------------- %
%
Lpp   = 7.0;     % (m)   length between perpendiculars

Bwl   = 1.1688;  % (m)   beam at waterline
DispV = 3.2724;  % (m^3) ship volume displacement

Tm   = 0.455; % (m) mean draught

% B2L = Bwl/Lpp; % (-) Breadth - length ratio

xG  = 0.244; % (m) xG = LCG = LCB*LPP w.r.t. MID

% Cb = 0.8098; % (-) Block coefficient

R0_non = -0.022;  % (-)

% XH
Xvv_non   = -0.040;  % (-)
Xvr_non   = 0.002;   % (-)
Xrr_non   = 0.011;   % (-)
Xvvvv_non = 0.771;   % (-)

% YH, NH
Yv_non   = -0.315;  % (-)
Yr_non   = 0.083;   % (-)
Yvvv_non = -1.607;  % (-)
Yvvr_non = 0.379;   % (-)
Yvrr_non = -0.391;  % (-)
Yrrr_non = 0.008;   % (-)
Nv_non   = -0.137;  % (-)
Nr_non   = -0.049;  % (-)
Nvvv_non = -0.030;  % (-)
Nvvr_non = -0.294;  % (-)
Nvrr_non = 0.055;   % (-)
Nrrr_non = -0.013;  % (-)
%
% ---------------------------------------------------------------------------- %
%% Propeller
% ---------------------------------------------------------------------------- %
%

DProp    = 0.216; % (m) Propeller diameter

KtPoly = [-0.1385,-0.2753,0.2931]; % [Kt2,Kt1,Kt0]

xProp_non = -0.48; % (-)
yProp_non = 0.00;  % (-)
yProp = yProp_non*Bwl; % (m)

w0Prop = 0.4;
t0Prop = 0.220;
%
% ---------------------------------------------------------------------------- %
% Rudder
% ---------------------------------------------------------------------------- %
%
SRudd     = 0.0539; % (m^2) Lateral area of the movable part

BRudd     = 0.3455; % (m)   Rudder span

ArGeoRudd = 1.827;  % (-)   Geometric aspect ratio, B/C

xRudd_non     = -0.5; % (-)
xRudd          = xRudd_non*Lpp; % (m)
yRudd_non     = 0.0;  % (-)
yRudd          = yRudd_non*Lpp; % (m)

tRudd          = 0.387;
ellRudd_non   = -0.710;
kappaRudd      = 0.5;
aHRudd         = 0.312;
xHRudd_non    = -0.464;
Lambda_K       = ArGeoRudd/(ArGeoRudd + 2.25);
epsilon_wRwP   = 1.09;
DProp2BRudd    = DProp/BRudd;
xHRudd         = xHRudd_non*Lpp;

dCN2dSinAOA_Inf_CFDPoly = [5.94454,0.0112];

%
% ---------------------------------------------------------------------------- %
%% Mass and moment
% ---------------------------------------------------------------------------- %
%
massFactor    = 0.5*rho*Lpp^2*Tm;
inertiaFactor = 0.5*rho*Lpp^4*Tm;

m = DispV*rho; % [kg]

mx_non = 0.02192; % (-)
my_non = 0.22290; % (-)
Jz_non = 0.01060; % (-) added moment J_z

mx = mx_non*massFactor; % [kg]
my = my_non*massFactor; % [kg]
Jz = Jz_non*inertiaFactor;  % [kgm^2]

iz_non = 0.25; % (-) the radius of yaw gyration
iz = iz_non*Lpp; % (m)
Iz = m*iz^2; % [kg*m^2]

delta = deg2rad(delta_deg_FS_in); % (rad) Current rudder angle

% x0G = yODE(1); % (m)     in earth-fixed coordinate systems
u     = yODE(2); % (m/s)   w.r.t. midship
% y0G = yODE(3); % (m)     in earth-fixed coordinate systems
v     = yODE(4); % (m/s)   w.r.t. midship
psi   = yODE(5); % (rad)   w.r.t. midship
r     = yODE(6); % (rad/s) w.r.t. midship

V      = sqrt(u^2 + v^2); % (m/s) resultant speed w.r.t. MID
% u_non = u/V;  % (-)
v_non = v / V;  % (-)
r_non = r * Lpp / V; % (-) dimensionless yaw rate
beta   = atan(-v/u); % (rad) drift angle w.r.t. MID

forceFactor  = 0.5*rho*Lpp*Tm*V^2;
momentFactor = 0.5*rho*Lpp^2*Tm*V^2;
% ---------------------------------------------------------------------------- %
%% Hull forces and moments
% ---------------------------------------------------------------------------- %

% Total resistance Rtot in straight moving
XHullR0 = R0_non*forceFactor; % (N)

XHull = forceFactor*(Xvv_non*v_non^2 + ...
    Xvr_non*v_non*r_non + ...
    Xrr_non*r_non^2 + ...
    Xvvvv_non*v_non^4) ...
    + XHullR0; % (N)

YHull_non = Yv_non*v_non + ...
    Yr_non*r_non + ...
    Yvvv_non*v_non^3 + ...
    Yvvr_non*v_non^2*r_non + ...
    Yvrr_non*v_non*r_non^2 + ...
    Yrrr_non*r_non^3; % (-)
YHull = YHull_non*forceFactor; % (N)

NHull_non = Nv_non*v_non + ...
    Nr_non*r_non + ...
    Nvvv_non*v_non^3 + ...
    Nvvr_non*v_non^2*r_non + ...
    Nvrr_non*v_non*r_non^2 + ...
    Nrrr_non*r_non^3; % (-)
NHull = NHull_non*momentFactor; %(Nm^2)
% ---------------------------------------------------------------------------- %
%% Propeller forces and moments
% ---------------------------------------------------------------------------- %
betaProp = beta - xProp_non*r_non; % (rad)

C1 = 2.0;
if betaProp > 0
    C2 = 1.6;
else
    C2 = 1.1;
end

wProp = 1 - (1 - w0Prop)*(1 + (1 - exp(-C1*abs(betaProp)))*(C2 - 1));

uProp = u*(1 - wProp);
JProp = uProp/(RPS_FS_in*DProp);
KtProp = polyval(KtPoly,JProp);
TProp = rho*RPS_FS_in^2*DProp^4*KtProp; % (N)

XProp = (1 - t0Prop)*TProp; % (N)
YProp = 0;
NProp = -yProp*XProp;
% ---------------------------------------------------------------------------- %
%% Rudder forces and moments
% ---------------------------------------------------------------------------- %
wRudd = 1 - epsilon_wRwP*(1 - wProp);

betaRudd = beta - ellRudd_non*r_non;

if betaRudd < 0
    gammaRudd = 0.395;
else
    gammaRudd = 0.640;
end

vRudd = V*gammaRudd*betaRudd;
uRudd = (1 - wRudd)*u ...
    *sqrt(DProp2BRudd*(1 + kappaRudd*(sqrt(1 ...
    + 8*KtProp/(pi*JProp^2)) - 1))^2 + 1 - DProp2BRudd);
VRudd = sqrt(uRudd^2 + vRudd^2);

delta_eff = delta - atan(vRudd/uRudd);

CNRudd = round(dCN2dSinAOA_Inf_CFDPoly(1),3) * sin(delta_eff) * Lambda_K;

% delta_hydro = atan(vRudd/uRudd);

FNRudd = 0.5 * rho * SRudd * VRudd ^ 2 * CNRudd; % (N) Normal force
FXRudd = FNRudd * sin(delta);
FYRudd = FNRudd * cos(delta);

XRudd = -(1 - tRudd)*FXRudd;
YRudd = -(1 + aHRudd)*FYRudd;
NRudd = -(xRudd + aHRudd*xHRudd)*FYRudd + yRudd*(1 - tRudd)*FXRudd;
%
% ---------------------------------------------------------------------------- %
%% Solve
% ---------------------------------------------------------------------------- %
%
XTotal = XHull + XProp + XRudd;
YTotal = YHull + YProp + YRudd;
NTotal = NHull + NProp + NRudd;

% X = (m + mx)*uDot - (m + my)*v*r - m*xG*r^2
% Y = (m + my)*vDot + (m + mx)*u*r +m*xG*rDot
% N = (Izz + xG^2*m + Jz)*rDot + m*xG*(vDot + u*r)

M22 = m + mx;
M23 = -(m + my)*r;
M25 = -m*xG*r;
M44 = m + my;
M45 = (m + mx)*u;
M46 = m*xG;
M64 = m*xG;
M65 = m*xG*u;
M66 = Iz + Jz + xG^2*m;

Mass = [1,   0,   0,   0,   0,   0;    % u      = u
        0,   M22, M23, 0,   M25, 0;    % uDot   = XTotal
        0,   0,   1,   0,   0,   0;    % v      = v
        0,   0,   0,   M44, M45, M46;  % vDot   = YTotal
        0,   0,   0,   0,   1,   0;    % r      = r
        0,   0,   0,   M64, M65, M66]; % rShipDot   = NTotal

temp = Mass\[u;XTotal;v;YTotal;r;NTotal];

% Change the speed
uG = u;
vG = v + r*xG;

u0G = uG*cos(psi) - vG*sin(psi);
v0G = uG*sin(psi) + vG*cos(psi);

yODEDerive = [u0G;temp(2);v0G;temp(4);temp(5);temp(6)];

end
