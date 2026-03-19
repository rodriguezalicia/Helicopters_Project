% =================================================================
%       PARAMS - Parameters for Helicopter Rotor Analysis
% =================================================================

% ------------------
%   Rotor Parameters
% ------------------
% here we type the rotor parameters, such as number of blades, blade length, etc.
params.D = 7.67;                         % Diameter of the rotor [m]
params.R = params.D/2;                 % Radius of the rotor [m]
params.m = 621;                       % MTOM of the helicopter [kg]
params.n_blades = 2;                   % Number of blades
params.Omega = 530*pi/30;              % Rotational speed [rad/s]
params.theta_t = deg2rad(-8); % Twist at the tip / unit rate of twist [rad]

% chord distribution (linear taper)
params.c = @(x) 0.18 * ones(size(x)); % Constant chord distribution (uncomment this line to use constant chord)         

% --------------------------------
%  Airfoil Parameters - NACA 63015
% --------------------------------
airfoil_data = readmatrix('xf-n63015a-il-1000000.csv'); % Use Re = 1e6 instead of 10K because its closer to our case
airfoil_data(:,1) = deg2rad(airfoil_data(:,1));
cl_coeffs = polyfit(airfoil_data(:,1),airfoil_data(:,2),1);
cd_coeffs = polyfit(airfoil_data(:,1),airfoil_data(:,3),2);
params.CL_alpha = cl_coeffs(1);                % Lift curve slope [1/rad]      
params.delta0 = cd_coeffs(3);
params.delta1 = cd_coeffs(2);
params.delta2 = cd_coeffs(1);
params.x_npoints = 1000; % Number of points for the integration of Cpo

params.cd = @(alpha) params.delta0 + params.delta1*alpha + params.delta2.*alpha.^2;

% -------------------------------
%   General Helicopter Parameters
% -------------------------------
params.g = 9.80665; % gravitational acceleration (m/s^2)

% Calculate rest of parameters
params.S = pi*params.R^2;                           % Rotor disk area [m^2]
params.W = params.m*params.g;                       % Weight of the helicopter [N]
