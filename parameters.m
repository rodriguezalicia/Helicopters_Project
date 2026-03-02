% =================================================================
%       PARAMS - Parameters for Helicopter Rotor Analysis
% =================================================================

% ---------------------
%   Rotor Parameters
% ---------------------
% here we type the rotor parameters, such as number of blades, blade length, etc.
<<<<<<< HEAD
params.D = 11.5;                       % Diameter of the rotor [m]
=======
params.D = 7.67;                         % Diameter of the rotor [m]
>>>>>>> 8c40ee3c8dd8919b4e2b1079b63069b3015a0e6f
params.R = params.D/2;                 % Radius of the rotor [m]
params.m = 621;                       % Mass of the helicopter [kg]
params.n_blades = 2;                   % Number of blades
params.Omega = 530*pi/30;              % Rotational speed [rad/s]
params.theta_twist = -8; % Twist at the tip [rad]
params.theta_t = params.R*params.theta_twist*pi/180;

% chord distribution (linear taper)
params.c = @(x) 0.18 * ones(size(x)); % Constant chord distribution (uncomment this line to use constant chord)         

% ---------------------
%  Airfoil Parameters - NACA 63015
% ---------------------
airfoil_data = readmatrix('xf-n63015a-il-50000.csv');
airfoil_data(:,1) = deg2rad(airfoil_data(:,1));
cl_coeffs = polyfit(airfoil_data(:,1),airfoil_data(:,2),1);
cd_coeffs = polyfit(airfoil_data(:,1),airfoil_data(:,3),2);
params.CL_alpha = cl_coeffs(1);                % Lift curve slope [1/rad]      
params.delta0 = cd_coeffs(3);
params.delta1 = cd_coeffs(2);
params.delta2 = cd_coeffs(1);
params.x_npoints = 1000; % Number of points for the integration of Cpo

params.cd = @(x) params.delta0 + params.delta1*x + params.delta2.*x.^2;

% ------------------
%   Air Parameters
% ------------------
% here we type the air parameters, such as density, viscosity, etc.
params.altitudes = [0, 2000]; % 2 altitudes chosen for the hover study in [m]
params.naltitudes = length(params.altitudes);
[T, a, P, params.rho(2)] = atmosisa(params.altitudes(2));
params.rho(1) = 1.225; % air density at sea level (kg/m^3)
params.g = 9.80665; % gravitational acceleration (m/s^2)

% Calculate rest of parameters
params.S = pi*params.R^2;                           % Rotor disk area [m^2]
params.W = params.m*params.g;                       % Weight of the helicopter [N]

% --------------------------
%   Helicopter Parameters
% --------------------------
% Velocity for hover
params.Vz = 0; 