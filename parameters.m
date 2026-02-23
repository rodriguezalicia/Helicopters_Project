% =================================================================
%       PARAMS - Parameters for Helicopter Rotor Analysis
% =================================================================

% ---------------------
%   Rotor Parameters
% ---------------------
% here we type the rotor parameters, such as number of blades, blade length, etc.
params.D = 11;                         % Diameter of the rotor [m]
params.R = params.D/2;                 % Radius of the rotor [m]
params.m = 3000;                       % Mass of the helicopter [kg]
params.n_blades = 4;                   % Number of blades
params.Omega = 340*pi/30;              % Rotational speed [rad/s]
params.theta_twist = -1.5;             % Twist slope [deg/m]
params.theta_t = params.theta_twist*pi/180*params.R; % Twist at the tip [rad]

% chord distribution (linear taper)
params.c = @(x) (0) .* (x >= 0 & x <= 0.05) + ...
         (2.*x - 0.1) .* (x > 0.05 & x <= 0.2) + ...
         (0.3) .* (x > 0.2 & x <= 1);
         
%params.c = @(x) 0.3 * ones(size(x)); % Constant chord distribution (uncomment this line to use constant chord)         

% ---------------------
%  Airfoil Parameters
% ---------------------
params.CL_alpha = 5.73;                % Lift curve slope [1/rad]      
params.delta0 = 0.0085;
params.delta1 = 0.263;
params.delta2 = 0.263;
params.x_npoints = 1000; % Number of points for the integration of Cpo

% ------------------
%   Air Parameters
% ------------------
% here we type the air parameters, such as density, viscosity, etc.
params.g = 9.80665; % gravitational acceleration (m/s^2)
params.rho = 1.225; % air density at sea level (kg/m^3)

% Calculate rest of parameters
params.S = pi*params.R^2;                           % Rotor disk area [m^2]
params.W = params.m*params.g;                       % Weight of the helicopter [N]

% --------------------------
%   Helicopter Parameters
% --------------------------
% Velocities vector for vertical flight
%   - We will analyze the performance of the rotor at different vertical velocities, from hover (0 m/s) to a climb rate of 5 m/s.
%   - Add more velocities if needed for a more detailed analysis.
params.Vz = [0; 5]; 
params.numVz = length(params.Vz);