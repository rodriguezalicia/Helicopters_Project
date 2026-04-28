% =================================================================
%       PARAMS - Parameters for Helicopter Rotor Analysis
% =================================================================

% ---------------------
%   Rotor Parameters
% ---------------------
% here we type the rotor parameters, such as number of blades, blade length, etc.
params.D = 7.67;                       % Diameter of the rotor [m]
params.R = params.D/2;                 % Radius of the rotor [m]
params.m = 621;                        % MTOM of the helicopter [kg]
params.n_blades = 2;                   % Number of blades
params.Omega = 530*pi/30;              % Rotational speed [rad/s]
params.theta_t = deg2rad(-8); % Twist at the tip / unit rate of twist [rad]
params.m_blade = 11.79;                % Mass of a single blade
params.S_fp = 2.8 * 0.0929;
params.xroot = 0.55/11.8;
params.ecc = params.xroot*params.R;
params.deltaX = 0.0889;
params.deltaY = 0.026;
params.deltaZ = 1.75/2;
params.R_tail_rotor = 1.07/2;
params.L_tail_rotor = params.R + params.R_tail_rotor;

% chord distribution (linear taper)
%params.c = @(x) 0.183 * ones(size(x)); % Constant chord distribution (uncomment this line to use constant chord)   

params.c = @(x) (0) .* (x >= 0 & x <= params.xroot) + ...
         (0.2*0.183/0.6 + (11.8*(0.183 - 0.2*0.183/0.6)/(0.75 - 0.55)).*(x - params.xroot)) .* (x > params.xroot & x <= 0.75/11.8) + ...
         (0.183) .* (x > 0.75/11.8 & x <= 1);

% ---------------------
%  Airfoil Parameters - NACA 63015
% ---------------------
airfoil_data = readmatrix('xf-n63015a-il-1000000.csv'); % Use Re = 1e6 instead of 10K because its closer to our case
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
params.g = 9.80665; % gravitational acceleration (m/s^2)

% Calculate rest of parameters
params.S = pi*params.R^2;                           % Rotor disk area [m^2]
params.W = params.m*params.g;                       % Weight of the helicopter [N]
params.Iy = params.m_blade*params.R^2 / 4 + (params.m_blade / (params.R * integral(@(x) params.c(x),0,1)))* integral(@(x) params.c(x),0,1)*(params.R^3 / 12);
params.V = 50; % ajustar según queramos
