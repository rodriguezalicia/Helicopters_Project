% =================================================================
%       PARAMS - Parameters for Helicopter Rotor Analysis
% =================================================================

% ---------------------
%   Rotor Parameters
% ---------------------
% here we type the rotor parameters, such as number of blades, blade length, etc.
D = 11;                         % Diameter of the rotor [m]
R = D/2;                        % Radius of the rotor [m]
m = 3000;                       % Mass of the helicopter [kg]
n_blades = 4;                   % Number of blades
Omega = 340*pi/30;              % Rotational speed [rad/s]
theta_twist = -1.5;             % Twist slope [deg/m]
theta_t = theta_twist*pi/180*R; % Twist at the tip [rad]

% chord distribution (linear taper)
c = @(x) (0) .* (x >= 0 & x <= 0.05) + ...
         (2.*x - 0.1) .* (x > 0.05 & x <= 0.2) + ...
         (0.3) .* (x > 0.2 & x <= 1);

% Calculate rest of parameters
S = pi*R^2;                    % Rotor disk area [m^2]
W = m*g;                       % Weight of the helicopter [N]

% ------------------
%   Air Parameters
% ------------------
% here we type the air parameters, such as density, viscosity, etc.
g = 9.80665; % gravitational acceleration (m/s^2)
rho = 1.225; % air density at sea level (kg/m^3)

% --------------------------
%   Helicopter Parameters
% --------------------------
% Velocities vector for vertical flight
%   - We will analyze the performance of the rotor at different vertical velocities, from hover (0 m/s) to a climb rate of 5 m/s.
%   - Add more velocities if needed for a more detailed analysis.
Vz = [0; 5]; 