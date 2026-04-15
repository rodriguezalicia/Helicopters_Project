%% TRIM %%
function results = trim(params, results_MT)
 
params.S_fp = 2.8 * 0.0929; % Convertir a m2. Airframe equivalent flat plate area [ft^2] - Paper de Maryland

% 1. Force Equilibrium and Rotor Tilting
Td = params.W; 

% Ajustar esta velocidad en el main

params.V = 50;

% params.S_fp = 2.8; % Airframe equivalent flat plate area [ft^2] - Paper de Maryland
Df = 1/2 * params.rho * params.V^2 * params.S_fp; % Fuselage Drag 
alpha_d = -Df / params.W; % Required rotor disk tilt 

% Center of Mass offsets (using the typical ratio from the slides) 
delta_x = 0.15; % distance in x from shaft to CG 
delta_z = 1.5;  % distance in z from shaft to CG 

% Longitudinal flapping to ensure moment equilibrium (rotor axis passes through CG) 
beta_c = atan(delta_x / delta_z);% [cite: 309]
beta_s = 0; % Assuming lateral trim is neglected (symmetric flight)

% 2. Flow Parameters
% As a starting approximation, assume shaft angle alpha_r is close to alpha_d
alpha_r = alpha_d; 
mu_x = (params.V / (params.Omega * params.R)) * cos(alpha_r); % Advance ratio 
lambda_x = (params.V / (params.Omega * params.R)) * sin(alpha_r); % Normal velocity parameter wind 

% Lock number
gamma = (params.rho * params.CL_alpha * integral(@(x) params.c(x),0,1) * params.R^4) / params.Iy;

% Assume uniform induced velocity v_i (can be taken from hover MT as an entry-level approximation)
% v_i = ... (needs to be calculated or passed in params)
lambda_i = results_MT.vi / (params.Omega * params.R);

% 3. Blade Dynamics System 
% A_matrix * [beta_0; beta_s; beta_c] 
A_matrix = [1,             0,                     0;
            1/3*mu_x,      1/8*mu_x^2 + 1/4,      0;
            0,             0,                     1/8*mu_x^2 - 1/4];

% B_matrix * [theta_0; theta_s; theta_c] 

B_matrix =  [(gamma/8)*(mu_x^2 + 1),  (gamma/6)*mu_x,              0;
                    0,                 0,                     1/8*(mu_x^2 + 2);
                    2/3*mu_x,          1/8*(3*mu_x^2 + 2),      0];


% Inflow components 
inflow_wind =  [gamma *(lambda_x / 6);  0; (lambda_x * mu_x) / 2];

inflow_induced =  [gamma *(lambda_i/6); 0; (mu_x * lambda_i) / 2];


% Thrust equation

% T_D = params.n_blades *(1/4)* params.rho*integral(@(x) params.c(x),0,1)*params.CL_alpha * params.Omega^2 * params.R^3 * (mu_x*theta_s + theta_0*(2/3 + mu_x^2) + lambda_x - lambda_i);

% Solve the 4x4 system
M = zeros(4,4);
Y = zeros(4,1);


% Eq 1
M(1,1) = A_matrix(1,1);   % beta_0
M(1,2) = -B_matrix(1,1);  % theta_0
M(1,3) = -B_matrix(1,2);  % theta_s
M(1,4) = -B_matrix(1,3);  % theta_c
Y(1)   = inflow_wind(1) - inflow_induced(1);

% Eq 2
M(2,1) = A_matrix(2,1);   
M(2,2) = -B_matrix(2,1);  
M(2,3) = -B_matrix(2,2);  
M(2,4) = -B_matrix(2,3); 
Y(2)   = inflow_wind(2) - inflow_induced(2) - A_matrix(2,2)*beta_s;

% Eq 3
M(3,1) = A_matrix(3,1);   
M(3,2) = -B_matrix(3,1);  
M(3,3) = -B_matrix(3,2);  
M(3,4) = -B_matrix(3,3);  
Y(3)   = inflow_wind(3) - inflow_induced(3) - A_matrix(3,3)*beta_c;

% eq 4
M(4,1) = 0;                             % beta_0 no afecta 
M(4,2) = params.n_blades * (1/4) * params.rho * integral(@(x) params.c(x),0,1) * params.CL_alpha * params.Omega^2 * params.R^3 * (2/3 + mu_x^2);         
M(4,3) = params.n_blades * (1/4) * params.rho * integral(@(x) params.c(x),0,1) * params.CL_alpha * params.Omega^2 * params.R^3 * mu_x;                    
M(4,4) = 0;                             % theta_c no afecta al empuje longitudinal
Y(4)   = Td - params.n_blades * (1/4) * params.rho * integral(@(x) params.c(x),0,1) * params.CL_alpha * params.Omega^2 * params.R^3*(lambda_x - lambda_i);

sol = M \ Y;

results.beta_0  = sol(1);
results.theta_0 = sol(2);
results.theta_s = sol(3);
results.theta_c = sol(4);

results.alpha_d = alpha_d;
results.beta_c = beta_c;
results.beta_s = beta_s;
results.thrust = Td;

end
