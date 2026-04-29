function results = workflow2_trim(params, results_MT, theta_0_init)
% Renaming parameters for better clarity
T_D     = params.W;
Df      = 0.5 * params.rho * params.V^2 * params.S_fp; % Fuselage drag
alpha_D = -Df / params.W;
delta_x = params.deltaX;
delta_y = params.deltaY;
delta_z = params.deltaZ;
beta_c  = atan(delta_x / delta_z);
beta_s  = atan(delta_y / delta_z);
phi_guess = 0;

lambda_i_MT = results_MT.vi / (params.Omega * params.R);

% Iterative loop: vary theta_0 until beta_s coincides
% First guess of beta_s has to be introduced from the main

lat_force_err = @(beta_s_guess) thrust_residual(params, theta_0_init, beta_c, beta_s_guess, phi_guess);
beta_s_init_guess = beta_s; % Assigning the initial guess for beta_s from the main
opt = optimset('Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-8);
[beta_s, fval, flag] = fsolve(lat_force_err, beta_s_init_guess, opt); % Solving to obtain the iteration for beta_s

if flag<0 % Checking if the solution is converged
    fprintf('Lateral trim force error did not converge')
end

[beta_0_sol, theta_s_sol, theta_c_sol, lambda_i_glauert] = blade_dynamics(params, theta_0_init, beta_c, beta_s);
[Td,Qd,Yd] = aero_module_simple (params, theta_0_init, theta_s_sol, theta_c_sol, beta_0_sol, beta_s, beta_c, lambda_i_glauert);

results.theta_0     = theta_0_init;
results.theta_s     = theta_s_sol;
results.theta_c     = theta_c_sol;
results.beta_0      = beta_0_sol;
results.beta_s      = beta_s;
results.beta_c      = beta_c;
results.beta_s      = beta_s;
results.alpha_D     = alpha_D;
results.T_D         = T_D;
results.Td          = Td;
results.Qd          = Qd;
results.Yd          = Yd;
results.lambda_i    = lambda_i_glauert;   
results.lambda_i_MT = lambda_i_MT;      
results.converged   = (flag > 0);
results.residual    = abs(fval);

end