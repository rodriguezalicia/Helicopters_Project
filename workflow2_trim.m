function results = workflow2_trim(params, results_MT, theta_0_init)

   
    % Step 1: Trim force/moment equilibrium (no controls needed)
   
    T_D     = params.W;
    Df      = 0.5 * params.rho * params.V^2 * params.S_fp;
    alpha_D = -Df / params.W;

    delta_x = 0.15;            
    delta_z = 1.5;              
    beta_c  = atan(delta_x / delta_z);  
    beta_s  = 0;                         

    % MT hover lambda_i , not needed
    lambda_i_MT = results_MT.vi / (params.Omega * params.R);

  
    % Step 2 - Iterative loop: vary theta_0 until T_aero = T_D
    % First guess of theta_0 has to be introduced in the main
    
    residual_fun = @(th0) thrust_residual(params, th0, beta_c, beta_s, T_D);


    % Parameters for the solver
    opt = optimset('Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-8);
    [theta_0_sol, fval, flag] = fsolve(residual_fun, theta_0_init, opt);

  
    % Step 3 : Calculations with the converged theta 0
    
    [beta_0_sol, theta_s_sol, theta_c_sol, lambda_i_glauert] = ...
        blade_dynamics(params, theta_0_sol, beta_c, beta_s);

    T_aero_sol = aero_module_simple(params, theta_0_sol, theta_s_sol, lambda_i_glauert);

   
    results.theta_0   = theta_0_sol;
    results.theta_s   = theta_s_sol;
    results.theta_c   = theta_c_sol;
    results.beta_0    = beta_0_sol;
    results.beta_c    = beta_c;
    results.beta_s    = beta_s;
    results.alpha_D   = alpha_D;
    results.T_D       = T_D;
    results.T_aero    = T_aero_sol;
    results.lambda_i  = lambda_i_glauert;   % Glauert 
    results.lambda_i_MT = lambda_i_MT;      % MT hover 
    results.converged = (flag > 0);
    results.residual  = abs(fval);

end


% Function for the difference between T from trim and T_d
% The aero module receives the lambda i compted in the blade dynamics using
% glauert
function err = thrust_residual(params, theta_0, beta_c, beta_s, T_D_ref)
    [~, theta_s, ~, lambda_i] = blade_dynamics(params, theta_0, beta_c, beta_s);
    T_aero = aero_module_simple(params, theta_0, theta_s, lambda_i);
    err = T_aero - T_D_ref;
end