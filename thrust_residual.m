function err = thrust_residual(params, theta_0, beta_c, beta_s, phi_guess)
[beta_0, theta_s, theta_c, lambda_i] = blade_dynamics(params, theta_0, beta_c, beta_s);
[Td,Qd,Yd] = aero_module_simple (params, theta_0, theta_s, theta_c, beta_0, beta_s, beta_c, lambda_i);

T_tail_rotor_thr = Qd/params.L_tail_rotor; % Calculating the thurst the tail rotor must produce to compensate for main rotor moment
err = Yd - (beta_s*Td) + T_tail_rotor_thr + params.W*sin(phi_guess); % Calculating the error
end