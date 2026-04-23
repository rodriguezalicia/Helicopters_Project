function Td = aero_module_simple (params, theta_0, theta_s, lambda_i)

% No twist, no taper and induced velocity from MT
    S_fp = params.S_fp;
    S = params.S;
    W = params.W;
    rho = params.rho;
    Omega = params.Omega;
    R = params.R;
    V = params.V;
    W = params.W;
    nb = params.n_blades;
    CL_alpha = params.CL_alpha;
    

Df = 1/2 * rho * V^2 * S_fp; 
alpha_d = -Df / W;

alpha_r = alpha_d; 
mu_x = (V / (Omega * R)) * cos(alpha_r); % Advance ratio 
lambda = (V / (Omega * R)) * sin(alpha_r) - lambda_i ; % Normal velocity parameter wind 

sigma = (nb * integral(@(x) params.c(x),0,1)); % Not needed


Td = nb *(1/4)*rho*integral(@(x) params.c(x),0,1)*CL_alpha *Omega^2 * R^3 * (mu_x*theta_s + theta_0*(2/3 + mu_x^2) + lambda);
end
