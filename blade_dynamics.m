function [beta_0, theta_s, theta_c] = blade_dynamics(params, theta_0, beta_c, beta_s)

W = params.W;
m_blade = params.m_blade;
Omega = params.Omega;
R = params.R;
rho = params.rho;
V = params.V;
S = params.S;
alpha_r = params.alpha_r;
nb = params.n_blades;
Iy = params.Iy;

    % Glauert iterative solution
    vi0    = sqrt(W / (2 * rho * S));
    Vz     = -V * sin(alpha_r);
    Vx     =  V * cos(alpha_r);
    Vz_nd  = Vz / vi0;
    Vx_nd  = Vx / vi0;

    fun    = @(vi_nd) vi_nd * sqrt(Vx_nd^2 + (Vz_nd + vi_nd)^2) - 1;
    vi_nd  = fsolve(fun, 1, optimset('Display','off'));
    vi     = vi_nd * vi0;

    mu_x     = Vx / (Omega * R);           
    lambda_x = Vz / (Omega * R);           
    lambda_i = vi  / (Omega * R);          

 
           
    gamma = rho * params.CL_alpha * integral(@(x) params.c(x),0,1) * R^4 / Iy;



% Same system as in Trim
% A_matrix * [beta_0; beta_s; beta_c] 
A = [1,             0,                     0;
            1/3*mu_x,      1/8*mu_x^2 + 1/4,      0;
            0,             0,                     1/8*mu_x^2 - 1/4];

% B_matrix * [theta_0; theta_s; theta_c] 

B =  [(gamma/8)*(mu_x^2 + 1),  (gamma/6)*mu_x,              0;
                    0,                 0,                     1/8*(mu_x^2 + 2);
                    2/3*mu_x,          1/8*(3*mu_x^2 + 2),      0];


% Inflow components 
inflow_wind =  [gamma *(lambda_x / 6);  0; (lambda_x * mu_x) / 2];

inflow_induced =  [gamma *(lambda_i/6); 0; (mu_x * lambda_i) / 2];


    % Solve system
    
    % theta_s
   
    theta_s = (A(3,3)*beta_c - B(3,1)*theta_0 - inflow_wind(3) + inflow_induced(3)) / B(3,2);

    %t beta_0
    
    beta_0 = (B(1,1)*theta_0 + B(1,2)*theta_s + inflow_wind(1) - inflow_induced(1)) / A(1,1);

    % theta_c
   
    theta_c = (A(2,1)*beta_0 + A(2,2)*beta_s) / B(2,3);

end
