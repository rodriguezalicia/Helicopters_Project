function [Td,Qd,Yd] = aero_module_simple (params, theta_0, theta_s, theta_c, beta_0, beta_s, beta_c, lambda_i)

% No twist, no taper and induced velocity from MT
    S_fp = params.S_fp;
    S = params.S;
    rho = params.rho;
    Omega = params.Omega;
    R = params.R;
    ecc = params.ecc;
    V = params.V;
    W = params.W;
    nb = params.n_blades;
    

Df = 1/2 * rho * V^2 * S_fp; 
alpha_d = -Df / W;

alpha_r = alpha_d; 
mu_x = (V / (Omega * R)) * cos(alpha_r); % Advance ratio 
lambda = (V / (Omega * R)) * sin(alpha_r) - lambda_i ; % Normal velocity parameter wind 
sigma = (nb * integral(@(x) params.c(x),0,1)); % Not needed

n_points = 100;
x_points = linspace(params.xroot,1,n_points);
psi_points = linspace(0,2*pi(),n_points);
[x,psi] = meshgrid(x_points,psi_points);

beta = beta_0 + beta_s*sin(psi) + beta_c*cos(psi);
dbeta_dpsi = beta_s*cos(psi) - beta_c*sin(psi); 
theta = theta_0 + theta_c*cos(psi) + theta_s*sin(psi) + params.theta_t*x;

UT_nondim = -mu_x*sin(psi) - x;
UP_nondim = -mu_x*beta*cos(psi) + lambda - dbeta_dpsi*(x - ecc/R);
phis = atan(-UP_nondim/UT_nondim);

alphas = theta + phis;
Cds = params.delta0 + params.delta1.*alphas + params.delta2.*alphas.^2;
Cls = params.CL_alpha.*alphas;

dCT = 0.5*sigma.*Cls.*UT_nondim.^2;
dCQ = 0.5*sigma.*(phis.*Cls - Cds).*UT_nondim.^2.*x;
dCY = 0.5*sigma.*(phis.*Cls - Cds).*UT_nondim.^2.*cos(psi);

dCT_rad = trapz(x_points,dCT,2);
dCQ_rad = trapz(x_points,dCQ,2);
dCY_rad = trapz(x_points,dCY,2);

CT = trapz(psi_points,dCT_rad);
CQ = trapz(psi_points,dCQ_rad);
CY = trapz(psi_points,dCY_rad);

Td = CT*rho*S*(Omega*R)^2;
Qd = CQ*rho*S*(Omega*R)^2*R;
Yd = CY*rho*S*(Omega*R)^2;

end
