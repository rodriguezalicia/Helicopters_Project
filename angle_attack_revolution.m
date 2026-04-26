function angle_attack_revolution(params, results_wf2, h)

% We define alpha as alpha = theta + phi
% theta = theta0 + theta_c + theta_twist, we get theta 0 and theta c from previous module
% phi is the inflow ratio

V = params.V;
Omega = params.Omega;
R = params.R;
W = params.W;
rho = params.rho;
S_fp = params.S_fp;
theta_t = params.theta_t;   
theta_0 = results_wf2.theta_0;           
theta_s = results_wf2.theta_s;   
theta_c = results_wf2.theta_c;   


%  Glauert, same as bladedynamics

Df      = 1/2 * rho * V^2 * S_fp;
alpha_D = -Df / W;

Vz = -V * sin(alpha_D);              
Vx =  V * cos(alpha_D);               

vi0    = sqrt(W / (2 * rho * params.S));
Vz_nd  = Vz / vi0;
Vx_nd  = Vx / vi0;

fun    = @(vi_nd) vi_nd * sqrt(Vx_nd^2 + (Vz_nd + vi_nd)^2) - 1;
vi_nd  = fsolve(fun, 1, optimset('Display','off'));
vi     = vi_nd * vi0;


mu_x   = Vx / (Omega * R);                        
lambda = V * sin(alpha_D) / (Omega * R) - vi / (Omega * R); 

% x = r/R
% psi = azimuth

N_psi = 720;
N_x   = 200;

psi_vec = linspace(0, 2*pi, N_psi);
x_vec   = linspace(0.04, 1.0, N_x);    % start away from hub to avoid infinite values

[PSI, X] = meshgrid(psi_vec, x_vec);

%collective + cyclic + linear twist and assuming no flapping
theta = theta_0 + theta_s .* sin(PSI) + theta_c .* cos(PSI) + theta_t .* X;

Ut_OmegaR = mu_x .* sin(PSI) + X;

% Inflow angle
PHI = lambda ./ Ut_OmegaR;

%alpha = theta + phi
alpha = theta + PHI;
alpha_deg = rad2deg(alpha);
alpha_deg(abs(Ut_OmegaR) < 0.02) = NaN; % reverse flow region

% transform from polar to cartesian with psi
x_cartesian =  X .* sin(PSI); % psitive in advancing side(psi=90) and egative in retreatig side (psi=270)
y_cartesian =  X .* cos(PSI);


%  Plot

figure;
hold on;

alpha_levels = -20 : 2 : 20;          
[C, hcl] = contour(x_cartesian, y_cartesian, alpha_deg, alpha_levels, 'k-', 'LineWidth', 0.7);
clabel(C, hcl, 'FontSize', 7, 'Color', [0.15 0.15 0.15],'LabelSpacing', 200);

% Same as in reverse flow
if mu_x > 0.01
    N_rf  = 500;
    psi_rf = linspace(pi, 2*pi, N_rf);
    r_rf   = max(0, -mu_x .* sin(psi_rf));
    X_rf   = r_rf .* sin(psi_rf);
    Y_rf   = r_rf .* cos(psi_rf);
    plot(X_rf, Y_rf, 'w--', 'LineWidth', 2.2, 'DisplayName', 'Reverse flow');
    
   
end


th = linspace(0, 2*pi, 600);
plot(sin(th), cos(th), 'k-', 'LineWidth', 2.5);

plot([0  0 ], [-1.08 1.08], '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
plot([-1.08 1.08], [0  0 ], '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);

text( 0.04,  1.14, '$\psi=0^\circ$',    'Interpreter','latex','FontSize',9,'Color',[0.35 0.35 0.35]);
text( 1.09,  0.04, '$\psi=90^\circ$',   'Interpreter','latex','FontSize',9,'Color',[0.35 0.35 0.35]);
text( 0.04, -1.16, '$\psi=180^\circ$',  'Interpreter','latex','FontSize',9,'Color',[0.35 0.35 0.35]);
text(-1.52,  0.04, '$\psi=270^\circ$',  'Interpreter','latex','FontSize',9,'Color',[0.35 0.35 0.35]);




xlabel('$y/R$ ', 'Interpreter','latex','FontSize',14);
ylabel('$x/R$ ', 'Interpreter','latex','FontSize',14);

title_str = sprintf(['Angle of Attack $\\alpha$ for one blade revolution at ' '$h=%d$ m '], h);
title(title_str, 'Interpreter','latex','FontSize',11);

axis equal;
xlim([-1.55 1.55]);
ylim([-1.60 1.30]);
grid on; box on;

end
