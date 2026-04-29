function reverse_flow(params, h)
% Renaming parameters for better clarity
V     = params.V;
Omega = params.Omega;
R     = params.R;
W     = params.W;
rho   = params.rho;
S_fp  = params.S_fp;

Df      = 0.5 * rho * V^2 * S_fp; % Fuselage drag
alpha_D = -Df / W;

mu_x = (V / (Omega * R)) * cos(alpha_D);

N   = 2000;
psi = linspace(0, 2*pi, N);

% Rotor disk
X_disk = sin(psi);
Y_disk = cos(psi);

% Reverse flow
psi_rf = linspace(pi, 2*pi, N);
r_rf   = -mu_x * sin(psi_rf);
X_rf   = r_rf .* sin(psi_rf);
Y_rf   = r_rf .* cos(psi_rf);

%% Plot
figure;
hold on;

plot(X_disk, Y_disk, 'k-', 'LineWidth', 2.5);
plot(X_rf, Y_rf, 'r-', 'LineWidth', 2.5, 'DisplayName', 'RF boundary');
quiver(0, -1.5, 0, 0.3, 0, 'b', 'LineWidth', 2.0, 'MaxHeadSize', 1.8);
text(0.08, -1.45, '$\mathbf{V}$', 'Interpreter', 'latex', 'FontSize', 15, 'Color', 'b');
text( 0.04,  1.14, '$\psi = 0^\circ$',    'Interpreter','latex','FontSize',10,'Color',[0.4 0.4 0.4]);
text( 1.10,  0.04, '$\psi = 90^\circ$',   'Interpreter','latex','FontSize',10,'Color',[0.4 0.4 0.4]);
text( 0.04, -1.16, '$\psi = 180^\circ$',  'Interpreter','latex','FontSize',10,'Color',[0.4 0.4 0.4]);
text(-1.50,  0.04, '$\psi = 270^\circ$',  'Interpreter','latex','FontSize',10,'Color',[0.4 0.4 0.4]);

% Indicate regiondiameter

text(-mu_x/2, 0.05, sprintf('$\\mu_x = %.3f$', mu_x), ...
    'Interpreter','latex','FontSize',10,'Color','r','HorizontalAlignment','center');

xlabel('$y/R$ ',       'Interpreter', 'latex', 'FontSize', 14);
ylabel('$x/R$ ', 'Interpreter', 'latex', 'FontSize', 14);

title(sprintf(('Reverse Flow Region \\ $h = %d$ m, \\ ' ), h), ...
    'Interpreter', 'latex', 'FontSize', 13);

legend({'Rotor disk', 'Reverse flow region', ''}, ...
    'Interpreter', 'latex', 'FontSize', 11, 'Location', 'northeast');

axis equal;
xlim([-1.65 1.65]);
ylim([-1.65 1.65]);
grid on;
box  on;

end
