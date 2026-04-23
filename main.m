% =======================================================
%    MAIN - Main script for Helicopter Rotor Analysis
% =======================================================
clc; clear; close all;
parameters; % Load parameters from params.m

% Define flight conditions
altitudes = [0, 2000];
params.Vz = 0;

% =======================================================
%    ANALYSIS LOOP & PRINTING
% =======================================================
fprintf('========================================================================================\n');
fprintf('                          RESULTS OF THE AERODYNAMIC ANALYSIS                           \n');
fprintf('========================================================================================\n');

for i = 1:length(altitudes)
    
    h = altitudes(i);
    
    % Dynamically calculate air density for this specific altitude
    [~, ~, ~, rho] = atmosisa(h);
    params.rho = rho; % Inject current density into the params struct
    
    % Run the aerodynamic analyses for THIS specific altitude
    results_MT(i)          = mt(params);
    results_BET(i)         = bet(params);
    results_BET_losses(i)  = bet_losses(params);
    results_BEMT(i)        = bemt(params, 0, false);
    results_BEMT_losses(i) = bemt(params, 0, true); 
    results_PW(i)          = pw(params);
    
    % Calculate dimensional power multiplier to convert Coefficients to kW
    power_mult = params.rho * params.S * (params.Omega * params.R)^3 / 1000;
    
    % Extract and calculate dimensional powers [kW]
    % MT Global assumes parasitic power is 0, so Total Power = Induced Power
    Pi_MT = results_MT(i).Power; 
    Po_MT = 0.0000;
    P_MT  = Pi_MT + Po_MT;
    
    % BET Ideal
    Pi_BET = results_BET(i).CPi * power_mult;
    Po_BET = results_BET(i).CPo * power_mult;
    P_BET  = results_BET(i).Power; % We already calculated Total Power inside the struct!
    
    % BET With Losses
    Pi_BET_losses = results_BET_losses(i).CPi * power_mult;
    Po_BET_losses = results_BET_losses(i).CPo * power_mult;
    P_BET_losses  = results_BET_losses(i).Power;

    % BEMT Ideal
    Pi_BEMT = results_BEMT(i).CPi * power_mult;
    Po_BEMT = results_BEMT(i).CPo * power_mult;
    P_BEMT  = results_BEMT(i).Power; % We already calculated Total Power inside the struct!
    
    % BEMT With Losses
    Pi_BEMT_losses = results_BEMT_losses(i).CPi * power_mult;
    Po_BEMT_losses = results_BEMT_losses(i).CPo * power_mult;
    P_BEMT_losses  = results_BEMT_losses(i).Power;

    % BEMT With Prescribed Wake (additional method)
    Pi_PW = results_PW(i).CPi * power_mult;
    Po_PW = results_PW(i).CPo * power_mult;
    P_PW  = results_PW(i).Power;
    
    % Print results dynamically in a side-by-side comparison table
    fprintf('\n---> ALTITUDE: %i m (rho = %.4f kg/m^3)\n', h, params.rho);
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('%-28s | %-15s | %-15s | %-15s | %-15s | %-15s | %-15s\n', 'Parameter', 'MT (Global)', 'BET (No Losses)', 'BET (With Losses)', 'BEMT (No Losses)', 'BEMT (With Losses)', 'BEMT Prescribed wake');
    fprintf('----------------------------------------------------------------------------------------\n');
    
    % Print Power Components
    fprintf('%-28s | %-15.4f | %-15.4f | %-16f  | %-16f  | %-16f  | %-15.4f\n', 'Induced Power (Pi) [kW]',     Pi_MT, Pi_BET, Pi_BET_losses, Pi_BEMT, Pi_BEMT_losses, Pi_PW);
    fprintf('%-28s | %-15.4f | %-15.4f | %-16f  | %-16f  | %-16f  | %-15.4f\n', 'Parasitic Power (Po) [kW]',   Po_MT, Po_BET, Po_BET_losses, Po_BEMT, Po_BEMT_losses, Po_PW);
    
    % Visual divider for the sum
    fprintf('----------------------------------------------------------------------------------------\n');
    
    % Print Total Power
    fprintf('%-28s | %-15.4f | %-15.4f | %-16f  | %-16f  | %-16f  | %-15.4f\n', 'Total Power (P) [kW]',          P_MT, P_BET, P_BET_losses, P_BEMT, P_BEMT_losses, P_PW);
    fprintf('----------------------------------------------------------------------------------------\n');
    
end

bar(altitudes,[results_MT(1).Power results_BET(1).Power results_BET_losses(1).Power results_BEMT(1).Power results_BEMT_losses(1).Power results_PW(1).Power; results_MT(2).Power results_BET(2).Power results_BET_losses(2).Power results_BEMT(2).Power results_BEMT_losses(2).Power results_PW(2).Power])
legend('MT','BET','BET with losses','BEMT','BEMT with losses','BEMT with prescribed wake','Interpreter','latex','FontSize',15)
xlabel('Altitude [m]','Interpreter','latex','FontSize',15)
ylabel('Power [kW]','Interpreter','latex','FontSize',15)


%% WORKFLOW 2 ADVANCED TRIM


for i = 1:length(altitudes)
 
    h = altitudes(i);
    [~, ~, ~, rho] = atmosisa(h);
    params.rho = rho;
 
    results_trim(i) = trim(params, results_MT(i));
    Td_aero(i) = aero_module_simple (params, results_trim(i).theta_0, results_trim(i).theta_s, results_trim(i).lambda_i);
    [beta_0_bd(i), theta_s_bd(i), theta_c_bd(i)] = blade_dynamics(params, results_trim(i).theta_0, results_trim(i).beta_c, results_trim(i).beta_s);

    % Use as an initial guess for theta_0 the results obtained for trim
    theta_0_init = results_trim(i).theta_0;
 
    % Workflow 2 function
    results_wf2(i) = workflow2_trim(params, results_MT(i), theta_0_init);
   
 
    % Convergence check
    if results_wf2(i).converged
        fprintf('\n  Altitude %d m: converged  |residual| = %.4f N\n', h, results_wf2(i).residual);
    else
        warning('  [!!]  Altitude %d m: did NOT converge\n', h);
    end
 
    % Print results
    sep = repmat('-', 1, 58);
    fprintf('  --- Altitude: %d m  (rho = %.4f kg/m^3) ---\n', h, params.rho);
    fprintf('  %s\n', sep);
    fprintf('  %-32s | %8.4f rad  (%7.3f deg)\n', 'theta_0  (collective)',     results_wf2(i).theta_0, rad2deg(results_wf2(i).theta_0));
    fprintf('  %-32s | %8.4f rad  (%7.3f deg)\n', 'theta_s  (long. cyclic)',   results_wf2(i).theta_s, rad2deg(results_wf2(i).theta_s));
    fprintf('  %-32s | %8.4f rad  (%7.3f deg)\n', 'theta_c  (lat. cyclic)',    results_wf2(i).theta_c, rad2deg(results_wf2(i).theta_c));
    fprintf('  %-32s | %8.4f rad  (%7.3f deg)\n', 'beta_0   (coning angle)',   results_wf2(i).beta_0,  rad2deg(results_wf2(i).beta_0));
    fprintf('  %-32s | %8.4f rad  (%7.3f deg)\n', 'beta_c   (long. flapping)', results_wf2(i).beta_c,  rad2deg(results_wf2(i).beta_c));
    fprintf('  %-32s | %8.4f rad  (%7.3f deg)\n', 'alpha_D  (disk tilt)',      results_wf2(i).alpha_D, rad2deg(results_wf2(i).alpha_D));
    fprintf('  %s\n', sep);
    fprintf('  %-32s | %10.2f N\n', 'T_D    (required thrust)', results_wf2(i).T_D);
    fprintf('  %-32s | %10.2f N\n', 'T_aero (achieved thrust)', results_wf2(i).T_aero);
    fprintf('  %s\n', sep);
 
end

% Plot Angles vs Altitude


data_0m = [results_wf2(1).theta_0, results_wf2(1).theta_s, results_wf2(1).theta_c, ...
           results_wf2(1).beta_0,  results_wf2(1).beta_c,  results_wf2(1).alpha_D] * (180/pi);

data_2000m = [results_wf2(2).theta_0, results_wf2(2).theta_s, results_wf2(2).theta_c, ...
              results_wf2(2).beta_0,  results_wf2(2).beta_c,  results_wf2(2).alpha_D] * (180/pi);

plot_data = [data_0m; data_2000m];


figure('Color', 'w');
b = bar(altitudes, plot_data, 'grouped'); 
xlabel('Altitude [m]', 'Interpreter', 'latex', 'FontSize', 15)
ylabel('Angles [deg]', 'FontSize', 15) 
xticks(altitudes);
grid on;


legend({'\theta_0 (Collective)', '\theta_s (Long. Cyclic)', '\theta_c (Lat. Cyclic)', ...
        '\beta_0 (Coning)', '\beta_c (Long. Flapping)', '\alpha_D (Disk Tilt)'}, ...
        'Location', 'northeastoutside', 'FontSize', 12);

title('Advanced Trim', 'FontSize', 14);


