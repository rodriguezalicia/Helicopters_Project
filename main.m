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
