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
    results_MT         = mt(params);
    results_BET        = bet(params);
    results_BET_losses = bet_losses(params);
    
    % Calculate dimensional power multiplier to convert Coefficients to kW
    power_mult = params.rho * params.S * (params.Omega * params.R)^3 / 1000;
    
    % Extract and calculate dimensional powers [kW]
    % MT Global assumes parasitic power is 0, so Total Power = Induced Power
    Pi_MT = results_MT.Power; 
    Po_MT = 0.0000;
    P_MT  = Pi_MT + Po_MT;
    
    % BET Ideal
    Pi_BET = results_BET.CPi * power_mult;
    Po_BET = results_BET.CPo * power_mult;
    P_BET  = results_BET.Power; % We already calculated Total Power inside the struct!
    
    % BET With Losses
    Pi_BET_losses = results_BET_losses.CPi * power_mult;
    Po_BET_losses = results_BET_losses.CPo * power_mult;
    P_BET_losses  = results_BET_losses.Power;
    
    % Print results dynamically in a side-by-side comparison table
    fprintf('\n---> ALTITUDE: %i m (rho = %.4f kg/m^3)\n', h, params.rho);
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('%-28s | %-15s | %-15s | %-15s\n', 'Parameter', 'MT (Global)', 'BET (No Losses)', 'BET (With Losses)');
    fprintf('----------------------------------------------------------------------------------------\n');
    
    % Print Power Components
    fprintf('%-28s | %-15.4f | %-15.4f | %-15.4f\n', 'Induced Power (Pi) [kW]',     Pi_MT, Pi_BET, Pi_BET_losses);
    fprintf('%-28s | %-15.4f | %-15.4f | %-15.4f\n', 'Parasitic Power (Po) [kW]',   Po_MT, Po_BET, Po_BET_losses);
    
    % Visual divider for the sum
    fprintf('----------------------------------------------------------------------------------------\n');
    
    % Print Total Power
    fprintf('%-28s | %-15.4f | %-15.4f | %-15.4f\n', 'Total Power (P) [kW]',          P_MT, P_BET, P_BET_losses);
    fprintf('----------------------------------------------------------------------------------------\n');
    
end