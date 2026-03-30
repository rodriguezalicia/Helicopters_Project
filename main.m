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
    
    % Print results dynamically in a side-by-side comparison table
    fprintf('\n>>> ALTITUDE: %i m (rho = %.4f kg/m^3) <<<\n', h, params.rho);
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('%-32s | %-15s | %-15s | %-15s\n', 'Parameter', 'MT (Global)', 'BET (No Losses)', 'BET (With Losses)');
    fprintf('----------------------------------------------------------------------------------------\n');
    
    % Standard numerical outputs for all three methods
    fprintf('%-32s | %-15.4f | %-15.4f | %-15.4f\n', 'Thrust Coefficient (Ct)',           results_MT.Ct,     results_BET.Ct,     results_BET_losses.Ct);
    fprintf('%-32s | %-15.8f | %-15.8f | %-15.8f\n', 'Induced Power Coefficient (CPi)',   results_MT.CPi,    results_BET.CPi,    results_BET_losses.CPi);
    fprintf('%-32s | %-15.4f | %-15.4f | %-15.4f\n', 'Total inflow ratio (lambda)',       results_MT.lambda, results_BET.lambda, results_BET_losses.lambda);
    
    % Variables that MT cannot calculate (printed as strings '---')
    fprintf('%-32s | %-15s | %-15.4f | %-15.4f\n', 'Collective pitch (theta0) [rad]',   '---',             results_BET.theta0, results_BET_losses.theta0);
    fprintf('%-32s | %-15s | %-15.5f | %-15.5f\n', 'Profile Power Coefficient (CPo)',   '---',             results_BET.CPo,    results_BET_losses.CPo);
    
    % For MT, Total CP is just CPi. Total Power is just Pi.
    fprintf('%-32s | %-15.7f | %-15.7f | %-15.7f\n', 'Total Power Coefficient (CP)',      results_MT.CPi,    results_BET.CP,     results_BET_losses.CP);
    fprintf('%-32s | %-15.4f | %-15.4f | %-15.4f\n', 'Total Power (P) [kW]',              results_MT.Power,  results_BET.Power,  results_BET_losses.Power);
    fprintf('----------------------------------------------------------------------------------------\n');
    
end