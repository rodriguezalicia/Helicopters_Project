% =======================================================
%    MAIN - Main script for Helicopter Rotor Analysis
% =======================================================

clc; clear; close all;

parameters; % Load parameters from params.m´

% Call BET function to perform the analysis
results = bet(params);


% Printing results
fprintf('--------------------------------\n')
fprintf('  Results of the analysis:\n');
fprintf('--------------------------------\n')
fprintf('   - Thrust Coefficient (Ct): %.4f\n', results.Ct);
for i = 1:length(params.Vz)
    fprintf('   - Vz = %.2f m/s\n', params.Vz(i))
    fprintf('       - Induced Power Coefficient (CPi): %.8f\n', results.CPi(i));
    fprintf('       - Total inflow ratio (lambda): %.4f\n', results.lambda(i));
    fprintf('       - Collective pitch angle (theta0): %.4f rad\n', results.theta0(i));
end