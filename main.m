% =======================================================
%    MAIN - Main script for Helicopter Rotor Analysis
% =======================================================

clc; clear; close all;

parameters; % Load parameters from params.m´

% Call BET function to perform the analysis
results = bet(params);

<<<<<<< HEAD

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
=======
fprintf('Results of the analysis:\n');
fprintf('Thrust Coefficient (Ct): %.4f\n', results.Ct);
fprintf('Induced Power Coefficient (CPi) at Vz = 0 m/s: %.8f\n', results.CPi(1));
fprintf('Induced Power Coefficient (CPi) at Vz = 5 m/s: %.8f\n', results.CPi(2));
fprintf('Total inflow ratio (lambda) at Vz = 0 m/s: %.4f\n', results.lambda(1));
fprintf('Total inflow ratio (lambda) at Vz = 5 m/s: %.4f\n', results.lambda(2));
fprintf('Collective pitch angle (theta0) at Vz = 0 m/s: %.4f rad\n', results.theta0(1));
fprintf('Collective pitch angle (theta0) at Vz = 5 m/s: %.4f rad\n', results.theta0(2));
fprintf('Profile Power Coefficient (CPo): %.5f\n', results.CPo);
fprintf('Power Coefficient (CP): %.7f at Vz = 0 m/s\n', results.CP(1));
fprintf('Power Coefficient (CP): %.7f at Vz = 5 m/s\n', results.CP(2));
fprintf('Power (P): %.4f  kW at Vz = 0 m/s\n', results.Power(1));
fprintf('Power (P): %.4f kW at Vz = 5 m/s\n', results.Power(2));
>>>>>>> d33721a6cab6c6d9c11a61cb76081b88c2ac60eb
