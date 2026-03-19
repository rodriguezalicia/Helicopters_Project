% =======================================================
%    MAIN - Main script for Helicopter Rotor Analysis
% =======================================================

% TODO: Implement loop within this script for the different heights and Vz

clc; clear; close all;

parameters; % Load parameters from params.m´

% Call MT function to perform the analysis
results = mt(params, 0, 1.225);

fprintf('\nResults of the MT analysis:\n');
fprintf('Power (P) at h = %i m: %.4f  kW\n', 0, results.Power);

% Call BET function to perform the analysis
results = bet(params, 0, 1.225);

fprintf('\nResults of the BET analysis:\n');
% fprintf('Thrust Coefficient (Ct) at h = %i m: %.4f\n', params.altitudes(1), results.Ct(1));
% fprintf('Induced Power Coefficient (CPi) at h = %i m: %.8f\n', params.altitudes(1), results.CPi(1));
% fprintf('Total inflow ratio (lambda) at h = %i m: %.4f\n', params.altitudes(1), results.lambda(1));
% fprintf('Collective pitch angle (theta0) at h = %i m: %.4f rad\n', params.altitudes(1), results.theta0(1));
% fprintf('Profile Power Coefficient (CPo) at h = %i m: %.5f\n', params.altitudes(1), results.CPo(1));
% fprintf('Power Coefficient (CP) at h = %i m: %.7f\n', params.altitudes(1), results.CP(1));
fprintf('Power (P) at h = %i m: %.4f  kW\n', 0, results.Power);

% Call BEMT function to perform the analysis
results = bemt(params, 0, 1.225);

fprintf('\nResults of the BEMT analysis w/ Prandtl correction:\n');
fprintf('Power (P) at h = %i m: %.4f  kW\n', 0, results.Power);