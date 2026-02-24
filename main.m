% =======================================================
%    MAIN - Main script for Helicopter Rotor Analysis
% =======================================================

clc; clear; close all;

parameters; % Load parameters from params.m´

% Call BET function to perform the analysis
results = bet(params);

fprintf('Results of the analysis:\n');
fprintf('Thrust Coefficient (Ct) at h = %i m: %.4f\n', params.altitudes(1), results.Ct(1));
fprintf('Thrust Coefficient (Ct) at h = %i m: %.4f\n', params.altitudes(2), results.Ct(2));
fprintf('Induced Power Coefficient (CPi) at h = %i m: %.8f\n', params.altitudes(1), results.CPi(1));
fprintf('Induced Power Coefficient (CPi) at h = %i m: %.8f\n', params.altitudes(2), results.CPi(2));
fprintf('Total inflow ratio (lambda) at h = %i m: %.4f\n', params.altitudes(1), results.lambda(1));
fprintf('Total inflow ratio (lambda) at h = %i m: %.4f\n', params.altitudes(2), results.lambda(2));
fprintf('Collective pitch angle (theta0) at h = %i m: %.4f rad\n', params.altitudes(1), results.theta0(1));
fprintf('Collective pitch angle (theta0) at h = %i m: %.4f rad\n', params.altitudes(2), results.theta0(2));
fprintf('Profile Power Coefficient (CPo) at h = %i m: %.5f\n', params.altitudes(1), results.CPo(1));
fprintf('Profile Power Coefficient (CPo) at h = %i m: %.5f\n', params.altitudes(2), results.CPo(2));
fprintf('Power Coefficient (CP) at h = %i m: %.7f\n', params.altitudes(1), results.CP(1));
fprintf('Power Coefficient (CP) at h = %i m: %.7f\n', params.altitudes(2), results.CP(2));
fprintf('Power (P) at h = %i m: %.4f  kW\n', params.altitudes(1), results.Power(1));
fprintf('Power (P) at h = %i m: %.4f kW\n', params.altitudes(2), results.Power(2));