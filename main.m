% =======================================================
%    MAIN - Main script for Helicopter Rotor Analysis
% =======================================================

clc; clear; close all;

parameters; % Load parameters from params.m´

% Call BET function to perform the analysis
results = bet(params);

fprintf('Results of the analysis:\n');
fprintf('Thrust Coefficient (Ct): %.4f\n', results.Ct);
fprintf('Induced Power Coefficient (CPi) at Vz = 0 m/s: %.8f\n', results.CPi(1));
fprintf('Induced Power Coefficient (CPi) at Vz = 5 m/s: %.8f\n', results.CPi(2));
fprintf('Total inflow ratio (lambda) at Vz = 0 m/s: %.4f\n', results.lambda(1));
fprintf('Total inflow ratio (lambda) at Vz = 5 m/s: %.4f\n', results.lambda(2));
fprintf('Collective pitch angle (theta0) at Vz = 0 m/s: %.4f rad\n', results.theta0(1));
fprintf('Collective pitch angle (theta0) at Vz = 5 m/s: %.4f rad\n', results.theta0(2));