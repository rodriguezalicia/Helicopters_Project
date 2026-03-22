% =======================================================
%    MAIN - Main script for Helicopter Rotor Analysis
% =======================================================

clc; clear; close all;

parameters; % Load parameters from params.m´

% MT analysis
results = mt(params, 0, 1.225);

fprintf('MT analysis for hover:\n');
fprintf('Power (P): %.4f  kW \n', results.Power);

% BET analysis

% !! Comento los resultados del bet para q no de error de q no sabe q es
% params.Vz y params.rho. Esto es lo q hay q cambiar para q sean dos inputs
% mas aparte de params, para que al llamar a la funcion se haga como:
% bet(params, Vz, rho), y en vez de devolver vectores solo devuelve 1 valor
% !!

%{
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
%}

% BEMT analysis
results = bemt(params, 0, 1.225);

fprintf('BEMT analysis w/ Prandtl correction for hover:\n');
fprintf('Power (P): %.4f  kW \n', results.Power);

%% BET forward flight

% Velocities vector

V_vector = 0:2:100; % [m/s]
P_total = zeros(size(V_vector));
Theta0 = zeros(size(V_vector));

for i = 1:length(V_vector)
    V_current = V_vector(i);

    % Aproximacion de alpha_R duda??
    alpha_R_current = deg2rad(-2 - (V_current/60)*5);

    results = bet_ff(params, V_current, alpha_R_current, 1.225);

        P_total(i) = results.Power / 1000;
        Theta0(i) = rad2deg(results.theta_0);

end

fprintf('BET forward flight analysis:\n'); % La potencia debería salir positiva tengo que corregir algo
plot(V_vector, P_total)
xlabel('Forward velocity V [m/s]');
ylabel('Power [kW]');
figure;
plot(V_vector, Theta0)
xlabel('Forward velocity V [m/s]');
ylabel('\theta_0 [deg]');
