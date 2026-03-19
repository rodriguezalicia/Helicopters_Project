% =================================================================
%    BET - Blade Element Theory for Helicopter Rotor Analysis
% =================================================================

function results = bet(params, Vz, rho)
    % ----------------------------------------------------------
    %   BET - Blade Element Theory for Helicopter Rotor Analysis
    % Inputs:
    %   params - structure containing all the parameters for the analysis
    % Outputs:
    %   results - structure containing the results of the analysis
    % ----------------------------------------------------------

    % Thrust Coefficient
    Ct = params.W ./ (rho .* params.S* params.Omega^2 * params.R^2);

    % Induced Power Coefficient
    CPi = 0.5*Ct.*(sqrt(2.*Ct + (Vz/(params.Omega*params.R)).^2) + Vz/(params.Omega*params.R));

    % Lambdas
    lambda_i = 0.5*(sqrt(2.*Ct + (Vz/(params.Omega*params.R)).^2)-Vz/(params.Omega*params.R));
    lambda = lambda_i + Vz/(params.Omega*params.R); 
    
    % Global solidity: is S_blades/S_disk, where S_blades is the total blade area and S_disk is the rotor disk area 
    sigma = params.n_blades*integral(params.c,0,params.R) / (pi*params.R^2);

    % Theta distribution
    theta0(:) = 6.*Ct/(sigma*params.CL_alpha) + 3*lambda/2 - 3*params.theta_t/4; % Collective pitch angle

    % Profile Power Coefficient (CPo) calculation
    % Define alpha for this specific Vz state
    alpha = @(x) theta0 + params.theta_t*x - lambda./x;
    
    % Evaluate alpha(x) inside cd(), and use element-wise operations (.* and .^)
    I = @(x) 0.5 * sigma * params.cd(alpha(x)) .* x.^3; 
    
    CPo = integral(I, 0, 1);

    % Power Coefficient (CP) calculation
    CP = CPi + CPo;

    % Power
    Power = rho.*params.S*(params.Omega*params.R)^3.*CP/1000;

    % Store results in a structure
    results.Ct = Ct;
    results.CPi = CPi;
    results.lambda = lambda;
    results.theta0 = theta0;
    results.CPo = CPo;
    results.CP = CP;
    results.Power = Power;
end