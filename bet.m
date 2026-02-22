% =================================================================
%    BET - Blade Element Theory for Helicopter Rotor Analysis
% =================================================================

function results = bet(params)
    % ----------------------------------------------------------
    %   BET - Blade Element Theory for Helicopter Rotor Analysis
    % Inputs:
    %   params - structure containing all the parameters for the analysis
    % Outputs:
    %   results - structure containing the results of the analysis
    % ----------------------------------------------------------

    % Thrust Coefficient
    Ct = params.W / (params.rho*params.S*params.Omega^2*params.R^2);

    % Initialize vectors
    CPi = zeros(length(params.Vz),1); % Induced Power Coefficient
    lambda_i = zeros(length(params.Vz),1); % Induced inflow ratio
    lambda = zeros(length(params.Vz),1); % Total inflow ratio

    % Induced Power Coefficient
    CPi(:) = Ct/2*(sqrt(2*Ct+(params.Vz(:)/(params.Omega*params.R)).^2)+params.Vz(:)/(params.Omega*params.R));

    % Lambdas
    lambda_i(:) = 0.5*(sqrt(2*Ct+(params.Vz(:)/(params.Omega*params.R)).^2)-params.Vz(:)/(params.Omega*params.R));
    lambda(:) = lambda_i(:) + params.Vz(:)/(params.Omega*params.R); 
    
    % Global solidity: is S_blades/S_disk, where S_blades is the total blade area and S_disk is the rotor disk area 
    sigma = params.n_blades*integral(@(x) params.c(x), 0, 1) / (pi*params.R);

    % Theta distribution
    theta0(:) = 6/(sigma*params.CL_alpha)*Ct + 3*lambda(:)/2 - 3*params.theta_t/4; % Collective pitch angle

    % Alicia: queda hacer la integral del Cpo y luego sacar Cp y la potencia. 


    % Store results in a structure
    results.Ct = Ct;
    results.CPi = CPi;
    results.lambda = lambda;
    results.theta0 = theta0;
end