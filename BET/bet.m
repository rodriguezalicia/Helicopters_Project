% =================================================================
%    BET - Blade Element Theory for Helicopter Rotor Analysis
% =================================================================

function results = bet(params)
    % BET - Blade Element Theory for Helicopter Rotor Analysis
    % Inputs:
    %   params - structure containing all the parameters for the analysis
    % Outputs:
    %   results - structure containing the results of the analysis
    
    % Thrust Coefficient
    Ct = params.W / (params.rho * params.n_blades * params.Omega^2 * params.R^4);

    % Induced Power Coefficient
    CPi(:) = Ct/2*(sqrt(2*Ct+(Vz(:)/(Omega*R)).^2)+Vz(:)/(Omega*R));

    % Lambdas
    lambda_i(:) = 0.5*(sqrt(2*Ct+(Vz(:)/(Omega*R)).^2)-Vz(:)/(Omega*R));
    lambda(:) = lambda_i(:) + Vz(:)/(Omega*R); 
    
    % Global solidity
    sigma = @(x) params.n_blades*c(x)/(pi*params.R);
end