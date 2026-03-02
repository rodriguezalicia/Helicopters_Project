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
<<<<<<< HEAD
    Ct = params.W / (params.rho*params.S*params.Omega^2*params.R^2);
=======
    Ct(:) = params.W ./ (params.rho.* params.S* params.Omega^2 * params.R^2);
>>>>>>> 8c40ee3c8dd8919b4e2b1079b63069b3015a0e6f

    % Initialize vectors
    CPi = zeros(params.naltitudes,1); % Induced Power Coefficient
    lambda_i = zeros(params.naltitudes,1); % Induced inflow ratio
    lambda = zeros(params.naltitudes,1); % Total inflow ratio

    % Induced Power Coefficient
    CPi(:) = 0.5*Ct(:).*(sqrt(2.*Ct(:) + (params.Vz/(params.Omega*params.R)).^2) + params.Vz/(params.Omega*params.R));

    % Lambdas
    lambda_i(:) = 0.5*(sqrt(2.*Ct(:) + (params.Vz/(params.Omega*params.R)).^2)-params.Vz/(params.Omega*params.R));
    lambda(:) = lambda_i(:) + params.Vz/(params.Omega*params.R); 
    
    % Global solidity: is S_blades/S_disk, where S_blades is the total blade area and S_disk is the rotor disk area 
<<<<<<< HEAD
    sigma = params.n_blades*integral(@(x) params.c(x), 0, 1) / (pi*params.R);

    % Theta distribution
    theta0(:) = 6/(sigma*params.CL_alpha)*Ct + 3*lambda(:)/2 - 3*params.theta_t/4; % Collective pitch angle
=======
    sigma = params.n_blades*integral(params.c,0,params.R) / (pi*params.R^2);

    % Theta distribution
    theta0(:) = 6.*Ct(:)/(sigma*params.CL_alpha) + 3*lambda(:)/2 - 3*params.theta_t/4; % Collective pitch angle
>>>>>>> 8c40ee3c8dd8919b4e2b1079b63069b3015a0e6f

   % Profile Power Coefficient (CPo) calculation
    CPo = zeros(params.naltitudes, 1);
    for i = 1:params.naltitudes
        % Define alpha for this specific Vz state
        alpha = @(x) theta0(i) + params.theta_t*x - lambda(i)./x;
        
        % Evaluate alpha(x) inside cd(), and use element-wise operations (.* and .^)
        I = @(x) 0.5 * sigma * params.cd(alpha(x)) .* x.^3; 
        
        CPo(i) = integral(I, 0, 1);
    end

    % Power Coefficient (CP) calculation
    CP(:) = CPi(:) + CPo(:);

    % Power
    Power(:) = params.rho(:).*params.S*(params.Omega*params.R)^3.*CP(:)/1000;

    % Store results in a structure
    results.Ct = Ct;
    results.CPi = CPi;
    results.lambda = lambda;
    results.theta0 = theta0;
    results.CPo = CPo;
    results.CP = CP;
    results.Power = Power;
end