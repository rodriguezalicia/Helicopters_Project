% =================================================================
%    BET - Blade Element Theory for Helicopter Rotor Analysis
% =================================================================
function results = bet_losses(params)
    % ----------------------------------------------------------
    %   BET - Blade Element Theory for Helicopter Rotor Analysis
    % Inputs:
    %   params - structure containing all the parameters for the analysis
    % Outputs:
    %   results - structure containing the results of the analysis
    % ----------------------------------------------------------
    
    % Thrust Coefficient from MT (It is fixed by the helicopter's weight)
    Ct_MT = params.W / (params.rho * params.S * params.Omega^2 * params.R^2);
    
    % For BET: reduction of effective radius, we will use Prandtl's Model
    B = 1 - sqrt(2*Ct_MT)/params.n_blades;
    
    % Induced velocity --> T = W for hovering in MT (adjusted for effective area)
    v_io = (1/B)*sqrt(params.W/(2*params.rho*params.S));
    
    % Lambdas from MT
    lambda_i = 0.5*(sqrt(2*(Ct_MT/B^2) + (params.Vz/(params.Omega*params.R))^2)-params.Vz/(params.Omega*params.R));
    lambda = lambda_i + params.Vz/(params.Omega*params.R); 
    
    % Global solidity: is S_blades/S_disk, where S_blades is the total blade area and S_disk is the rotor disk area 
    sigma = params.n_blades*integral(params.c,0,params.R) / (pi*params.R^2);
    
    % Blade discretization for numerical integration
    N = 500;
    x = linspace(0.01, B, N); % Integrate strictly up to the effective radius B
    
    % Exact inflow angle (phi) without small-angle approximation
    phi = atan((params.Vz + v_io) ./ (params.Omega * params.R .* x));
    
    % Theta distribution
    % Find collective pitch (theta0) using fzero to match Ct_MT exactly
    dCt = @(th0) 0.5 * sigma * params.CL_alpha .* ((th0 + params.theta_t .* x) - phi) .* x.^2;
    Ct_integral = @(th0) trapz(x, dCt(th0)) - Ct_MT;
    theta0 = fzero(Ct_integral, 0.5); % Collective pitch angle
    
    % Pitch limiter safety check
    theta_0_max = deg2rad(15.4);
    if abs(theta0) > theta_0_max
        theta0 = sign(theta0) * theta_0_max;     
    end
    
    % Induced Power Coefficient
    dCPi = phi .* dCt(theta0);
    CPi = trapz(x, dCPi);
    
    % Profile Power Coefficient (CPo) calculation
    % Define alpha for this specific Vz state using the exact phi
    alpha = (theta0 + params.theta_t .* x) - phi;
    
    % Evaluate alpha(x) inside cd(), and use element-wise operations (.* and .^)
    Cd = params.cd(alpha);
    dCPo = 0.5 * sigma .* Cd .* x.^3;
    CPo = trapz(x, dCPo);
    
    % Power Coefficient (CP) calculation
    CP = CPi + CPo;
    
    % Power
    Power = params.rho * params.S * (params.Omega * params.R)^3 * CP / 1000;
    Pi = params.rho * params.S * (params.Omega * params.R)^3 * CPi / 1000;
    Po = params.rho * params.S * (params.Omega * params.R)^3 * CPo / 1000;
    
    % Store results in a structure
    results.Ct = Ct_MT;
    results.CPi = CPi;
    results.lambda = lambda;
    results.theta0 = theta0;
    results.CPo = CPo;
    results.CP = CP;
    results.Power = Power;
    results.FM = Pi/(Pi+Po);
end