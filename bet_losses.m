% =================================================================
%    BET - Blade Element Theory (WITH LOSSES - EXACT KINEMATICS)
% =================================================================
function results = bet_losses(params)
    
    % Thrust Coefficient from MT (It is fixed by the helicopter's weight)
    Ct_MT = params.W/(params.rho*params.S*params.Omega^2*params.R^2);
    
    % For BET: reduction of effective radius, we will use Prandtl's Model
    B = 1-sqrt(2*Ct_MT)/params.n_blades;
    
    % Lambdas from MT
    lambda_i = 0.5*(sqrt(2*(Ct_MT/B^2)+(params.Vz/(params.Omega*params.R))^2)-params.Vz/(params.Omega*params.R));
    lambda = lambda_i+params.Vz/(params.Omega*params.R); 
   
    % Global solidity
    sigma = params.n_blades*integral(params.c,0,params.R)/(pi*params.R^2);
    
    % Define the integrand as a function of both x and th0.
    % (Operations must be element-wise for x, which you've already done!)
    dCt = @(x, th0) 0.5*sigma*params.CL_alpha.*((th0+params.theta_t.*x)-lambda./x).*x.^2;
    
    % Define the objective function for fzero.
    Ct_integral = @(th0) integral(@(x) dCt(x, th0), 1e-3, B) - Ct_MT;
    
    % Solve for theta0
    theta0 = fzero(Ct_integral, 0);
    
    % Induced Power Coefficient
    dCPi = @(x) lambda./x.*x.*dCt(x, theta0);
    CPi = integral(dCPi, 1e-3, B);
    
    % Profile Power Coefficient (CPo) calculation
    dCPo = @(x) 0.5*sigma.*params.cd((theta0 + params.theta_t.*x) - (lambda./x)).*x.^3;
    CPo = integral(dCPo, 1e-3, B);
    
    % Power Coefficient (CP) calculation
    CP = CPi + CPo;
    
    % Dimensional Power [kW]
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