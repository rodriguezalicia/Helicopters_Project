% =====================================================================
% BEMT + Prescribed Wake Model for Helicopter Rotor Analysis
% =====================================================================

function results = pw(params)
% ----------------------------------------------------------
    %   PW - Blade Element Momentum Theory with Prescribed Wake
    % Inputs:
    %   params - structure containing all the parameters for the analysis
    % Outputs:
    %   results - structure containing the results of the analysis
    % ----------------------------------------------------------

    % Parameters 
    S = params.S;
    W = params.W;
    R = params.R;
    w = params.Omega;
    rho = params.rho;
    cl_a = params.CL_alpha;
    th_t = params.theta_t;
    cd = @(al) (params.cd(al));
    nb = params.n_blades;
    c = @(x) params.c(x);

    C_T_req = (W/(rho * S * (w*R)^2)); % Required thrust coefficient

    sigma = @(x) (nb * c(x)) / (pi*R);
    th_fun = @(x, th_0) (th_0 + x.*th_t);

    % Iteration for theta_0
    i = 1;
    i_max = 1000;
    tol = 1e-6;
    e = 1;

    th_0l = 0.05;
    th_0h = 0.6;

    while i <= i_max && e > tol

        th_0 = (th_0l + th_0h) / 2;

        % Estimate lambda0 using previous CT assumption
        lambda0 = sqrt(C_T_req / 2);

        % Wake shape function
        f_wake = @(x) sqrt(1 - x.^2);

        % Induced inflow distribution
        lambda = @(x) lambda0 .* f_wake(x);

        f1 = @(x) ( sigma(x) .* cl_a .* ...
            (th_fun(x, th_0) - lambda(x)./x) .* x.^2 / 2 );

        C_T = integral(f1, 0.15, 1); % avoid singularity at root for integral limits

        % Update lambda0 
        lambda0 = sqrt(abs(C_T) / 2);
        lambda = @(x) lambda0 .* f_wake(x);

        % Recompute thrust with updated inflow
        f1 = @(x) ( sigma(x) .* cl_a .* ...
            (th_fun(x, th_0) - lambda(x)./x) .* x.^2 / 2 );

        C_T = integral(f1, 0.15, 1);

        if C_T > C_T_req
            th_0h = th_0;
        else
            th_0l = th_0;
        end

        e = abs((C_T - C_T_req) / C_T_req);
        i = i + 1;
    end

    if e > tol
        error("theta_0 did not converge. Consider incrementing i_max or select better initial guesses");
    end

    % Final definitions
    th = @(x) th_fun(x, th_0);
    phi = @(x) lambda(x)./x;
    alpha = @(x) th(x) - phi(x);

    % Induced power
    f2 = @(x) ( 0.5 .* sigma(x) .* lambda(x) .* cl_a .* alpha(x) .* x.^2 );
    C_Pi = integral(f2, 0.15, 1);

    % Parasitic power
    f3 = @(x) ( sigma(x) .* cd(alpha(x)) .* x.^3 ./ 2 );
    C_P0 = integral(f3, 0.15, 1);

    % Total power coefficient
    C_P = C_Pi + C_P0;

    % Dimensional power (kW)
    P = C_P * (rho * S * (w*R)^3) / 1000;

    results.theta0 = th_0;
    results.CT = C_T;
    results.CPi = C_Pi;
    results.CPo = C_P0;
    results.CP = C_P;
    results.Power = P;

end