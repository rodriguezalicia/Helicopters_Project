% =====================================================================
%    BEMT - Blade Element Momentum Theory for Helicopter Rotor Analysis
% =====================================================================

function results = bemt(params, Vz, useLosses)

    % ----------------------------------------------------------
    %   BEMT - Blade Element Momentum Theory for Helicopter Rotor Analysis
    % Inputs:
    %   params - structure containing all the parameters for the analysis
    %   Vz - vertical velocity
    %   rho - air density
    %   useLosses - T/F use prandtl loss factor. if Vz != 0, this will be
    %   false regardless
    % Outputs:
    %   results - structure containing the results of the analysis
    % ----------------------------------------------------------

    % Define parameters and variables
    S = params.S;
    W = params.W;
    R = params.R;
    w = params.Omega;
    cl_a = params.CL_alpha;
    th_t = params.theta_t;
    cd = @(al) (params.cd(al));
    nb = params.n_blades;
    c = @(x) params.c(x);
    rho = params.rho;
    
    % Calculations
    C_T_req = (W/(rho * S * (w*R)^2)); % The required thrust coeff. for hover
    
    sigma = @(x) (nb * c(x)) / (pi*R);
    
    th = @(x, th_0_) (th_0_ + x.*th_t);
    
    v_i0 = @(x, th_0_) (w*R * 0.5 .* ( ...
    -(Vz/(w*R) + cl_a.*sigma(x)./8) + ...
    sqrt( (Vz/(w*R) + cl_a.*sigma(x)./8).^2 + ...
          cl_a.*sigma(x)./2 .* (x.*th(x,th_0_) - Vz/(w*R)) ) ...
    ));

    % Iteration to find theta_0 and v_i0(x)
    i = 1;
    i_max = 1000; % max number of iterations
    e = 1; % relative error
    tol_i = 1e-6; % tolerance of approximation

    j_max = 50;
    tol_j = 1e-8;

    th_0l = 0.1; % initial guess for theta_0 low
    th_0h = 0.6; % initial guess for theta_0 high
    
    if Vz == 0 && useLosses
        % With prandtl correction
        while i <= i_max && e > tol_i
            th_00 = (th_0l + th_0h) / 2; % current guess
    
            x_grid = linspace(0.15, 1, 1000); % discretize space
            vi0_grid = zeros(size(x_grid));
            F_grid = vi0_grid;
    
            for k = 1:length(x_grid)
                xk = x_grid(k);
                for j = 1:j_max  % inner loop
                    lam = vi0_grid(k) / (w*R);  % use previous vi0
                    % Prandtl only valid away from root
                    if xk > 0.4
                        F = max((2/pi) * acos(exp(-nb*(1-xk)/(2*max(lam, 1e-6)))), 1e-6);
                    else
                        F = 1;
                    end
                    vi0_k = (w*R) * (cl_a*sigma(xk)/(16*F)) * ...
                            (sqrt(1 + 32*F/(cl_a*sigma(xk))*xk*th(xk,th_00)) - 1);
                    converged = abs(vi0_k - vi0_grid(k)) < tol_j;
                    vi0_grid(k) = vi0_k;
                    F_grid(k) = F;
                    if converged; break; end
                end
            end
            % then interpolate for integral()
            v_i0 = @(x) interp1(x_grid, vi0_grid, x, 'linear', 0);
            F = @(x) interp1(x_grid, F_grid, x, 'linear', 1);
            f1 = @(x, th_0_) (F(x) .* sigma(x) .* cl_a .* (th(x, th_0_) - v_i0(x)./(x .* w * R)).*x.^2 / 2);
    
            C_T = integral(@(x) f1(x, th_00), 0, 1); % calculate C_T
    
            if C_T > C_T_req
                th_0h = th_00; % lower th_0
            else
                th_0l = th_00; % higher th_0
            end
    
            e = abs((C_T - C_T_req) / C_T_req); % Calculate relative error
            i = i + 1;
        end

        if e > tol_i
            error("th_0 not found. Consider incrementing i_max or select better initial guesses")
        end
    
        th_0 = th_00;
        th = @(x) (th(x, th_0));

        lambda = @(x) (v_i0(x))./(w*R); 
        phi = @(x) (lambda(x)./x);
        alpha = @(x) (th(x) - phi(x));
        f2 = @(x) (F(x) .* 0.5.*sigma(x).*lambda(x).*cl_a.*alpha(x).*x.^2 );
        C_Pi = integral(f2, 0, 1);
    else
        % Without prandtl correction
        f1 = @(x, th_0_) (sigma(x) .* cl_a .* (th(x, th_0_) - v_i0(x, th_0_)./(x .* w * R)).*x.^2 / 2);
        while i <= i_max && e > tol_i
            th_00 = (th_0l + th_0h) / 2; % current guess
            C_T = integral(@(x) f1(x, th_00), 0, 1); % calculate C_T

            if C_T > C_T_req
                th_0h = th_00; % lower th_0
            else
                th_0l = th_00; % higher th_0
            end

            e = abs((C_T - C_T_req) / C_T_req); % Calculate relative error
            i = i + 1;
        end

        if e > tol_i
            error("th_0 not found. Consider incrementing i_max or select better initial guesses")
        end
    
        th_0 = th_00;
        th = @(x) (th(x, th_0));
        v_i0 = @(x) v_i0(x, th_0);

        lambda = @(x) (v_i0(x))./(w*R); 
        phi = @(x) (lambda(x)./x);
        alpha = @(x) (th(x) - phi(x));
        f2 = @(x) (0.5.*sigma(x).*lambda(x).*cl_a.*alpha(x).*x.^2 );
        C_Pi = integral(f2, 0, 1);
    end 

    f3 = @(x) (sigma(x) .* cd(alpha(x)) .* x.^3 ./ 2);
    C_P0 = integral(f3, 0, 1);
    
    C_P = C_P0 + C_Pi;
    P = C_P * (rho*S*(w*R)^3) / 1000;

    results.CPi = C_Pi;
    results.CPo = C_P0;
    results.CP = C_P;
    results.Power = P;
end