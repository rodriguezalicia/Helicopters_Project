%%%% SIMPLE MT MODEL FOR R22 %%%%

%%% INTPUT: %%%
% params,
% rho (density), 
% Vz (vertical speed) 

%%% OUTPUT: %%%
% vi0 (reference induced velocity at hover), 
% vi (induced velocity), 
% Pi (induced power)

function [vi0, vi, Pi] = MT(params, Vz, rho)
    % Define parameters and variables
    S = params.S;
    W = params.W;
    
    % Calculations
    vi0 = sqrt(W ./ (2 .* rho .* S)); % Reference induced velocity at hover
    Vz_ = Vz ./ vi0; % Non dimensional vertical velocity
    
    f1 = @(vi_) (vi_ .* (Vz_ + vi_) - 1);
    vi_ = fzero(f1, 1); % Solve the quadratic eq. to get non dimensional induced velocity
    Pi_ = Vz_ + vi_; % Non dimensional induced power
    
    vi = vi_ * vi0;
    Pi = Pi_ * (W * vi0);
end