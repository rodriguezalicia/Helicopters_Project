function results = mt(params)
% Renaming parameters for better clarity
S = params.S;
W = params.W;
Vz = params.Vz;
rho = params.rho;
Omega = params.Omega;
R = params.R;

% Calculations
vi0 = sqrt(W ./ (2 .* rho .* S)); % Reference induced velocity at hover
Vz_ = Vz ./ vi0; % Non dimensional vertical velocity

f1 = @(vi_) (vi_ .* (Vz_ + vi_) - 1);
vi_ = fzero(f1, 1); % Solve the quadratic eq. to get non dimensional induced velocity
Pi_ = Vz_ + vi_; % Non dimensional induced power

vi = vi_ * vi0;
Pi = Pi_ * (W * vi0);

Ct = W / (rho * S * Omega^2 * R^2);
lambda = (Vz + vi) / (Omega * R);
CPi = Ct * lambda;
Power_kW = Pi / 1000; % Convert Watts to kW for the table

% Return the results
results.vi = vi;
results.vi0 = vi0;
results.Pi = Pi;

% Table compatibility outputs
results.Ct = Ct;
results.lambda = lambda;
results.CPi = CPi;
results.Power = Power_kW;
end