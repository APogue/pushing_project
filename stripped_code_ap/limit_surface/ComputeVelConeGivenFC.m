% Input: 
% fc_edges: 3*(2N) edges of the friction cone, angular component already multiplied with pho.
% lc_coeffs: limit surface coefficients.
% if 'gp', lc_coeffs would be a struct containing training data and
% gp parameters.
% lc_type: 'quadratic', 'poly4' or 'gp'.
% Output:
% vc_edges: 3*(2N) edges of the velocity cone, with torque normalized.
function [ vc_edges ] = ComputeVelConeGivenFC(fc_edges, lc_coeffs, lc_type)
   A = lc_coeffs;
   vel = A * fc_edges;
   dir_vel = bsxfun(@rdivide, vel, sqrt(sum(vel.^2, 1)));
   vc_edges = dir_vel;
end

