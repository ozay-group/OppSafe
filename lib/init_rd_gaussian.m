function rd_list = init_rd_gaussian(N, std, max_rd, rnd)
% Generate a list of gaussian random variables truncated at max_rd 
% Inputs: 
%           N (int) --- simulation steps
%           std (double) --- the standard deviation of Gaussian random
%           variable
%           max_rd --- the maximal threshold
%           rnd --- random seed
    if nargin == 4
        rng(rnd);
    end
    % rd_list = max_curvature * (-1 + 2 * rand(N,1));
    rd_list = max(-max_rd, min(std * randn(N,1),  max_rd));
end