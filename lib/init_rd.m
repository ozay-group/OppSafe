function rd_list = init_rd(N, d_max, rnd)
% Generate a list of random disturbances
% Inputs: 
%           N (int) --- simulation steps
%           d_max (double) --- the max disturbance size
%           rnd --- random seed
    if nargin == 3
        rng(rnd);
    end
    rd_list = d_max * (-1 + 2 * rand(N,1));
end