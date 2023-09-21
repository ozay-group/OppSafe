function x_list = gen_rand_states(C_max, ub, lb, N)
% Sample a list of random states  based on the given alpha
% Input: 
%       C_max --- inv set of alpha dynamics
%       ub ---  upper bound of the maximum alpha
%       lb ---  lower bound of the maximum alpha
%       N (int) --- number of samples for each alpha range

    dim = size(C_max.A, 2);
    x_list = zeros(dim-1, N);
    parfor i=1:N
        x_list(:, i) = rand_pt(C_max.slice(dim, lb), C_max.slice(dim, ub));
        alpha = get_max_alpha(C_max, x_list(:, i));
        disp(alpha);
    end
end