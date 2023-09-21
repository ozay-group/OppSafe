% Implement the extended safety goverener in the paper
% Li, Nan, Yutong Li, and Ilya Kolmanovsky. "A Unified Safety Protection and Extension Governor." arXiv preprint arXiv:2304.07984 (2023).
function u_out = k_filter_tXk(tX_xu, pre_tXk, x, u)
    K = length(pre_tXk);
    for i = K:-1:1
        if pre_tXk{i}.contains(x)
            C = tX_xu{i}; 
            % fprintf('k_filter: %d\n', i);
            break;
        end
        if i == 1
                % x is safe set, just pass u
                % fprintf('k_filter: %d\n', i);
                u_out = u;
                return;
        end
    end
    
    U = C.slice(1:length(x), x);
    U.minHRep;

    if length(u) == 1
        u_min = min(U.V);
        u_max = max(U.V);
        u_out = min(max(u, u_min), u_max);
        if ~isempty(u_out)
            return;
        end
    end

    u_ = sdpvar(size(U.A, 2), 1);

    objective = (u_ - u)' * (u_ - u);
    constraints = U.A * u_ <= U.b;
    options = sdpsettings('verbose', 0, 'solver','gurobi');
    optimize(constraints, objective, options);
    
    val = value(u_);
    u_out = val(1:length(u));

end