% Implement the extended safety goverener in the paper
% Li, Nan, Yutong Li, and Ilya Kolmanovsky. "A Unified Safety Protection and Extension Governor." arXiv preprint arXiv:2304.07984 (2023).
function u_out = k_filter(dyn, Xk, x, u)
    K = length(Xk);
    for i = K:-1:1
        if Xk(i).contains(x)
            if i == K
                C = Xk(K); % x in the inv set, stay in it
            elseif i == 1
                % x is safe set, just pass u
                u_out = u;
                return;
            else
                C = Xk(i-1); % x is Xi, stay in X_{i-1}
            end
            break;
        end
    end
    V = dyn.D.V;
    nv = size(V,1);
    nA = length(C.b);
    nXU =  length(dyn.XU.b);
    H = zeros(nv*nA+nXU, size(dyn.B,2));
    h = zeros(nv*nA+nXU, 1);

    for i = 1:nv
        d_i = V(i,:)';
        % H_i = H_C * [B]
        H((i-1)*nA + (1:nA),:) = C.A * dyn.B;
        % h_i = h_C - H_C * (A * x + E * d_i)
        h((i-1)*nA + (1:nA)) = C.b - C.A * (dyn.A *x + dyn.Fd{1} * d_i);
    end
    % XU pair constraint
    H(nv*nA + (1:nXU),:) = dyn.XU.A(:, length(x)+1:end);
    h(nv*nA + (1:nXU),:) = dyn.XU.b - dyn.XU.A(:, 1:length(x))*x;

    U = Polyhedron('H', [H, h]);
    U.minHRep;

    if length(u) == 1
        u_min = min(U.V);
        u_max = max(U.V);
        u_out = min(max(u, u_min), u_max);
        if ~isempty(u_out)
            return;
        end
    end

    u_ = sdpvar(size(dyn.B, 2), 1);
    Constraints = U.A*u_<=U.b;
    % Minimizing expected action offset
    % Objective = norm(u_(1:length(u)) - u, 2)^2;
    Objective = (u_(1:length(u)) - u)'*(u_(1:length(u)) - u);

    options = sdpsettings('solver', 'gurobi', 'verbose', 0);

    optimize(Constraints, Objective, options);
    
    val = value(u_);
    u_out = val(1:length(u));
end