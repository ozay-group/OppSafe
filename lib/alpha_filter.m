function [ u_out ] = alpha_filter(dyn, C, x, alpha, u)
% The supervisor that minimally modifies the given input with respect to
% the CIS
% Inputs:
%       dyn (Dyn) --- the alpha dynamics
%       C         --- the CIS of the alpha dynamics
%       x         --- current state
%       alpha     --- the maximal alpha corresp. to x
%       u         --- a reference input
    if alpha == -1
        u_out = u;
        return
    end

    nv = length(dyn.XW_V);
    nA = length(C.b);
    nXU = length(dyn.XU.b);
    % Disturbance constraints with alpha-relaxation
    alpha_idx = length(x)+1;
    H = zeros(nv*nA+nXU, size(dyn.B,2));
    h = zeros(nv*nA+nXU, 1);
    for i = 1:nv
        d_i = dyn.XW_V{i}(:, alpha_idx);
        % H_i = H_C * [B, E]
        H((i-1)*nA + (1:nA),:) = C.A * dyn.B;
        % h_i = h_C - H_C * (A * x + alpha * E * d_i)
        if alpha < 0
            h((i-1)*nA + (1:nA)) = C.b - C.A * dyn.A * [x; alpha];
        else
            h((i-1)*nA + (1:nA)) = C.b - C.A * (dyn.A * [x; alpha] + alpha * dyn.Ew * d_i);
        end
        
    end
    % XU pair constraint
    H(nv*nA + (1:nXU),:) = dyn.XU.A(:, alpha_idx+1:end);
    h(nv*nA + (1:nXU),:) = dyn.XU.b - dyn.XU.A(:, 1:alpha_idx) * [x;alpha];

    U = Polyhedron('H', [H, h]);
    U.minHRep;

    if length(u) == 1
        U1 = U.projection(1);
        u_min = min(U1.V);
        u_max = max(U1.V);
        u_out = min(max(u, u_min), u_max);
        if ~isempty(u_out)
            return;
        end
    end

    u_ = sdpvar(size(dyn.B, 2), 1);
    Constraints = [U.A*u_<=U.b];
    % Minimizing expected action offset
    % Objective = norm(u_(1:length(u)) - u, 2)^2;
    Objective = (u_(1:length(u)) - u)'*(u_(1:length(u)) - u);

    options = sdpsettings('solver', 'gurobi', 'verbose', 0);

    optimize(Constraints, Objective, options);
    
    val = value(u_);
    u_out = val(1:length(u));

end
