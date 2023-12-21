% Compute the CIS for CF dynamics
clc;clear all;close all;
%% CF dynamics
% original dynamics
dyn = get_cf_dyn();
Safe = Polyhedron('lb', [10; -5], 'ub', [20; 5]);

C_max_1  = dyn.win_always(Safe,0,0,1);

%% Compute the maximal RCIS C_{max,[0,1]} for the auxiliary system
[dyn1, Safe1] = alpha_dyn(dyn,Safe);
C_alpha = dyn1.win_always(Safe1,0,1,1);

%% Compute the backward reachable sets used in 
% "Li, Nan, Yutong Li, and Ilya Kolmanovsky. "A Unified Safety Protection
% and Extension Governor." arXiv preprint arXiv:2304.07984 (2023)".
% K = dlqr(dyn.A, dyn.B, eye(1), 2);
K = [0.2842, 0.8056];
[dyn_, Safe_] = k_dyn(dyn, K, Safe);


Ac = dyn.A + dyn.B * K;
eigsAc = eig(Ac);
isSchur = all(abs(eigsAc) < 1);
fprintf("isSchur: %d\n", isSchur);

k_max = 200;
Pi = Safe_;
Xk = [Safe];

for k = 1:k_max
    k
    Pi_last = Pi;
    Pi_pre = dyn_.pre(Pi, 0);
    Pi = Pi_pre.intersect(Safe_);
    Pi = minHRep(Pi);
    if Pi.isEmptySet()
        break;
    else
        X = Pi.projection(1:size(dyn.A, 2));
        if (Pi_last <= Pi)
            break;
        else
            Xk(end+1) = X.minHRep;
        end
    end
end

k_max = length(Xk);
tX_xu = cell(k_max, 1);
pre_tXk = cell(k_max, 1);
for k = 1:k_max
    k
    [tX_xu{k}, pre_tXk{k}] = compute_pre_tXk(dyn, Xk(k)); 
end

% compute tilde X inf
Pi_inf = dyn_.win_always(Safe_,0,0,1);
X_inf = Pi_inf.projection(1:size(dyn.A, 2));
% assert(X(end) == X_inf);

