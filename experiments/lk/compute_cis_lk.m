% Compute the RCIS for LK dynamics
clc;clear all;close all;
%% LK dynamics
% original dynamics
[param,Safe] = constant_lk4();
param.rd_max = 0.08; 
param.rd_min = -0.08;
dyn = get_lk_dyn(param);
%% Compute the maximal RCIS of the original system
C_max_1 = dyn.win_always(Safe, 0, 0, 1);

%% Compute the maximal RCIS C_{max,[0,1]} for the auxiliary system
% construct the auxiliary system and safe set
[dyn_, Safe_] = alpha_dyn(dyn,Safe,-10);
% compute the maximal RCIS
C_max = dyn_.win_always(Safe_, 0, 0, 1);

%% Compute the backward reachable sets used in 
% "Li, Nan, Yutong Li, and Ilya Kolmanovsky. "A Unified Safety Protection
% and Extension Governor." arXiv preprint arXiv:2304.07984 (2023)".

K = dlqr(dyn.A, dyn.B, eye(1), 0);
[dyn_k, Safe_k] = k_dyn(dyn, -K, Safe);

Ac = dyn.A - dyn.B * K;
eigsAc = eig(Ac);
isSchur = all(abs(eigsAc) < 1);
fprintf("isSchur: %d\n", isSchur);

k_max = 200;
Pi = Safe_k;
Xk = [Safe];

for k = 1:k_max
    k
    Pi_last = Pi;
    Pi_pre = dyn_k.pre(Pi, 0);
    Pi = Pi_pre.intersect(Safe_k);
    Pi = minHRep(Pi);
    if Pi.isEmptySet()
        disp("Pi is empty, the invariant set is empty.")
        break;
    else
        X = Pi.projection(1:size(dyn.A, 2));
        if (Pi_last <= Pi)
            break;
        else
            Xk(end+1) = X;
        end
    end
end

k_max = length(Xk);
tX_xu = cell(k_max, 1);
pre_tXk = cell(k_max, 1);
for k = 1:k_max
    [tX_xu{k}, pre_tXk{k}] = compute_pre_tXk(dyn, Xk(k)); 
end

Pi_inf = dyn_k.win_always(Safe_k,0,0,1);
X_inf = Pi_inf.projection(1:size(dyn.A, 2));