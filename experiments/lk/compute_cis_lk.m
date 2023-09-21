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