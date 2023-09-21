% This file contains the code to reproduce the results in Section IV.A for
% the car following example
clc;clear all;close all;
%% Compute required RCISs
% Uncomment the following lines if you want to compute the RCISs used by
% the robust safety supervisor, opportunistic supervisor, and the safety
% governer in the paper
% "Li, Nan, Yutong Li, and Ilya Kolmanovsky. "A Unified Safety Protection
% and Extension Governor." arXiv preprint arXiv:2304.07984 (2023)".
% This may take 10-20 mins. Otherwise, the precomputed RCIS will be loaded.

% compute_cis_cf
% save("data/cf_k_filter_data_approx.mat");

%% Sample initial states with different alpha^*(x)
% Uncomment the following lines if you want to resample the initial states 
% (it may take 20 mins). Otherwise, you can load precomputed ones.

% load data/cf_k_filter_data_approx.mat
% alpha = linspace(0, 1, 11);
% xs = cell(1,10);
% count = zeros(1,10);
% 
% for i = 1:10
%     xs{i} = gen_rand_states(C_alpha,  0.1*i, 0.1*(i-1),1000);
% end
% save data/initial_states.mat

%% Compute the average exit time and safety rate for three supervisors
load data/cf_k_filter_data_approx.mat
load data/initial_states.mat
C_max = C_alpha; % C_{max,[0,1]}

N = 500; % simulation time
C_max_0 = C_max.slice(size(C_max.A, 2), 0);
n_V = size(C_max_0.V,1);

% synthesize a LQR controller
K = [0.2842, 0.8056];
cont = @(x) K*x;

% iterate over different runs (d_max=1)
Nr = 1000; % num of roads
d_max = 1; % maximal disturbance size
sr_list1= zeros(Nr,10);
sr_list2 = zeros(Nr,10);
sr_list3 = zeros(Nr,10);
for a = 1:10
    parfor rt = 1:1000
        rt
        d_list = init_rd(N, d_max); % generate disturbance sequences
        x0 = xs{a}(:,rt);
        [sr1, sr2, sr3]=simu_k_filter(dyn, dyn1, Safe, C_max, tX_xu, pre_tXk, Xk(end), d_list, x0, cont, false);
        sr_list1(rt,a) = sr1;
        sr_list2(rt,a) = sr2;
        sr_list3(rt,a) = sr3;
    end
end
save data/experiment_cf_1.mat

% iterate over different runs (d_max=1.05)
d_max = 1.05; % maximal disturbance size
sr_list1= zeros(Nr,10);
sr_list2 = zeros(Nr,10);
sr_list3 = zeros(Nr,10);
for a = 1:10
    parfor rt = 1:1000
        rt
        d_list = init_rd(N, d_max);
        x0 = xs{a}(:,rt);
        [sr1, sr2, sr3]=simu_k_filter(dyn, dyn1, Safe, C_max, tX_xu, pre_tXk, Xk(end), d_list, x0, cont, false);
        sr_list1(rt,a) = sr1;
        sr_list2(rt,a) = sr2;
        sr_list3(rt,a) = sr3;
    end
end
save data/experiment_cf_1_05.mat


%% Generate plots in Section IV.A
close all;
load data/experiment_cf_1.mat
exit_time1 = mean(sr_list1);
exit_time2 = mean(sr_list2);
exit_time3 = mean(sr_list3);
safe_rate1 = sum(sr_list1==1)/1000;
safe_rate2 = sum(sr_list2==1)/1000;
safe_rate3 = sum(sr_list3==1)/1000;
load data/experiment_cf_1_05.mat
exit_time4 = mean(sr_list1);
exit_time5 = mean(sr_list2);
exit_time6 = mean(sr_list3);
safe_rate4 = sum(sr_list1==1)/1000;
safe_rate5 = sum(sr_list2==1)/1000;
safe_rate6 = sum(sr_list3==1)/1000;
linewidth = 1.5;
figure(1);hold on;
plot(1:10, exit_time1*500, 'bo-', 'LineWidth',linewidth); 
plot(1:10, exit_time2*500, 'b^-', 'LineWidth',linewidth);
plot(1:10, exit_time3*500, 'bx-', 'LineWidth',linewidth);
plot(1:10, exit_time4*500, 'ro-', 'LineWidth',linewidth); 
plot(1:10, exit_time5*500, 'r^-', 'LineWidth',linewidth);
plot(1:10, exit_time6*500, 'rx-', 'LineWidth',linewidth);
legend('opportunistic ($$d_{max} = 1$$)', 'safety governer ($$d_{max} = 1$$)', 'robust ($$d_{max} = 1$$)', 'opportunistic ($$d_{max} = 1.05$$)', 'safety governer ($$d_{max} = 1.05$$)', 'robust ($$d_{max} = 1.05$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Average exit time')
set(gca,'FontSize',12)
figure(2);hold on;
plot(1:10, safe_rate1, 'bo-', 'LineWidth',linewidth); 
plot(1:10, safe_rate2, 'b^-', 'LineWidth',linewidth);
plot(1:10, safe_rate3, 'bx-', 'LineWidth',linewidth);
plot(1:10, safe_rate4, 'ro-', 'LineWidth',linewidth); 
plot(1:10, safe_rate5, 'r^-', 'LineWidth',linewidth);
plot(1:10, safe_rate6, 'rx-', 'LineWidth',linewidth);
legend('opportunistic \\($$d_{max} = 1$$)', 'safety governer ($$d_{max} = 1$$)', 'robust ($$d_{max} = 1$$)', 'opportunistic ($$d_{max} = 1.05$$)', 'safety governer ($$d_{max} = 1.05$$)', 'robust ($$d_{max} = 1.05$$)', 'Interpreter','latex')
ylabel('Safety rate')
xlabel('Group index of initial states');
set(gca,'FontSize',12)