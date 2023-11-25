% This file contains the code to reproduce the results in the response letter for
% the car following example
clc;clear all;close all;
% we use precomputed RCIS and inital state samples
load data/cf_k_filter_data_approx.mat
load data/initial_states.mat
C_max = C_alpha; 

rng(10); % fix the random seed

%% Compute the average exit time and safety rate for three supervisors
N = 500; % simulation time
d_max = 1;
C_max_0 = C_max.slice(size(C_max.A, 2), 0);
n_V = size(C_max_0.V,1);

% generate a feedback controller
K = [0.2842, 0.8056];
cont = @(x) K*x;

% iterate over different runs (std = 0.05)
Nr = 1000; % num of roads
std = 0.05;
sr_list1= zeros(Nr,10);
sr_list2 = zeros(Nr,10);
sr_list3 = zeros(Nr,10);
for a = 1:10
    parfor rt = 1:1000
        rt
        d_list = init_rd_gaussian(N, std, d_max);
        x0 = xs{a}(:,rt);
        [sr1, sr2, sr3]=simu_k_filter(dyn, dyn1, Safe, C_max, tX_xu, pre_tXk, Xk(end), d_list, x0, cont, true);
        sr_list1(rt,a) = sr1;
        sr_list2(rt,a) = sr2;
        sr_list3(rt,a) = sr3;
    end
end
save data/experiment_cf_g_0_05.mat

% iterate over different runs (std = 0.5)
std = 0.5;
sr_list1= zeros(Nr,10);
sr_list2 = zeros(Nr,10);
sr_list3 = zeros(Nr,10);
for a = 1:10
    parfor rt = 1:1000
        rt
        d_list = init_rd_gaussian(N, std, d_max);
        x0 = xs{a}(:,rt);
        [sr1, sr2, sr3]=simu_k_filter(dyn, dyn1, Safe, C_max, tX_xu, pre_tXk, Xk(end), d_list, x0, cont, true);
        sr_list1(rt,a) = sr1;
        sr_list2(rt,a) = sr2;
        sr_list3(rt,a) = sr3;
    end
end
save data/experiment_cf_g_0_5.mat

% iterate over different runs (std = 5)
std = 5;
sr_list1= zeros(Nr,10);
sr_list2 = zeros(Nr,10);
sr_list3 = zeros(Nr,10);
for a = 1:10
    parfor rt = 1:1000
        rt
        d_list = init_rd_gaussian(N, std, d_max);
        x0 = xs{a}(:,rt);
        [sr1, sr2, sr3]=simu_k_filter(dyn, dyn1, Safe, C_max, tX_xu, pre_tXk, Xk(end), d_list, x0, cont, true);
        sr_list1(rt,a) = sr1;
        sr_list2(rt,a) = sr2;
        sr_list3(rt,a) = sr3;
    end
end
save data/experiment_cf_g_5.mat

%% Histograms of disturbancess
rng(10);
figure(7)
histogram(init_rd_gaussian(N, 0.05, d_max),10)
axis([-1,1,0,140])

figure(8)
histogram(init_rd_gaussian(N, 0.5, d_max),10)
axis([-1,1,0,90])

figure(9)
histogram(init_rd_gaussian(N, 5, d_max),10)
axis([-1,1,0,250])


%% plots in the response letter
close all;
load data/experiment_cf_g_0_05.mat
exit_time1 = mean(sr_list1);
exit_time2 = mean(sr_list2);
exit_time3 = mean(sr_list3);
safe_rate1 = sum(sr_list1==1)/1000;
safe_rate2 = sum(sr_list2==1)/1000;
safe_rate3 = sum(sr_list3==1)/1000;
load data/experiment_cf_g_0_5.mat
exit_time4 = mean(sr_list1);
exit_time5 = mean(sr_list2);
exit_time6 = mean(sr_list3);
safe_rate4 = sum(sr_list1==1)/1000;
safe_rate5 = sum(sr_list2==1)/1000;
safe_rate6 = sum(sr_list3==1)/1000;
load data/experiment_cf_g_5.mat
exit_time7 = mean(sr_list1);
exit_time8 = mean(sr_list2);
exit_time9 = mean(sr_list3);
safe_rate7 = sum(sr_list1==1)/1000;
safe_rate8 = sum(sr_list2==1)/1000;
safe_rate9 = sum(sr_list3==1)/1000;
linewidth = 1.5;
figure(1);hold on;
plot(1:10, exit_time1*500, 'bo-', 'LineWidth',linewidth); 
plot(1:10, exit_time2*500, 'r^-', 'LineWidth',linewidth);
plot(1:10, exit_time3*500, 'gx-', 'LineWidth',linewidth);
legend('opportunistic ($$std = 0.05$$)', 'safety governer ($$std = 0.05$$)', 'robust ($$std = 0.05$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Average exit time')
set(gca,'FontSize',12)
figure(2);hold on;
plot(1:10, exit_time4*500, 'bo-', 'LineWidth',linewidth); 
plot(1:10, exit_time5*500, 'r^-', 'LineWidth',linewidth);
plot(1:10, exit_time6*500, 'gx-', 'LineWidth',linewidth);
legend('opportunistic ($$std = 0.5$$)', 'safety governer ($$std = 0.5$$)', 'robust ($$std = 0.5$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Average exit time')
set(gca,'FontSize',12)
figure(3);hold on;
plot(1:10, exit_time7*500, 'bo-', 'LineWidth',linewidth); 
plot(1:10, exit_time8*500, 'r^-', 'LineWidth',linewidth);
plot(1:10, exit_time9*500, 'gx-', 'LineWidth',linewidth);
legend('opportunistic ($$std = 5$$)', 'safety governer ($$std = 5$$)', 'robust ($$std = 5$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Average exit time')
set(gca,'FontSize',12)
figure(4);hold on;
plot(1:10, safe_rate1, 'bo-', 'LineWidth',linewidth); 
plot(1:10, safe_rate2, 'r^-', 'LineWidth',linewidth);
plot(1:10, safe_rate3, 'gx-', 'LineWidth',linewidth);
legend('opportunistic ($$std = 0.05$$)', 'safety governer ($$std = 0.05$$)', 'robust ($$std = 0.05$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Safety rate')
set(gca,'FontSize',12)
figure(5);hold on;
plot(1:10, safe_rate4, 'bo-', 'LineWidth',linewidth); 
plot(1:10, safe_rate5, 'r^-', 'LineWidth',linewidth);
plot(1:10, safe_rate6, 'gx-', 'LineWidth',linewidth);
legend('opportunistic ($$std = 0.5$$)', 'safety governer ($$std = 0.5$$)', 'robust ($$std = 0.5$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Safety rate')
set(gca,'FontSize',12)
figure(6);hold on;
plot(1:10, safe_rate7, 'bo-', 'LineWidth',linewidth); 
plot(1:10, safe_rate8, 'r^-', 'LineWidth',linewidth);
plot(1:10, safe_rate9, 'gx-', 'LineWidth',linewidth);
legend('opportunistic ($$std = 5$$)', 'safety governer ($$std = 5$$)', 'robust ($$std = 5$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Safety rate')
set(gca,'FontSize',12)