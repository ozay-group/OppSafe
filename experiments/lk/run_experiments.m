% This file contains the code to reproduce the results in Section IV.B for
% the lane keeping example
clc;clear all;close all;
%% Compute required RCISs
% Uncomment the following lines if you want to compute the RCISs used by
% the robust safety supervisor, opportunistic supervisor. This step may 
% take hours. Otherwise, the precomputed RCISs will be loaded.

% compute_cis_lk 
% save data/lk_inv_set.mat C_max Safe Safe_ dyn dyn_ param

%% Sample initial states with different alpha^*(x)
% Uncomment the following lines if you want to resample the initial states 
% (it may take hours). Otherwise, you can load precomputed ones.

% load('data/lk_inv_set.mat')
% num_interval = 10;
% num_samples = 1000;
% 
% kp = linspace(0, 1, num_interval+1);
% xs = cell(1, num_interval);
% for i = 1:num_interval
%     fprintf('ub: %d, lb: %d\n', kp(i), kp(i+1));
%     xs{i} = gen_rand_states(C_max, kp(i+1), kp(i), num_samples);
% end
% save('data/init_state_pos_10a_1000slp.mat', 'xs')

%% Compute the average exit time and safety rate for both supervisors
% ---- load precomputed RCISs and samples of initial states ----
load data/lk_inv_set.mat
load data/init_state_pos_10a_1000slp.mat

N = 500; % simulation time
% compute C_{max,0}
C_max_0 = C_max.slice(size(C_max.A, 2), 0);
n_V = size(C_max_0.V,1);

% synthesize a LQR controller
K = dlqr(dyn.A, dyn.B, eye(1), 0);
cont = @(x) controller(K, x);

for rd_max = [0.08, 0.12, 0.16] % maximal disturbance size
    % iterate over different roads
    Nr = 1000; % num of roads
    sr_list1= zeros(Nr,10);
    sr_list2 = zeros(Nr,10);
    for a = 1:10
        parfor rt = 1:Nr
            rd_list = init_rd(N, rd_max); % generate disturbance sequences
            x0 = xs{a}(:,rt);
            [sr1, sr2]=simu_lk(dyn, dyn_, Safe, C_max, rd_list, x0, cont, false);
            sr_list1(rt,a) = sr1;
            sr_list2(rt,a) = sr2;
        end
    end
    rd_txt = num2str(rd_max);
    save("data/experiment_lk_0_" + rd_txt(3:4)+".mat")
end
%% Generate plots in Section IV.B
load data/experiment_lk_0_08.mat
exit_time1 = mean(sr_list1);
exit_time2 = mean(sr_list2);
safe_rate1 = sum(sr_list1==1)/1000;
safe_rate2 = sum(sr_list2==1)/1000;
load data/experiment_lk_0_12.mat
exit_time3 = mean(sr_list1);
exit_time4 = mean(sr_list2);
safe_rate3 = sum(sr_list1==1)/1000;
safe_rate4 = sum(sr_list2==1)/1000;
load data/experiment_lk_0_16.mat
exit_time5 = mean(sr_list1);
exit_time6 = mean(sr_list2);
safe_rate5 = sum(sr_list1==1)/1000;
safe_rate6 = sum(sr_list2==1)/1000;
linewidth = 1.5;
figure(1);hold on;
plot(1:10, exit_time1*500, 'bo-', 'LineWidth',linewidth); 
plot(1:10, exit_time2*500, 'bx-', 'LineWidth',linewidth);
plot(1:10, exit_time3*500, 'go-', 'LineWidth',linewidth);
plot(1:10, exit_time4*500, 'gx-', 'LineWidth',linewidth);
plot(1:10, exit_time5*500, 'ro-', 'LineWidth',linewidth); 
plot(1:10, exit_time6*500, 'rx-', 'LineWidth',linewidth);
legend('opportunistic ($$d_{max} = 0.08$$)', 'robust ($$d_{max} = 0.08$$)','opportunistic ($$d_{max} = 0.12$$)', 'robust ($$d_{max} = 0.12$$)','opportunistic ($$d_{max} = 0.16$$)', 'robust ($$d_{max} = 0.16$$)', 'Interpreter','latex')
xlabel('Group index of initial states');
ylabel('Average exit time')
set(gca,'FontSize',12)
figure(2);hold on;
plot(1:10, safe_rate1, 'bo-', 'LineWidth',linewidth); 
plot(1:10, safe_rate2, 'bx-', 'LineWidth',linewidth);
plot(1:10, safe_rate3, 'go-', 'LineWidth',linewidth);
plot(1:10, safe_rate4, 'gx-', 'LineWidth',linewidth);
plot(1:10, safe_rate5, 'ro-', 'LineWidth',linewidth); 
plot(1:10, safe_rate6, 'rx-', 'LineWidth',linewidth);
legend('opportunistic ($$d_{max} = 0.08$$)', 'robust ($$d_{max} = 0.08$$)','opportunistic ($$d_{max} = 0.12$$)', 'robust ($$d_{max} = 0.12$$)','opportunistic ($$d_{max} = 0.16$$)', 'robust ($$d_{max} = 0.16$$)', 'Interpreter','latex')
ylabel('Safety rate')
xlabel('Group index of initial states');
set(gca,'FontSize',12)