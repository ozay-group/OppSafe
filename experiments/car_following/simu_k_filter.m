function [safe_rate1, safe_rate2, safe_rate3]=simu_k_filter(dyn, dyn_, Safe, C_max, tX_xu, pre_tXk, X_inf, rd_list, x0, cont, fast_mode)
% Simulate the lk with best effort control and Nan Li's safety extension RG
% (reference governer version)
% Input: 
%       dyn (Dyn) --- original dynamics
%       dyn_ (Dyn) --- alpha dynamics
%       Safe --- original safe set
%       C_max --- inv set of alpha dynamics
%       rd_list --- list of rd
%       x0 --- initial state
%       cont --- controller 
%       fast_mode --- 

%% simulation (best effort)
safe_cnt = 1;
N = length(rd_list);
X_list1 = zeros(2, N);
U_list1 = zeros(1, N-1);
X_list1(:,1) = x0;
x = x0;
for i = 1:N-1
    u = cont(x);
    if fast_mode && C_max.contains([x;1])
        % if x is in the max winning set, stop the simu
        safe_cnt = safe_cnt + N-i;
        break;
    end
    if C_max.contains([x;0])
        alpha = get_max_alpha(C_max, x);
        u_hat = alpha_filter(dyn_, C_max, x, alpha, u);
        % fprintf('alpha: %d\n', alpha);
    else
        u_hat = min(max(u,-2),2);
    end

    x = dyn.A * x + dyn.B * u_hat + dyn.Fd{1} * rd_list(i);
    X_list1(:,i+1) = x;
    U_list1(:,i) = u_hat;

    if Safe.contains(x)
        safe_cnt = safe_cnt + 1;
    else
        break;
    end
    
%     fprintf('step: %d\n', i);
end

safe_rate1 = safe_cnt / N;
%% simulation (with k filter)
safe_cnt = 1;
% input_cnt = 0;

X_list2 = zeros(2, N);
U_list2 = zeros(1, N-1);
X_list2(:,1) = x0;
x = x0;
for i = 1:N-1    
    u = cont(x);
    if fast_mode && X_inf.contains(x)
        % if x is in the max winning set, stop the simu
        safe_cnt = safe_cnt + N-i;
        break;
    end
    u = min(max(u,-2),2);
    u_hat = k_filter_tXk(tX_xu, pre_tXk, x, u);
    
    x = dyn.A * x + dyn.B * u_hat + dyn.Fd{1} * rd_list(i);
    X_list2(:,i+1) = x;
    U_list2(:,i) = u_hat;

    if Safe.contains(x)
        safe_cnt = safe_cnt + 1;
    else
        break;
    end
    
%     fprintf('step: %d\n', i);
end

safe_rate2 = safe_cnt / N;

%% simulation (without best effort)
safe_cnt = 1;
% input_cnt = 0;

X_list3 = zeros(2, N);
U_list3 = zeros(1, N-1);
X_list3(:,1) = x0;
x = x0;
for i = 1:N-1
    u = cont(x);

    if C_max.contains([x;1])
        if fast_mode
            safe_cnt = safe_cnt + N-i;
            break;
        else
            u_hat = alpha_filter(dyn_, C_max, x, 1, u);
%             fprintf('alpha: %d\n', 1);
        end
    else
        u_hat = min(max(u,-2), 2);
        % if u~=u_hat
        %     input_cnt = input_cnt+1;
        % end
    end
    
    x = dyn.A * x + dyn.B * u_hat + dyn.Fd{1} * rd_list(i);
    X_list3(:,i+1) = x;
    U_list3(:,i) = u_hat;

    if Safe.contains(x)
        safe_cnt = safe_cnt + 1;
    else
        break;
    end
    
%     fprintf('step: %d\n', i);
end

safe_rate3 = safe_cnt / N;
% disp("input violation:"+num2str(input_cnt))

% save("data/vis_data")