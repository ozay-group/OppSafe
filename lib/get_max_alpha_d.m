function [alpha_max, C_alpha] = get_max_alpha_d(Cs, x)
% Compute the maximum and minimum alpha values with a given state x
% Inputs:
%       Cs --- (2 x n) list of inv sets and corresponding discretized alpha values
%       x --- state of the dynamics
% Outputs:
%       alpha_max --- the maximum alpha value
%       C_alpha --- inv sets 

    alphas = {1, 1, 1, 1, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0};
    Cs = [Cs; alphas];

    ub = 1;
    lb = size(Cs, 2);
    
    if ~Cs{1, lb}.contains([x; Cs{2, lb}])
%         C_alpha = NaN;
%         alpha_max = NaN;
        C_alpha = -1;
        alpha_max = -1;
        fprintf("out of bound")
        return
    end
    
    if Cs{1, ub}.contains([x; Cs{2, ub}])
        C_alpha = Cs{1, ub};
        alpha_max = Cs{2, ub};
        return
    end

    for i =ub:lb
        if Cs{1, i}.contains([x; Cs{2, i}])
            C_alpha = Cs{1, i};
            alpha_max = Cs{2, i};
            return
        end
    end
    
%     while ub < lb
%         mid = floor((ub + lb) / 2);
%         
%         if Cs{1, mid}.contains([x; Cs{2, mid}])
%             lb = mid;
%         else
%             ub = mid + 1;
%         end
%     end
%     C_alpha = Cs{1, lb};
%     alpha_max = Cs{2, lb};
end


% Cs = cell(2, 11);
% for i = 1:11
%     a = alpha(i);
%     if ~isempty(C_alpha{i})
%         Cs{1, i} = C_alpha{i} * Polyhedron('lb', a, 'ub', a);
%     end
%     Cs{2, i} = a;
% end
% Cs = fliplr(Cs);
