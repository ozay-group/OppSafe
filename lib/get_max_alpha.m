function [alpha_max, alpha_min] = get_max_alpha(C, x)
% Compute the maximum and minimum alpha values with a given state x
% Inputs:
%       C --- inv set of alpha dynamics
%       x --- state of the dynamics
% Outputs:
%       alpha_max --- the maximum alpha value
%       alpha_min --- the minimum alpha value

    if ~C.hasHRep()
        C.computeHRep();
    end

    A = C.A(:, end);
    b = C.b - C.A(:, 1:end-1) * x;

    pos_ind = A>0;
    zero_ind = A==0;
    neg_ind = A<0;
    alpha_max = inf;
    alpha_min = -inf;
    if any(pos_ind)
        alpha_max = min(b(pos_ind)./A(pos_ind));
    end
    if any(neg_ind)
        alpha_min = max(b(neg_ind)./A(neg_ind));
    end

    if (alpha_max < alpha_min) || (any(b(zero_ind)<-1e-6))
%         error("getMaxAlpha:NonExistence","A valid alpha does not exist!");
        alpha_max = -inf;
        alpha_min = -inf;
    end

end



    
