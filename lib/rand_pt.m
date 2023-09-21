function pt = rand_pt(P, res_P)
% Generate a random point inside the given Polyhedron via "Hit-and-Run"
% Inputs: 
%           P --- target polyhedron
%           P --- restricted polyhedron(optional)

    if ~P.hasVRep()
        P.computeVRep(); 
    end
    
    lb = min(P.V)';
    ub = max(P.V)';

    while true
        pt = lb + rand(size(lb)).*(ub - lb);
        
        if P.contains(pt) && ~res_P.contains(pt)
            break;
        end
    end
end