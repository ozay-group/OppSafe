% Test the function alpha_filter
function tests = alphaFilterTest
    tests = functiontests(localfunctions);
end

%% Case 1: 1-dimensional input
function test1DInputOutside(testCase)
    C =  Polyhedron('lb', [-1;-1; 0], 'ub',[1; 1; 1]);
    x = [0; 0];
    
    A = [0 1;
        0 0];
    B = [0
        1];
    XU = Polyhedron('lb', [-inf;-inf; -1], 'ub', [inf;inf; 1]);
    D = 0.1*Polyhedron('lb',[-1;-1], 'ub',[1;1]);
   
    dyn = Dyn(A, zeros(2,1), B, XU, [], [], [], {zeros(2), zeros(2)}, {[1;0], [0;1]}, D);
    dyn.check()
    dyn_ = alpha_dyn(dyn, C);
    for alpha = 0:0.1:1
        u_out = alpha_filter(dyn_, C, x, alpha, 2);
        testCase.verifyTrue(abs(u_out - min(1-0.1*alpha+0.1*(1-alpha),1))< 1e-10)
    end
end

function test1DInputInside(testCase)
    C =  Polyhedron('lb', [-1;-1; 0], 'ub',[1; 1; 1]);
    x = [0; 0];
    
    A = [0 1;
        0 0];
    B = [0
        1];
    XU = Polyhedron('lb', [-inf;-inf; -1], 'ub', [inf;inf; 1]);
    D = 0.1*Polyhedron('lb',[-1;-1], 'ub',[1;1]);
   
    dyn = Dyn(A, zeros(2,1), B, XU, [], [], [], {zeros(2), zeros(2)}, {[1;0], [0;1]}, D);
    dyn.check()
    dyn_ = alpha_dyn(dyn, C);
    for alpha = 0:0.1:1
        u_out = alpha_filter(dyn_, C, x, alpha, 0.4);
        testCase.verifyTrue(u_out==0.4)
    end
end

%% Case 2: 2-dimensional input
function test2DInputOutside(testCase)
    C =  Polyhedron('lb', [-1;-1; 0], 'ub',[1; 1; 1]);
    x = [0; 0];
    A = eye(2);
    B = eye(2);
    XU = Polyhedron('lb', [-inf;-inf; -1; -1], 'ub', [inf;inf; 1; 1]);
    D = 0.1*Polyhedron('lb',[-1;-1], 'ub',[1;1]);
   
    dyn = Dyn(A, zeros(2,1), B, XU, [], [], [], {zeros(2), zeros(2)}, {[1;0], [0;1]}, D);
    dyn.check()
    dyn_ = alpha_dyn(dyn, C);
    for alpha = 0:0.1:1
        u_out = alpha_filter(dyn_, C, x, alpha, [2;2]);
        testCase.verifyTrue(all(abs(u_out - min(1-0.1*alpha+0.1*(1-alpha),1)*[1;1])< 1e-6))
    end
end

function test2DInputInside(testCase)
    C =  Polyhedron('lb', [-1;-1; 0], 'ub',[1; 1; 1]);
    x = [0; 0];
    A = eye(2);
    B = eye(2);
    XU = Polyhedron('lb', [-inf;-inf; -1; -1], 'ub', [inf;inf; 1; 1]);
    D = 0.1*Polyhedron('lb',[-1;-1], 'ub',[1;1]);
   
    dyn = Dyn(A, zeros(2,1), B, XU, [], [], [], {zeros(2), zeros(2)}, {[1;0], [0;1]}, D);
    dyn.check()
    dyn_ = alpha_dyn(dyn, C);
    for alpha = 0:0.1:1
        u_out = alpha_filter(dyn_, C, x, alpha, [0.5;0.5]);
        testCase.verifyTrue(all(abs(u_out - [0.5;0.5])< 1e-6))
    end
end
