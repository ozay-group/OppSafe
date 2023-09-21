% Test the function get_max_alpha
function tests = maxAlphaTest
tests = functiontests(localfunctions);
end

%% Case 1
function testGetAlpha1(testCase)
    C =  Polyhedron('lb', [-inf;-inf;-1;-1], 'ub',[inf;inf;1;1]);
    x = [0; 0; 0];
    
    [alpha_max, alpha_min] = get_max_alpha(C, x);
    testCase.verifyTrue(alpha_max == 1)
    testCase.verifyTrue(alpha_min == -1)
end

%% Case 2
function testGetAlpha2(testCase)
    C =  Polyhedron('lb', [-inf;-inf;-1;-1], 'ub',[inf;inf;1;inf]);
    C.minHRep;
    x = [0; 0; 0];
    
    [alpha_max, alpha_min] = get_max_alpha(C, x);
    testCase.verifyTrue(alpha_max == inf)
    testCase.verifyTrue(alpha_min == -1)
end

%% Case 3
function testGetAlpha3(testCase)
    C =  Polyhedron('lb', [-inf;-inf;-1;-inf], 'ub',[inf;inf;1;1]);
    C.minHRep;
    x = [0; 0; 0];
    
    [alpha_max, alpha_min] = get_max_alpha(C, x);
    testCase.verifyTrue(alpha_max == 1)
    testCase.verifyTrue(alpha_min == -inf)
end
%% Case 4
function testGetAlpha4(testCase)
    C =  Polyhedron('lb', [-inf;-inf;-1;-inf], 'ub',[inf;inf;1;1]);
    C.minHRep;
    x = [0; 0; 2];
    testCase.verifyError(@() get_max_alpha(C, x), "getMaxAlpha:NonExistence");
end