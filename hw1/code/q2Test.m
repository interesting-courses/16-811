function q2Test

A1 = [1 -1 0; 0 2 -1; 1 0 -1/2];
A2 = [-1 1 0 0; -1 0 1 0; 0 -4 1 0; 0 -1 0 1; 0 0 -2 1];
A3 = [2 2 5; 1 1 5; 3 2 5];

[L_A1,D_A1,U_A1,P_A1,U1_A1,S1_A1,V1_A1] = q2(A1)
[L_A2,D_A2,U_A2,P_A2,U1_A2,S1_A2,V1_A2] = q2(A2)
[L_A3,D_A3,U_A3,P_A3,U1_A3,S1_A3,V1_A3] = q2(A3)

end

