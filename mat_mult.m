%%
%4 ii)
syms Z_a V M_a Z_d M_d K_a K_q
%Paramters
V = 886.78;
Z_a = -1.3046*V;
Z_d = -0.2142*V;
M_a = 47.4109;
M_d = -104.8346;
K_a=-.0015;
K_q=-0.32;

%System matrices
A_p = [Z_a/V 1; M_a, 0];
B_p = [Z_d/V M_d].';
C_p = [Z_a 0; 0 1];
D_p = [Z_d 0].';

%Controller Matrices
D_c1 = [-K_a*K_q -K_q];
D_c2 = [K_a*K_q];
Z_matrix = 1 - D_c1*D_p;
C_c = [K_q 1];

x_term = A_p + (B_p*(Z_matrix^-1))*(D_c1*C_p); 
xc_term = B_p*(Z_matrix^-1)*C_c;
r_term = B_p*(Z_matrix^-1)*D_c2;