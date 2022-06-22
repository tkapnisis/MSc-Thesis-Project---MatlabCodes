function [A,B,C,D] = Ident_Model(a21,a22,a23,a24,a41,a42,a43,a44,b2,b4,h)

A = [0 1 0 0; a21 a22 a23 a24; 0 0 0 1; a41 a42 a43 a44];
B = [0; b2; 0; b4];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

% K = zeros(4,1);
% x0 = [0; 0; 0; 0];
% h=Ts;
if h>0 % Sample the model with sample time Ts
    sys_c = ss(A, B, C, D);
    sys_d = c2d(sys_c,h);
    A = sys_d.A;
    B = sys_d.B;
    C = sys_d.C;
    D = sys_d.D;
end    

end