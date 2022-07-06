syms g11 g12 g13 g21 g22 g23 g31 g32 g33 W11 W12 W13 W21 W22 W23 W31 W32 W33
syms D11 D12 D13 D21 D22 D23 D31 D32 D33

% Gp = [g11*(1+W11*D11), g12*(1+W12*D12); g21*(1+W21*D21), g22*(1+W22*D22)]
G = [g11, g12, g13; g21, g22, g23; g31, g32, g33];

% Gun = [g11*W11*D11, g12*W12*D12; g21*W21*D21, g22*W22*D22]

D = diag([D11 D21 D31 D12 D22 D32 D13 D23 D33]);

aux_mat = [eye(3),eye(3),eye(3)];

G_diag = reshape(G,[],1);
G_diag = diag(G_diag);
W = [W11,   0,   0, W12,   0,   0, W13,   0,   0;...
       0, W21,   0,   0, W22,   0,   0, W23,   0;...
       0,   0, W31,   0,   0, W32,   0,   0, W33];


mat = [ones(3,1), zeros(3,1), zeros(3,1);...
       zeros(3,1),  ones(3,1), zeros(3,1);...
       zeros(3,1), zeros(3,1),  ones(3,1)];

mat2 = [1 0 0;...
        0 1 0;...
        0 0 1;...
        0 0 0;...
        0 0 0;...
        0 0 0;...
        0 0 0;...
        0 0 0;...
        0 0 0];


%%
rhs = mat'*D*W';

%%
W = [W11, W12; W21, W22];
W = [W11,   0, W12,   0;...
       0, W21,   0, W22]';

G = [g11, g12; g21, g22];

aux1 = [1 0;0 0;0 1;0 0];
aux2 = [zeros(2,2);eye(2)];
% res = G*aux1;

mat = [ones(2,1), zeros(2,1);...
       zeros(2,1),  ones(2,1)]';
%%
% res11 = [G(1,1),0; 0 0];
% res12 = [0,0; 0 G(1,2)];