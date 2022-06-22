syms x y z

S =  [x*y 2 3 4; y 6 7 8; 9 10 11 z];
%%
x = ureal('x',29.38,'Percentage',[-20 20]);
y = ureal('y',29.38,'Percentage',[-20 20]);
z = ureal('z',29.38,'Percentage',[-20 20]);
%%

%%
res = arrayfun(@char, S, 'UniformOutput', 0);
string_res = string(res);
%%
for i=1:size(string_res,1)
    for j=1:size(string_res,2)
        res_umat(i,j) = eval(string_res(i,j));
    end
end

%%
% x=6;
x = ureal('x',29.38,'Percentage',[-20 20]);
y = ureal('y',29.38,'Percentage',[-20 20]);
z=0;
res  = ht(x,y)
%%

%%
ht = matlabFunction(symfun_S)

%%
res = umat(ht(x,y,z))