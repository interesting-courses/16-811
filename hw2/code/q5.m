function [root, val] = q5(f, X0)
% f = function for which to find root
% X0 = initial approximations of x
% root = actual root closest to initial approximations
% val = value of the function at the root

xtol = 1e-5;
ftol = 1e-5;
iter = 1e3;

if X0(1) == X0(2) || X0(1) == X0(3) || X0(2) == X0(3)
    error('The inital 3 estimates must be different');
end

x0 = X0(1);
x1 = X0(2);
x2 = X0(3);

f0 = feval (f, x0);
f1 = feval (f, x1);
f2 = feval (f, x2);

for num_iter = 1:iter
    c = f2;
    d0 = f0-f2;
    d1 = f1-f2;
    h0 = x0-x2;
    h1 = x1-x2;
    b = (d1*h0^2 - d0*h1^2)/(h0*h1*(h0-h1));
    a = (d0*h1 - d1*h0)/(h0*h1*(h0-h1));
    
    if (a ~= 0)
        det = b^2 - 4*a*c;
        val1 = (b + sqrt(det));
        val2 = (b - sqrt(det));
        if b > 0
            x3 = x2 - (2*c/val1);
        else
            x3 = x2 - (2*c/val2);
        end
    elseif (b~=0)
        x3 = x2 - (c/b);
    else
        root = x2;
        val = f2;
        return
    end
    
    f3 = feval(f, x3);
    if (abs(x3-x2) < xtol || abs(f3) < ftol)
        root = x3;
        val = f3;
        return
    end
    
    x0 = x1;
    x1 = x2;
    x2 = x3;
    
    f0 = f1;
    f1 = f2;
    f2 = f3;
end

root = x2;
val = f2;

end