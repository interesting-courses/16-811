function q7a

ez1 = ezplot('2*x^2 + 2*y^2 - 1', [-2,2,-2,2]);
hold on
ez2 = ezplot('x^2+y^2+2*x*y-x+y', [-2,2,-2,2]);
grid on;
set(ez1,'color',[1 0 0]);
title('Zero Contours of p(x) and q(x)');
legend('2x^2 + 2y^2 = 1', 'x^2+y^2+2xy-x+y = 0');
hold off

end