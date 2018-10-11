function q7c

ez1 = ezplot('2*x^2 + 2*y^2 - 1', [-2,2,-2,2]);
hold on
ez2 = ezplot('x^2+y^2+2*x*y-x+y', [-2,2,-2,2]);
grid on;
set(ez1,'color',[1 0 0]);

x = -0.08405869;
y = -0.7020926:0.05:0.3;
l1 = line([x x],[y(1) y(end)], 'Color', 'm', 'LineStyle', '--');

x = 0.70209268;
y = -0.3:0.005:0.7020926;
l2 = line([x x],[y(1) y(end)], 'Color', 'k', 'LineStyle', '--');

title('Zero Contours of p(x) and q(x) with roots marked ');
legend('2x^2 + 2y^2 = 1', 'x^2+y^2+2xy-x+y = 0', 'x=-0.084', 'x=0.702');
hold off

end
