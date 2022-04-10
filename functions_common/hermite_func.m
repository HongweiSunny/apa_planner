x0 = -3.847;
y0 = -0.9161
dy0 = 0.8497/180*pi;
x1 = -4.57 + 0.544 + 0.5;
y1 = -0.9
dy1 = 0;
dyy1 = tan(530/180*pi/16.68)/2.305

x = x0:0.001:x1;
y = [];
for xi = x0:0.001:x1
    y = [y hermiter_func_inline(x0, y0, dy0, x1, y1, dy1, xi)];
end
figure
plot(x,y,'k-')

pp = csape([x0, x1], [dy0 y0 y1 dy1],[dyy1 0], 'complete')
y_pp = ppval(pp,x);
hold on
plot(x, y_pp,'r-o')


pp = spline([x0, x1], [y0 y1])
y_pp = ppval(pp,x);
hold on
plot(x, y_pp,'b-.')


function y = hermiter_func_inline(x0, y0, dy0, x1, y1, dy1, x)
% 两点三次赫尔米特多项式

y = y0 * (1 -2 * (x - x0)/(x0-x1)) * ((x - x1) / (x0 - x1))^2 + ...
    y1 * (1 -2 * (x - x1)/(x1-x0)) * ((x - x0) / (x1 - x0))^2 + ...
    dy0 * (x - x0) * ((x - x1) / (x0 - x1))^2 + ...
    dy1 * (x - x1) * ((x - x0) / (x1 - x0))^2;
end