function a = polyfit_5th(x0,y0,dy0,ddy0, x1,y1,dy1,ddy1)

B = [1 x0 x0^2 x0^3 x0^4 x0^5;
    0 1 2*x0 3*x0^2 4*x0^3 5*x0^4;
    0 0 2 6*x0 12*x0^2 20*x0^3;
    1 x1 x1^2 x1^3 x1^4 x1^5;
    0 1 2*x1 3*x1^2 4*x1^3 5*x1^4;
    0 0 2 6*x1 12*x1^2 20*x1^3];
Y = [y0 dy0 ddy0 y1 dy1 ddy1]';

a = inv(B)*Y;

end


