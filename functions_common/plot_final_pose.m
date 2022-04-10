function plot_final_pose(pose)    
    x = pose(1);
    y = pose(2);
    th = pose(3);
    n = 0.72; % Front overhang  the front and back wheel axes
    m = 0.544; % Rear overhanlength
    l = 2.305; % Distance betweeng length
    b = 1.551/2; % Vehicle half width

    Ax = x + (n+l)*cos(th) - b*sin(th); Ay = y + (n+l)*sin(th) + b*cos(th);
    Bx = x + (n+l)*cos(th) + b*sin(th); By = y + (n+l)*sin(th) - b*cos(th);
    Cx = x - m*cos(th) + b*sin(th); Cy = y - m*sin(th) - b*cos(th);
    Dx = x - m*cos(th) - b*sin(th); Dy = y - m*sin(th) + b*cos(th);
    
    