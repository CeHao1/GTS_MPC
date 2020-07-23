function kappa = calculate_kap(x,y)
%calculate_kap: calculate radius of curvature for 2-D path
% Description:
%     calculation of radius of curvature using cubic spline between waypoints
%     (x,y). It is assumed that the initinal and final waypoint create a 
%     non-overlapping closed loop. For details on method refer to 
%     A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp,
%     and B. Lohmann, “Minimum curvature trajectory planning and control 
%     for an autonomous race car,” Vehicle System Dynamics, Jun. 2019, 
%     Available: https://www.tandfonline.com/doi/abs/10.1080/00423114.2019.1631455.

% Inputs: 
%     x: x positions of waypoints in global coordinates
%     y: y positions of waypoints in global coordinates
%     
%     Note: the x position and y position must form a continuous closed loop.
%         The coordinates CANNOT overlap.
% Output:
%     kappa: radius of curvature for each waypoint (x,y)
    
       
    %% perform spline for optimal path
    n = length(x);
    dx = diff([x;x(1)]);
    dy = diff([y;y(1)]);
    ds = sqrt(dx.^2+dy.^2);

    % create matrices
    lowert = triu(ones(n),0) - triu(ones(n),-1);
    lowert(1,n) = -1;
    % t = 0 for reference point
    aa = eye(n); ab = zeros(n); ac = ab; ad = ab;
    % first derivatives
    ba = zeros(n);
    bb = lowert+eye(n);
    bc = lowert*2;
    bd = lowert*3;

    % second derivatives
    ca = ab; cb = ab;
    cc = bb *2;
    cd = lowert*6;

    % end point
    da = bb;
    db = lowert;
    dc = lowert;
    dd = lowert;

    A = [aa ab ac ad;
        ba bb bc bd;
        ca cb cc cd;
        da db dc dd];
    q_x= [x; zeros(3*n,1)];
    coefs_x = linsolve(A, q_x);
    q_y= [y; zeros(3*n,1)];
    coefs_y = linsolve(A, q_y);
    path.coefs_x = coefs_x;
    path.coefs_y = coefs_y;         
    q_z = zeros(4*n,1);
    coefs_z = linsolve(A, q_z);
    path.coefs_z = coefs_z;

    % extraction matrices
    A_aex = [eye(n) zeros(n) zeros(n) zeros(n)];
    A_bex = [zeros(n) eye(n) zeros(n) zeros(n)];
    A_cex = [zeros(n) zeros(n) eye(n) zeros(n)];
    A_dex = [zeros(n) zeros(n) zeros(n) eye(n)];
    
    %% calculate kappa
    xp = A_bex*coefs_x./ds;
    yp = A_bex*coefs_y./ds;
    xpp = A_cex*coefs_x./(ds.^2)*2;
    ypp = A_cex*coefs_y./(ds.^2)*2;
    
    % calculate raw kappa
    kappa1 = (xp.*ypp - yp.*xpp)./((xp.^2+yp.^2).^(3/2));
    
    % smooth with endpoints wrapped around and clip
    kappa2 = smooth([kappa1; kappa1; kappa1], length(kappa1)/100);
    kappa = kappa2(length(kappa1)+1:length(kappa1)*2);
    
end