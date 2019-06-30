function [vfEllipseParameters, bSuccess] = fitEllipseToContour(oPCMovableLabel, mfContourPoints, bLSSolution, oAxEll_h)
% ---------------------------------------------------------------------------------------------
% Function fitEllipseToContour(...) fits an ellipse to given contour points of a point cloud. 
% The algorithm is based on a Gauss-Newton algorithm that minimizes the squared error of each contour point to an
% optimal ellipse. 
%
% INPUT:
%   oPCMovableLabel:       Object of class cPCMovableLabel containing an the estimation for starting values
%   mfContourPoints:       Matrix containing the contour points of the point cloud
%   bLSSolution:           True if a least squares based algorithm should be used
%   oAxEll_h:              Plotting axes
%
% OUTPUT:
%   vfEllipseParameters:   Vector containing the estimated ellipse parameters
%   bSuccess:              True if fitting was successfull
% ---------------------------------------------------------------------------------------------

% Make warning temporarily an error 
warn = warning('error', 'MATLAB:nearlySingularMatrix');

bEnablePlot = 0;
if (~isempty(oAxEll_h))
    bEnablePlot = 1;
end

bSuccess = 1; vfEllipseParameters = [];
x = mfContourPoints(:,1)';
y = mfContourPoints(:,2)';
numPoints = size(mfContourPoints,1);

if size(mfContourPoints, 1) < 10
    bSuccess = 0;
    return
end

%% Least squares solution
if (bLSSolution)
    ellipse_t = fit_ellipse(x, y, oAxEll_h);
    if isempty(ellipse_t)
        bSuccess = 0;
    else
        yaw = -ellipse_t.phi;
        a = ellipse_t.a;
        b = ellipse_t.b;
        x0 = ellipse_t.X0_in;
        y0 = ellipse_t.Y0_in;
        if (bEnablePlot)
            phi_o = linspace(0,2*pi(),numPoints);
            x_s = repmat(x0, [1,numPoints]) + a.*cos(phi_o).*cos(yaw)-b.*sin(phi_o).*sin(yaw);
            y_s = repmat(y0, [1,numPoints]) + a.*cos(phi_o).*sin(yaw)+b.*sin(phi_o).*cos(yaw);
            plot(x_s, y_s, 'b', 'Parent', oAxEll_h, 'LineWidth', 5); hold on; drawnow;
        end
        yaw = yaw * 180/pi();
        if(yaw < 0)
            yaw = yaw + 360;
        elseif(yaw >= 360)
            yaw = yaw - 360;
        end
        vfEllipseParameters = [yaw; a; b; x0; y0];
    end
    return;
end

if(numPoints <= 5)
    bSuccess = 0;
    return;
end

if (bEnablePlot)
    scatter(x,y, 250, [0 0 0], '.', 'Parent', oAxEll_h); hold on;
    xlim([oPCMovableLabel.m_fBBMiddle_x-5,oPCMovableLabel.m_fBBMiddle_x+5]); ylim([oPCMovableLabel.m_fBBMiddle_y-5,oPCMovableLabel.m_fBBMiddle_y+5]);
end

%% Gauss-Newton
%  Starting values
yawStart    = oPCMovableLabel.m_fBBYaw*pi()/180;
addPi = 0; subPi = 0;

% Map yaw starting values for convergence reasons
if (yawStart > pi()/2)
    yawStart = yawStart - pi();
    addPi = 1;
elseif(yawStart < -pi()/2)
    yawStart = yawStart + pi();
    subPi = 1;
end

x0Start     = oPCMovableLabel.m_fBBMiddle_x;
y0Start     = oPCMovableLabel.m_fBBMiddle_y;
aStart      = oPCMovableLabel.m_fBBLength;
bStart      = oPCMovableLabel.m_fBBWidth;
phiStart = linspace(0,2*pi(),numPoints);

% Parameter vector
u = [phiStart'; yawStart; aStart; bStart; x0Start; y0Start];

if (bEnablePlot)
    x_s = repmat(x0Start, [1,numPoints]) + aStart.*cos(phiStart).*cos(yawStart)-bStart.*sin(phiStart).*sin(yawStart);
    y_s = repmat(y0Start, [1,numPoints]) + aStart.*cos(phiStart).*sin(yawStart)+bStart.*sin(phiStart)*cos(yawStart);
    color = [rand(1,1) rand(1,1) rand(1,1)];
    h = scatter(x_s, y_s, 30, color, 'x', 'Parent', oAxEll_h, 'LineWidth', 1); hold on; drawnow;
end

numIt = 300; n = 30;
for it = 1 : numIt;
    phi = u(1:numPoints,1)';
    yaw = u(numPoints+1,1);
    a   = u(numPoints+2,1);
    b   = u(numPoints+3,1);
    x0  = u(numPoints+4,1);
    y0  = u(numPoints+5,1);

    if (bEnablePlot && ((mod(it,n)==0) || (it<3) || (it==numIt)))
        x_s = repmat(x0, [1,numPoints]) + a.*cos(phi).*cos(yaw)-b.*sin(phi).*sin(yaw);
        y_s = repmat(y0, [1,numPoints]) + a.*cos(phi).*sin(yaw)+b.*sin(phi)*cos(yaw);
        color = [rand(1,1) rand(1,1) rand(1,1)];
        set(h, 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
        set(h, 'XData', x_s);
        set(h, 'YData', y_s);
        h2 = scatter(x_s, y_s, 30, color, 'x', 'Parent', oAxEll_h, 'LineWidth', 2); hold on; drawnow;
        drawnow;
        % pause(0.10);
    end
    
    %% Build Jacobian
    J = zeros(2*numPoints, numPoints+5);
    for i = 1 : size(J,1)/2
        for j = 1 : size(J,2)
                % phi
            if(j <= size(x,2))
                if(j == i)
                    J(i*2-1,j) = -(-sin(phi(1,i))*a*cos(yaw) - b*cos(phi(1,i))*sin(yaw));
                    J(i*2,  j) = -(-sin(phi(1,i))*a*sin(yaw) + b*cos(phi(1,i))*cos(yaw));
                else
                    continue;
                end
                % yaw
            elseif(j == numPoints+1)
                J(i*2-1,j) = -(a*cos(phi(1,i))*-sin(yaw)- b*sin(phi(1,i))*cos(yaw));
                J(i*2,  j) = -(a*cos(phi(1,i))*cos(yaw) + b*sin(phi(1,i))*-sin(yaw));
                % a
            elseif(j == numPoints+2)
                J(i*2-1,j) = -(cos(phi(1,i)*cos(yaw)));
                J(i*2,  j) = -(cos(phi(1,i)*sin(yaw)));
                % b
            elseif(j == numPoints+3)
                J(i*2-1,j) = -(-sin(phi(1,i))*sin(yaw));
                J(i*2,  j) = -( sin(phi(1,i))*cos(yaw));
                % x0
            elseif(j == numPoints+4)
                J(i*2-1,j) = -1;
                J(i*2  ,j) = 0;
                % y0
            elseif(j == numPoints+5)
                J(i*2-1,j) = 0;
                J(i*2  ,j) = -1;
            end
        end
    end 
    
    %% Score vector
    score = zeros(2*numPoints,1);
    for i = 1 : numPoints
       xs = x0 + a*cos(u(i,1))*cos(yaw) - b*sin(u(i,1))*sin(yaw);
       ys = y0 + a*cos(u(i,1))*sin(yaw) + b*sin(u(i,1))*cos(yaw);
       score(i*2-1,1)    = x(1,i) - xs;
       score(i*2  ,1)    = y(1,i) - ys;
    end
    
    %% New parameter vector
    %  Pseudo inverse jacobian
    try
        Jpsinv = (J'*J)\J';
    catch
        % No pseudo inverse, no convergence, stop
        bSuccess = 0;
        return;
    end
   
    dk = -Jpsinv*score;
    
    % Convergence
    if(max(abs(dk)) < 0.01)
        break;
    end
    
    % Damping
    dk(numPoints+1) = dk(numPoints+1)*0.1;
    dk(numPoints+2) = dk(numPoints+2)*0.1;
    dk(numPoints+3) = dk(numPoints+3)*0.1;
    dk(numPoints+4) = dk(numPoints+4)*0.1;
    dk(numPoints+5) = dk(numPoints+5)*0.1;
    
    u = u + dk;
end

if (bEnablePlot)
    phi_o = linspace(0,2*pi(),numPoints);
    x_s = repmat(x0, [1,numPoints]) + a.*cos(phi_o).*cos(yaw)-b.*sin(phi_o).*sin(yaw);
    y_s = repmat(y0, [1,numPoints]) + a.*cos(phi_o).*sin(yaw)+b.*sin(phi_o).*cos(yaw);
    plot(x_s, y_s, 'g', 'Parent', oAxEll_h); hold on; drawnow;
end

yaw = u(numPoints+1,1)*180/pi();
a   = u(numPoints+2,1);
b   = u(numPoints+3,1);
x0  = u(numPoints+4,1);
y0  = u(numPoints+5,1);

% Apply mapping
if(addPi)
    yaw = yaw + 180;
elseif(subPi)
    yaw = yaw - 180;
end

while(yaw >= 360)
    yaw = yaw - 360;
end
while(yaw < 0)
    yaw = yaw + 360;
end

vfEllipseParameters = [yaw; a; b; x0; y0];

% Restore warning
warning(warn);
end

