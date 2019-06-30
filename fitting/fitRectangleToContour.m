function [  vfMiddleEstimated, ...
    fYawEstimated, ...
    mfContourPoints, ...
    fAngleOfIntersection, ...
    bSuccess] = fitRectangleToContour(  oPCMovableLabel, ...
                                        mfAssignedPoints, ...
                                        fYawRoughEstimate, ...
                                        oAxRAN_h, bEnablePlotRANSAC, bEnablePlotNewton)
% ---------------------------------------------------------------------------------------------
% Function fitRectangleToContour(...) fits a rectangle of given size onto a point cloud. Frist, the contour points are determined.
% Second, the point clouds edges are estimated using RANSAC. Finally, target points onto those lines are defined and the
% rectangle is fitted using Newton's algorithm.

% The contour points are extracted by placing a virutal circle in the middle of the point cloud. The circle is split
% into channels and the outmost points (in relation the the circle's middle) within each channel define the contour points.
% Next, corner points are roughly estimated by averaging points that are closest to the predicted object's corners. The
% point cloud's dominant sides (one lenth side and one width side) are determined by considering the variance of all points
% of the respective side. For those two dominant sides a line is fitted using RANSAC.

% In order to fit the (predicted) rectangle onto the point cloud, three targets points are defined at the intersection
% of the dominant sides and along the fitted lines. The position of the rectangle is optimized using Newton's algorithm
% by minimizing the distance between the rectangle's target points and the defined target points along the fitted lines.

%
% INPUT:
%   oPCMovableLabel:      (Predicted) Object of class cMovablelLabel containing position information (given rectangle)
%   mfAssignedPoints:     Current points or registered fifo buffer
%   fYawRoughEstimate:    Rough estimate of the target point cloud's yaw
%   oAxRAN_h:             Axes for plotting
%   bEnablePlotRANSAC:    True if RANSAC algorithm plot is desired
%   bEnablePlotNewton:    True if Newton plot is desired
%
%
% OUTPUT:
%   vfMiddleEstimated:    Estimated middle of fitted rectangle
%   fYawEstimated:        Estimated yaw of fitted rectangle
%   mfContourPoints:      Extracted contour points
%   fAngleOfIntersection: Angle between longer and shorter estimated edge, used as confidence metric
% ---------------------------------------------------------------------------------------------
warn = warning('error', 'MATLAB:nearlySingularMatrix');

bSuccess = 1; vfMiddleEstimated = [0;0]; fYawEstimated = 0; fAngleOfIntersection = 0;
mfAssignedPoints = mfAssignedPoints(:,1:2);

if bEnablePlotRANSAC || bEnablePlotNewton
    scatter(mfAssignedPoints(:,1), mfAssignedPoints(:,2), 20, [0 0 0], '.', 'Parent', oAxRAN_h); hold on; axis equal;
end

%% Determine contour points
%  centroid = mean(assignedPoints);
middle(1,1) = oPCMovableLabel.m_fBBMiddle_x;
middle(1,2) = oPCMovableLabel.m_fBBMiddle_y;

if bEnablePlotRANSAC || bEnablePlotNewton
    scatter(middle(1,1), middle(1,2), 100, 'r', 'x', 'Parent', oAxRAN_h); hold on;
    oAxRAN_h.XLim = [middle(1,1)-3, middle(1,1)+3]; oAxRAN_h.YLim = [middle(1,2)-3, middle(1,2)+3];
end

numChannels = 72;
DPhi = 360/numChannels;

% Init test matrix
channelTest = zeros(numChannels,2); % column 1 is min, column 2 is max
for i = 1 : size(channelTest,1)
    channelTest(i,1) = (i-1)*DPhi;
    channelTest(i,2) = i * DPhi;
end

% Init collect matrices
channels = zeros(numChannels,(size(mfAssignedPoints,1)*3));
inX     = ones(numChannels,1);
inPhi   = ones(numChannels,1).*4;
ctrVec  = zeros(numChannels,1);

% Test points
for i = 1 : size(mfAssignedPoints,1)
    xC = mfAssignedPoints(i,1) - middle(1,1);
    yC = mfAssignedPoints(i,2) - middle(1,2);
    phiC = atan2(yC, xC)*180/pi();
    
    % Do a rough rotation
    Dyaw = fYawRoughEstimate - 90;
    phiC = phiC - Dyaw;
    
    if(phiC < 0)
        phiC = phiC + 360;
    elseif(phiC > 360)
        phiC = phiC - 360;
    end
    
    rC = sqrt(xC^2 + yC^2);
    
    for j = 1 : size(channelTest,1)
        if((phiC >= channelTest(j,1)) && (phiC < channelTest(j,2)))
            channels(j,inX(j):inX(j)+1) = mfAssignedPoints(i,:);
            channels(j,inPhi(j)-1) = rC;
            channels(j,inPhi(j)) = phiC;
            inX(j) = inX(j) + 4;
            inPhi(j) = inPhi(j) + 4;
            ctrVec(j) = ctrVec(j) + 1;
        end
    end
end

% Get contour points
contourPointsMax    = zeros(size(mfAssignedPoints,1),4);
contourPointsThres  = zeros(size(mfAssignedPoints,1),4);
channelMedian = zeros(numChannels,4); channelMedianCtr = zeros(numChannels,1);
minReqPoints = 3; % 3
ctrMax = 1;
ctrThres = 0;
inPhi = inPhi - ones(size(inPhi,1),1).*4;

for i = 1 : size(channels,1)
    % Get points in channel
    if ctrVec(i) >= minReqPoints
        points = channels(i, 1:inPhi(i));
        points = reshape(points, 4, [])';
        points = sortrows(points,-3);
        
        % Max: Get maxNr points with the highest distance
        maxNr = 2; num = size(points,1);
        if(num < maxNr)
            maxNr = num;
        end
        contourPointsMax(ctrMax:(ctrMax+maxNr-1),:) = points(1:maxNr,1:4);
        ctrMax = ctrMax + maxNr;
        
        if(size(points,1) > 1)
            med = median(points);
            channelMedian(i,:) = med;
        else
            channelMedian(i,:) = points;
        end
        channelMedianCtr(i) = channelMedianCtr(i) + 1;
        
        % Thresholding. Get all points above 90 % of the maxium distance within this channel
        DMaxMin = points(1,3) - points(end,3);
        for j = 1 : size(points,1)
            if points(j,3) >= DMaxMin*.9
                ctrThres = ctrThres + 1;
                contourPointsThres(ctrThres,:) = points(j,1:4);
            end
        end
    end
end
ctrMax = ctrMax - 1;
contourPointsMax = contourPointsMax(1:ctrMax,:);

if(bEnablePlotRANSAC)
    scatter(contourPointsMax(:,1), contourPointsMax(:,2), 50, [1 0 0], 'o', 'Parent', oAxRAN_h); hold on;
    %     scatter(contourPointsThres(:,1), contourPointsThres(:,2), 50, [0 0 1], '.', 'Parent', axRAN); hold on;
end
mfContourPoints = contourPointsMax;
% contourPoints = contourPointsThres;

%% Estimate corner points

fBBYaw = oPCMovableLabel.m_fBBYaw * pi()/180;
bbMiddlePlane(1,1) = oPCMovableLabel.m_fBBMiddle_x;
bbMiddlePlane(2,1) = oPCMovableLabel.m_fBBMiddle_y;
lvec(1,1) = oPCMovableLabel.m_fBBLength*cos(fBBYaw);
lvec(2,1) = oPCMovableLabel.m_fBBLength*sin(fBBYaw);
wvec(1,1) = oPCMovableLabel.m_fBBWidth*cos(fBBYaw + pi()/2);
wvec(2,1) = oPCMovableLabel.m_fBBWidth*sin(fBBYaw + pi()/2);

cornerDest(1,:) = (bbMiddlePlane + lvec - wvec)';
cornerDest(2,:) = (bbMiddlePlane + lvec + wvec)';
cornerDest(3,:) = (bbMiddlePlane - lvec + wvec)';
cornerDest(4,:) = (bbMiddlePlane - lvec - wvec)';

corners = zeros(4,4);
contourPointsSorted = mfContourPoints;
numMean = 2;
for i = 1 : 4
    contourPointsSortedCell = num2cell(contourPointsSorted, 2);
    point = cornerDest(i,:);
    distFnc = @(v) sqrt((point(1,1)-v(1,1))^2 + (point(1,2)-v(1,2))^2);
    distance = cellfun(distFnc, contourPointsSortedCell);
    contourPointsSorted(:,5) = distance;
    contourPointsSorted = sortrows(contourPointsSorted,5);
    if(size(contourPointsSorted,1) < numMean)
        corners(i,:) = mean(contourPointsSorted(:,1:4));
    else
        corners(i,:) = mean(contourPointsSorted(1:numMean,1:4));
    end
end
corner1 = corners(1,:); corner2 = corners(2,:); corner3 = corners(3,:); corner4 = corners(4,:);

if bEnablePlotRANSAC
    lw = 3;
    scatter(corner1(1,1), corner1(1,2), 200, [0 1 1], 'x', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
    scatter(corner2(1,1), corner2(1,2), 200, [0 1 1], 'x', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
    scatter(corner3(1,1), corner3(1,2), 200, [0 1 1], 'x', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
    scatter(corner4(1,1), corner4(1,2), 200, [0 1 1], 'x', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
end

%% Determine width line contour points

phiMin1 = corner1(1,4); phiMax1 = corner2(1,4);
phiMin2 = corner3(1,4); phiMax2 = corner4(1,4);

bWidth = 1;
[bWidthSide_1_dominant, mfWidthPoints] = determineDominantEdge(mfContourPoints, ...
                                                               phiMin1, phiMax1, phiMin2, phiMax2, ...
                                                               oAxRAN_h, bWidth, bEnablePlotRANSAC);

%% Determine length line contour points

phiMin1 = corner2(1,4); phiMax1 = corner3(1,4);
phiMin2 = corner4(1,4); phiMax2 = corner1(1,4);

bWidth = 0;
[bLengthSide_1_dominant, mfLengthPoints] = determineDominantEdge(mfContourPoints, ...
                                                                 phiMin1, phiMax1, phiMin2, phiMax2, ...
                                                                 oAxRAN_h, bWidth, bEnablePlotRANSAC);
                                                             
%% RANSAC to determine edge lines

[pBestLengthSide, bSuccess] = estimateLine_RANSAC(mfLengthPoints, bEnablePlotRANSAC, bEnablePlotNewton, oAxRAN_h);
if ~bSuccess
    return
end

[pBestWidthSide, bSuccess] = estimateLine_RANSAC(mfWidthPoints, bEnablePlotRANSAC, bEnablePlotNewton, oAxRAN_h);
if ~bSuccess
    return
end

%% Determine target points
l = oPCMovableLabel.m_fBBLength; w = oPCMovableLabel.m_fBBWidth;

% sA is intersection of length and width edge
grLength = pBestLengthSide(:,2); gsLength = pBestLengthSide(:,1);   % gr: direction vector, gs: support point
grWidth = pBestWidthSide(:,2);   gsWidth = pBestWidthSide(:,1);

A = [grLength(1,1), -grWidth(1,1); grLength(2,1), -grWidth(2,1)];
b = [gsWidth(1,1)-gsLength(1,1); gsWidth(2,1)-gsLength(2,1)];

try
    sol = A\b;
catch
    % no solution, stop
    bSuccess = 0;
    return;
end
sA = gsLength + sol(1,1).*grLength;
sAx = sA(1,1); sAy = sA(2,1);

% Calculate angle of intersection between both lines to as a consistency metric
fAngleOfIntersection = acos(abs(dot(grLength,grWidth))/(norm(grLength) * norm(grWidth))) * 180/pi(); % sharp angle

% sB lies on length edge, in direction of the mean length point and with the distance of the BB's length
meanLength = mean(mfLengthPoints);
cen2Mean = meanLength(1,1:2) - sA';
scal = dot(cen2Mean, grLength);
sign = scal/abs(scal);
sB = sA + sign*grLength./norm(grLength) .* l; 
sBx = sB(1,1); 
sBy = sB(2,1);

% sC lies on width edge, in direction of the mean width point and with the distance of the BB's width
meanWidth = mean(mfWidthPoints);
cen2Mean = meanWidth(1,1:2) - sA';
scal = dot(cen2Mean, grWidth);
sign = scal/abs(scal);
sC = sA + sign*grWidth./norm(grWidth) .* w;
sCx = sC(1,1); 
sCy = sC(2,1);

% Determine signs for optimization
if (bLengthSide_1_dominant)    % "left"
    if(bWidthSide_1_dominant)  % "top"
        sigPAl =  1; sigPAw =  1;
        sigPBl = -1; sigPBw =  1;
        sigPCl =  1; sigPCw = -1;
    else                        % "bottom"
        sigPAl = -1; sigPAw =  1;
        sigPBl =  1; sigPBw =  1;
        sigPCl = -1; sigPCw = -1;
    end
else
    if(bWidthSide_1_dominant)  % "top"
        sigPAl =  1; sigPAw = -1;
        sigPBl = -1; sigPBw = -1;
        sigPCl =  1; sigPCw =  1;
    else                      % "bottom"
        sigPAl = -1; sigPAw = -1;
        sigPBl =  1; sigPBw = -1;
        sigPCl = -1; sigPCw =  1;
    end
end

if(bEnablePlotNewton)
    lw = 3;
    scatter(sAx, sAy, 200, [0 1 1], 'o', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
    scatter(sBx, sBy, 200, [0 1 1], 'o', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
    scatter(sCx, sCy, 200, [0 1 1], 'o', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
end

%% Apply Newton's algorithm for optimization
%  Idea: Minimze the sum of squared distances between the target points lying on the determined edge lines and the
%  corresponding target points on the rectangle itself whose position (x, y, yaw) need to be optimized. The hessian and
%  gradient get ugly, but it works, though ;-). Feel free to add more efficient ways!

n = 25;     % plotting iterations
fi = 500;   % number of iterations

g = oPCMovableLabel.m_fBBYaw * pi()/180;
x = oPCMovableLabel.m_fBBMiddle_x;
y = oPCMovableLabel.m_fBBMiddle_y;

if (bEnablePlotNewton)
    oPCMovableLabel_draw = copy(oPCMovableLabel);
end

for i = 1 : fi
    % Plot
    if bEnablePlotNewton && (i < 150) && ((mod(i,n)==0) || (i<3) || (i==fi))
        color = [rand(1,1) rand(1,1) rand(1,1)];
        drawRectangle(oAxRAN_h, color, oPCMovableLabel_draw, 0); hold on; drawnow;
        pA(1,1) = (x + sigPAl*l/2*cos(g) + sigPAw*w/2*cos(g+pi()/2));
        pA(2,1) = (y + sigPAl*l/2*sin(g) + sigPAw*w/2*sin(g+pi()/2));
        pB(1,1) = (x + sigPBl*l/2*cos(g) + sigPBw*w/2*cos(g+pi()/2));
        pB(2,1) = (y + sigPBl*l/2*sin(g) + sigPBw*w/2*sin(g+pi()/2));
        pC(1,1) = (x + sigPCl*l/2*cos(g) + sigPCw*w/2*cos(g+pi()/2));
        pC(2,1) = (y + sigPCl*l/2*sin(g) + sigPCw*w/2*sin(g+pi()/2));
        scatter(pA(1,1), pA(2,1), 100, color, 'x', 'Parent', oAxRAN_h); hold on;
        scatter(pB(1,1), pB(2,1), 100, color, 'x', 'Parent', oAxRAN_h); hold on;
        scatter(pC(1,1), pC(2,1), 100, color, 'x', 'Parent', oAxRAN_h); hold on;
    end
    
    % Hessian, determined using symbolic toolbox 
    h31 = - l*sigPAl*sin(g) - l*sigPBl*sin(g) - sigPAw*w*sin(g + pi/2) - sigPBw*w*sin(g + pi/2);
    h32 =   l*sigPAl*cos(g) + l*sigPBl*cos(g) + sigPAw*w*cos(g + pi/2) + sigPBw*w*cos(g + pi/2);
    h33 = 2*((l*sigPAl*sin(g))/2 + (sigPAw*w*sin(g + pi/2))/2)^2 - 2*((l*sigPBl*sin(g))/2 + ...
        (sigPBw*w*sin(g + pi/2))/2)*(y - sBy + (l*sigPBl*sin(g))/2 + ...
        (sigPBw*w*sin(g + pi/2))/2) - 2*((l*sigPAl*sin(g))/2 + ...
        (sigPAw*w*sin(g + pi/2))/2)*(y - sAy + (l*sigPAl*sin(g))/2 + ...
        (sigPAw*w*sin(g + pi/2))/2) + 2*((l*sigPBl*sin(g))/2 + ...
        (sigPBw*w*sin(g + pi/2))/2)^2 - 2*((l*sigPAl*cos(g))/2 + ...
        (sigPAw*w*cos(g + pi/2))/2)*(x - sAx + (l*sigPAl*cos(g))/2 + ...
        (sigPAw*w*cos(g + pi/2))/2) - 2*((l*sigPBl*cos(g))/2 + ...
        (sigPBw*w*cos(g + pi/2))/2)*(x - sBx + (l*sigPBl*cos(g))/2 + ...
        (sigPBw*w*cos(g + pi/2))/2) + 2*((l*sigPAl*cos(g))/2 + ...
        (sigPAw*w*cos(g + pi/2))/2)^2 + 2*((l*sigPBl*cos(g))/2 + (sigPBw*w*cos(g + pi/2))/2)^2;
    
    hessian = [...
        4,      0,            - l*sigPAl*sin(g) - l*sigPBl*sin(g) - sigPAw*w*sin(g + pi/2) - sigPBw*w*sin(g + pi/2); ...
        0,      4,              l*sigPAl*cos(g) + l*sigPBl*cos(g) + sigPAw*w*cos(g + pi/2) + sigPBw*w*cos(g + pi/2); ...
        h31, h32, h33];
    
    g11 = 6*x - 2*sBx - 2*sCx - 2*sAx + l*sigPAl*cos(g) + l*sigPBl*cos(g) + l*sigPCl*cos(g) + sigPAw*w*cos(g + pi/2) ...
        + sigPBw*w*cos(g + pi/2) + sigPCw*w*cos(g + pi/2);
    g12 = 6*y - 2*sBy - 2*sCy - 2*sAy + l*sigPAl*sin(g) + l*sigPBl*sin(g) + l*sigPCl*sin(g) + sigPAw*w*sin(g + pi/2) ...
        + sigPBw*w*sin(g + pi/2) + sigPCw*w*sin(g + pi/2);
    g13 = 2*((l*sigPAl*cos(g))/2 + (sigPAw*w*cos(g + pi/2))/2)*(y - sAy + (l*sigPAl*sin(g))/2 + ...
        (sigPAw*w*sin(g + pi/2))/2) - 2*((l*sigPBl*sin(g))/2 + ...
        (sigPBw*w*sin(g + pi/2))/2)*(x - sBx + (l*sigPBl*cos(g))/2 + ...
        (sigPBw*w*cos(g + pi/2))/2) - 2*((l*sigPCl*sin(g))/2 + ...
        (sigPCw*w*sin(g + pi/2))/2)*(x - sCx + (l*sigPCl*cos(g))/2 + ...
        (sigPCw*w*cos(g + pi/2))/2) - 2*((l*sigPAl*sin(g))/2 + ...
        (sigPAw*w*sin(g + pi/2))/2)*(x - sAx + (l*sigPAl*cos(g))/2 + ...
        (sigPAw*w*cos(g + pi/2))/2) + 2*((l*sigPBl*cos(g))/2 + ...
        (sigPBw*w*cos(g + pi/2))/2)*(y - sBy + (l*sigPBl*sin(g))/2 + ...
        (sigPBw*w*sin(g + pi/2))/2) + 2*((l*sigPCl*cos(g))/2 + ...
        (sigPCw*w*cos(g + pi/2))/2)*(y - sCy + (l*sigPCl*sin(g))/2 + (sigPCw*w*sin(g + pi/2))/2);
    
    gradient = [g11; g12; g13];
    
    Dp = -hessian\gradient;
    Dp(1,1) = Dp(1,1) .* 0.2;
    Dp(2,1) = Dp(2,1) .* 0.2;
    Dp(3,1) = Dp(3,1) .* 0.01;
    
    x = x + Dp(1,1);
    y = y + Dp(2,1);
    g = g + Dp(3,1);
    
    if (bEnablePlotNewton)
        oPCMovableLabel_draw.m_fBBMiddle_x   = x;
        oPCMovableLabel_draw.m_fBBMiddle_y   = y;
        oPCMovableLabel_draw.m_fBBYaw = g*180/pi();
    end
end

if (bEnablePlotNewton)
    drawRectangle(oAxRAN_h, [0 1 0], oPCMovableLabel_draw, 0); hold on;
    axis equal
end

vfMiddleEstimated(1,1) = x;
vfMiddleEstimated(2,1) = y;
fYawEstimated          = g * 180/pi();

% Restore warning
warning(warn);

end

% This functions determines the dominant side among the two sides specified by their min and max azimuth (virtual
% circle).
function [bSide_1_dominant, mfEdgePoints] = determineDominantEdge(  mfContourPoints, ...
                                                                    phiMin1, phiMax1, phiMin2, phiMax2, ...
                                                                    oAxRAN_h, bWidth, bEnablePlotRANSAC)
    
    bSide_1_dominant = 0;   % True if side 1 is dominant (specified by phiMin1 and phiMax1)
    pointsSide1 = zeros(size(mfContourPoints,1),4);     % stores points of edge 1
    pointsSide2 = zeros(size(mfContourPoints,1),4);     % stores points of edge 2
    ctrSide_1 = 0;  % number of points on edge 1
    ctrSide_2 = 0;  % number of points on edge 2

    for i = 1 : size(mfContourPoints,1)
        if (mfContourPoints(i,4) > phiMin1) && (mfContourPoints(i,4) < phiMax1)
            ctrSide_1 = ctrSide_1 + 1;
            pointsSide1(ctrSide_1,:) = mfContourPoints(i,:);
        end
        if bWidth
            if (mfContourPoints(i,4) > phiMin2) && (mfContourPoints(i,4) < phiMax2)
                ctrSide_2 = ctrSide_2 + 1;
                pointsSide2(ctrSide_2,:) = mfContourPoints(i,:);
            end
        else
            % phiMax2 > phiMin1
            if (mfContourPoints(i,4) > phiMin2) || (mfContourPoints(i,4) < phiMax2)
                ctrSide_2 = ctrSide_2 + 1;
                pointsSide2(ctrSide_2,:) = mfContourPoints(i,:);
            end
        end
    end
    
    lower = min([ctrSide_1, ctrSide_2]);
    greater = max([ctrSide_1, ctrSide_2]);
    pointsSide1 = pointsSide1(1:ctrSide_1,:);
    pointsSide2 = pointsSide2(1:ctrSide_2,:);
    
    % If there are significant more points in one edge: done
    if greater/lower > 2    
        if ctrSide_1 > ctrSide_2
            bSide_1_dominant = 1;
            mfEdgePoints = pointsSide1;
        else
            mfEdgePoints = pointsSide2;
        end
    % They have about the same size    
    else
        % Calc variances and compare
        if (ctrSide_1 > 1)
            % Calc mean
            muR = 0;
            for i = 2 : ctrSide_1
                muR = muR + abs(pointsSide1(i,3) - pointsSide1(i-1,3));
            end
            muR = muR/(ctrSide_1-1);
            
            % Calc var
            sumSq = 0;
            for i = 2 : ctrSide_1
                ddist = abs(pointsSide1(i,3) - pointsSide1(i-1,3));
                sumSq = sumSq + (ddist-muR)^2;
            end
            varRW1 = sumSq/(ctrSide_1-2);
        elseif (ctrSide_1 > 0)
            varRW1 = pointsSide1(1,3);
        else
            varRW1 = 1000;
        end

        if (ctrSide_2 > 2)
            % Calc mean
            muR = 0;
            for i = 2 : ctrSide_2
                muR = muR + abs(pointsSide2(i,3) - pointsSide2(i-1,3));
            end
            muR = muR/(ctrSide_2-1);
            
            % Calc var
            sumSq = 0;
            for i = 2 : ctrSide_2
                ddist = abs(pointsSide2(i,3) - pointsSide2(i-1,3));
                sumSq = sumSq + (ddist-muR)^2;
            end
            varRW2 = sumSq/(ctrSide_2-2);
        elseif (ctrSide_2 > 0)
            varRW2 = pointsSide2(1,3);
        else
            varRW2 = 2000;
        end

        if varRW1 < varRW2
            bSide_1_dominant = 1;
            mfEdgePoints = pointsSide1;
        else
            mfEdgePoints = pointsSide2;
        end
    end

    if (bEnablePlotRANSAC)
        lw = 3;
        if bWidth
            scatter(mfEdgePoints(:,1), mfEdgePoints(:,2), 100, [.5 0 .5], 'o', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
        else
            scatter(mfEdgePoints(:,1), mfEdgePoints(:,2), 100, [1 .5 .5], 'o', 'Parent', oAxRAN_h, 'LineWidth', lw); hold on;
        end
    end
end

% This functions fits a line to a given set of points using RANSAC.
function [pBest, bSuccess] = estimateLine_RANSAC(mfPoints, bEnablePlotRANSAC, bEnablePlotNewton, oAxRAN_h)
    bSuccess = 1;
    % Min 5 points needed for dependability
    if (size(mfPoints,1) < 5)   
%         disp('Not enough points');
        bSuccess = 0;
        pBest = [];
        return;
    end
    
    % Error bounds
    numIt = 5;
    maxMargin = .5;
    minMargin = .05;
    stepMargin = (maxMargin-minMargin)/numIt;
    
    % Estimate
    maxNr = 30;
    for i = 1 : numIt
        pBest = [0 0; 0 0];
        distMeanBest = 10^6;
        errorMargin = maxMargin - i*stepMargin;
        num = size(mfPoints,1);
        
        if(num < maxNr)
            maxNr = num;
        end
        
        for j = 1 : maxNr
            % Randomly choose 2 points
            index1 = round((size(mfPoints,1)-1)*rand(1,1) + 1);
            index2 = round((size(mfPoints,1)-1)*rand(1,1) + 1);
            
            bSearch = 1; nStuck = 0;
            while bSearch
                nStuck = nStuck + 1;
                index2 = round((size(mfPoints,1)-1)*rand(1,1) + 1);
                               
                % Line in parameter form:
                x1 = mfPoints(index1,1); x2 = mfPoints(index2,1);
                y1 = mfPoints(index1,2); y2 = mfPoints(index2,2);
                gs = [x1;y1]; 
                gr = [(x2-x1);(y2-y1)];
                gn = [gr(2,1); -gr(1,1)];
                
                if sum(gr)
                    break;
                end
                
                if nStuck > 1000
                   bSuccess = 0;
                   pBest = [];
                   return;
                end
            end
            
            % Determine mean distance to this line
            distMean = 0;
            for k = 1 : size(mfPoints,1)
                point = mfPoints(k,1:2)';
                A = [gr(1,1), -gn(1,1); gr(2,1), -gn(2,1)];
                b = [point(1,1)-gs(1,1); point(2,1)-gs(2,1)];
                sol = A\b;
                s = gs + sol(1,1).*gr;
                distMean = distMean + (norm(s-point))^2;
            end
            
            distMean = distMean/size(mfPoints,1)/2;
            
            if (distMean < distMeanBest)
                distMeanBest = distMean;
                pBest = [gs, gr];
            end
        end
        
        % Sort out
        index = true(1, size(mfPoints,1));
        proceed = 0;
        gs = pBest(:,1); gr = pBest(:,2);
        gn = [gr(2,1); -gr(1,1)];
        
        for k = 1 : size(mfPoints,1)
            point = mfPoints(k,1:2)';
            A = [gr(1,1), -gn(1,1); gr(2,1), -gn(2,1)];
            b = [point(1,1)-gs(1,1);point(2,1)-gs(2,1)];
            sol = A\b;
            s = gs + sol(1,1).*gr;
            dist = norm(s-point);
            if(dist > errorMargin)
                index(k) = false;
            end
            if (dist > minMargin) && ~proceed
                proceed = 1;
            end
        end
        
        if (~proceed)
            break;
        end
        
        outlier = mfPoints(~index,:);
        mfPoints = mfPoints(index,:);
        
        % Plot
        if (bEnablePlotRANSAC)
            scatter(outlier(:,1), outlier(:,2), 120, 'r', '.', 'Parent', oAxRAN_h); hold on; drawnow;
            scatter(mfPoints(:,1), mfPoints(:,2), 120, 'g', '.', 'Parent', oAxRAN_h); hold on; drawnow;
            p1 = pBest(:,1);
            p2 = pBest(:,1) + 3.* pBest(:,2); p3 = pBest(:,1) + -3.* pBest(:,2);
            xline = [p1(1,1); p2(1,1); p3(1,1)]; yline = [p1(2,1); p2(2,1); p3(2,1)];
            plot(xline,yline, 'Color', [.8 .8 .8], 'Parent', oAxRAN_h);
        end
    end

    if bEnablePlotRANSAC || bEnablePlotNewton
        p1 = pBest(:,1);
        p2 = pBest(:,1) + 3.* pBest(:,2); p3 = pBest(:,1) + -3.* pBest(:,2);
        xline = [p1(1,1); p2(1,1); p3(1,1)]; yline = [p1(2,1); p2(2,1); p3(2,1)];
        plot(xline,yline, 'Color', 'g', 'Parent', oAxRAN_h);
    end
end

