function [ middleEstimated, yawEstimated, contourPoints, angleOfIntersection, success] = estimateBoxPC_RANSAC_Newton( movableData, assignedPoints, yawRegistered, axRAN, enablePlotRANSAC, enablePlotNewton)

success = 1; middleEstimated = [0;0]; yawEstimated = 0; angleOfIntersection = 0;
assignedPoints = assignedPoints(:,1:2);

if(enablePlotRANSAC || enablePlotNewton)
    scatter(assignedPoints(:,1), assignedPoints(:,2), 20, [0 0 0], '.', 'Parent', axRAN); hold on; axis equal;
end
%% determine contour points
% centroid = mean(assignedPoints);
middle(1,1) = movableData.m_BBMiddle_x;
middle(1,2) = movableData.m_BBMiddle_y;

if(enablePlotRANSAC || enablePlotNewton)
    scatter(middle(1,1), middle(1,2), 100, 'r', 'x', 'Parent', axRAN); hold on;
    axRAN.XLim = [middle(1,1)-3, middle(1,1)+3]; axRAN.YLim = [middle(1,2)-3, middle(1,2)+3];
end

numChannels = 72;
DPhi = 360/numChannels;

% init test matrix
channelTest = zeros(numChannels,2); % column 1 is min, column 2 is max
for i = 1 : size(channelTest,1)
   channelTest(i,1) = (i-1)*DPhi;
   channelTest(i,2) = i * DPhi;
end

% init collect matrices
channels = zeros(numChannels,(size(assignedPoints,1)*3));
inX     = ones(numChannels,1);
inPhi   = ones(numChannels,1).*4;
ctrVec  = zeros(numChannels,1);
% test points
for i = 1 : size(assignedPoints,1)
    xC = assignedPoints(i,1) - middle(1,1);
    yC = assignedPoints(i,2) - middle(1,2);
    phiC = atan2(yC, xC)*180/pi();
    % do a rough rotation
    Dyaw = yawRegistered - 90;
    phiC = phiC - Dyaw;
    if(phiC < 0)
        phiC = phiC + 360;
    elseif(phiC > 360)
        phiC = phiC - 360; 
    end
    rC = sqrt(xC^2 + yC^2);
    for j = 1 : size(channelTest,1)
        if((phiC >= channelTest(j,1)) && (phiC < channelTest(j,2)))
           channels(j,inX(j):inX(j)+1) = assignedPoints(i,:); 
           channels(j,inPhi(j)-1) = rC;
           channels(j,inPhi(j)) = phiC;
           inX(j) = inX(j) + 4;
           inPhi(j) = inPhi(j) + 4;
           ctrVec(j) = ctrVec(j) + 1;
        end
    end
end

% get countour points
contourPointsMax    = zeros(size(assignedPoints,1),4);
contourPointsThres  = zeros(size(assignedPoints,1),4);
channelMedian = zeros(numChannels,4); channelMedianCtr = zeros(numChannels,1);
minReqPoints = 3; % 3
ctrMax = 1;
ctrThres = 0;
inPhi = inPhi - ones(size(inPhi,1),1).*4;
for i = 1 : size(channels,1)
    % get points in channel
    if(ctrVec(i) >= minReqPoints)
        points = channels(i, 1:inPhi(i));
        points = reshape(points, 4, [])';
        points = sortrows(points,-3);  
        % max
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
        % thresholding
        DMaxMin = points(1,3) - points(end,3);
        for j = 1 : size(points,1)
            if(points(j,3) >= DMaxMin*.9)
                ctrThres = ctrThres + 1;
                contourPointsThres(ctrThres,:) = points(j,1:4);
            end
        end
    end
end
ctrMax = ctrMax - 1;
contourPointsMax = contourPointsMax(1:ctrMax,:);

if(enablePlotRANSAC)
    scatter(contourPointsMax(:,1), contourPointsMax(:,2), 50, [1 0 0], 'o', 'Parent', axRAN); hold on;
%     scatter(contourPointsThres(:,1), contourPointsThres(:,2), 50, [0 0 1], '.', 'Parent', axRAN); hold on;
end
contourPoints = contourPointsMax;
% contourPoints = contourPointsThres;
%% estimate corner points
m_BBYaw = movableData.m_BBYaw * pi()/180;
bbMiddlePlane(1,1) = movableData.m_BBMiddle_x;
bbMiddlePlane(2,1) = movableData.m_BBMiddle_y;
lvec(1,1) = movableData.m_BBLength*cos(m_BBYaw);
lvec(2,1) = movableData.m_BBLength*sin(m_BBYaw);
wvec(1,1) = movableData.m_BBWidth*cos(m_BBYaw + pi()/2);
wvec(2,1) = movableData.m_BBWidth*sin(m_BBYaw + pi()/2);

cornerDest(1,:) = (bbMiddlePlane + lvec - wvec)';
cornerDest(2,:) = (bbMiddlePlane + lvec + wvec)';
cornerDest(3,:) = (bbMiddlePlane - lvec + wvec)';
cornerDest(4,:) = (bbMiddlePlane - lvec - wvec)';

corners = zeros(4,4);
contourPointsSorted = contourPoints;
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
if(enablePlotRANSAC)
    lw = 3;
    scatter(corner1(1,1), corner1(1,2), 200, [0 1 1], 'x', 'Parent', axRAN, 'LineWidth', lw); hold on;
    scatter(corner2(1,1), corner2(1,2), 200, [0 1 1], 'x', 'Parent', axRAN, 'LineWidth', lw); hold on;
    scatter(corner3(1,1), corner3(1,2), 200, [0 1 1], 'x', 'Parent', axRAN, 'LineWidth', lw); hold on;
    scatter(corner4(1,1), corner4(1,2), 200, [0 1 1], 'x', 'Parent', axRAN, 'LineWidth', lw); hold on; 
end

%% determine width line contour points
widthSided1Dominant = 0;
widthSided1 = zeros(size(contourPoints,1),4);
widthSided2 = zeros(size(contourPoints,1),4);
ctrWidthSided1 = 0; ctrWidthSided2 = 0;

phiMin1 = corner1(1,4); phiMax1 = corner2(1,4);
phiMin2 = corner3(1,4); phiMax2 = corner4(1,4);

for i = 1 : size(contourPoints,1)
    if((contourPoints(i,4) > phiMin1) && (contourPoints(i,4) < phiMax1))
        ctrWidthSided1 = ctrWidthSided1 + 1;
        widthSided1(ctrWidthSided1,:) = contourPoints(i,:);
    end
    if((contourPoints(i,4) > phiMin2) && (contourPoints(i,4) < phiMax2))
        ctrWidthSided2 = ctrWidthSided2 + 1;
        widthSided2(ctrWidthSided2,:) = contourPoints(i,:);
    end
end
lower = min([ctrWidthSided1, ctrWidthSided2]);
greater = max([ctrWidthSided1, ctrWidthSided2]);
widthSided1 = widthSided1(1:ctrWidthSided1,:);
widthSided2 = widthSided2(1:ctrWidthSided2,:);
if(greater/lower > 2) % 10 % more points compared to lower
    if(ctrWidthSided1 > ctrWidthSided2)
        widthSided1Dominant = 1;
        widthPoints = widthSided1;
    else
        widthPoints = widthSided2;
    end
else % they have about the same size
    % calc variances and compare
    if(ctrWidthSided1 > 1)
        % calc mean
        muR = 0;
        for i = 2 : ctrWidthSided1
            muR = muR + abs(widthSided1(i,3) - widthSided1(i-1,3));
        end
        muR = muR/(ctrWidthSided1-1);
        % calc var
        sumSq = 0;
        for i = 2 : ctrWidthSided1
            ddist = abs(widthSided1(i,3) - widthSided1(i-1,3));
            sumSq = sumSq + (ddist-muR)^2;
        end
        varRW1 = sumSq/(ctrWidthSided1-2);
    elseif(ctrWidthSided1 > 0)
        varRW1 = widthSided1(1,3);
    else
        varRW1 = 1000;
    end
    
    if(ctrWidthSided2 > 2)
         % calc mean
        muR = 0;
        for i = 2 : ctrWidthSided2
            muR = muR + abs(widthSided2(i,3) - widthSided2(i-1,3));
        end
        muR = muR/(ctrWidthSided2-1);
        % calc var
        sumSq = 0;
        for i = 2 : ctrWidthSided2
            ddist = abs(widthSided2(i,3) - widthSided2(i-1,3));
            sumSq = sumSq + (ddist-muR)^2;
        end
        varRW2 = sumSq/(ctrWidthSided2-2);
    elseif(ctrWidthSided2 > 0)
        varRW2 = widthSided2(1,3);
    else
        varRW2 = 2000;
    end
    
    if(varRW1 < varRW2)
        widthSided1Dominant = 1;
        widthPoints = widthSided1;
    else
        widthPoints = widthSided2;
    end
end

if(enablePlotRANSAC)
    scatter(widthPoints(:,1), widthPoints(:,2), 100, [.5 0 .5], 'o', 'Parent', axRAN, 'LineWidth', lw); hold on;
end
%% determine length line contour points
lengthSided1Dominant = 0; 
lengthSided1 = zeros(size(contourPoints,1),4);
lengthSided2 = zeros(size(contourPoints,1),4);
ctrLengthSided1 = 0; ctrLengthSided2 = 0;

phiMin1 = corner2(1,4); phiMax1 = corner3(1,4);
phiMin2 = corner4(1,4); phiMax2 = corner1(1,4);
for i = 1 : size(contourPoints,1)
    if((contourPoints(i,4) > phiMin1) && (contourPoints(i,4) < phiMax1))
        ctrLengthSided1 = ctrLengthSided1 + 1;
        lengthSided1(ctrLengthSided1,:) = contourPoints(i,:);
    end
    if((contourPoints(i,4) > phiMin2) || (contourPoints(i,4) < phiMax2))
        ctrLengthSided2 = ctrLengthSided2 + 1;
        lengthSided2(ctrLengthSided2,:) = contourPoints(i,:);
    end
end
lengthSided1 = lengthSided1(1:ctrLengthSided1,:); lengthSided1 = sortrows(lengthSided1, 4);
lengthSided2 = lengthSided2(1:ctrLengthSided2,:); lengthSided2 = sortrows(lengthSided2, 4);
lower = min([ctrLengthSided1, ctrLengthSided2]);
greater = max([ctrLengthSided1, ctrLengthSided2]);
if(greater/lower > 2) %  more points in 1 or 2
    if(ctrLengthSided1 > ctrLengthSided2)
        lengthSided1Dominant = 1;
        lengthPoints = lengthSided1;
    else
        lengthPoints = lengthSided2;
    end
else % check variance
    if(ctrLengthSided1 > 1)
        % calc mean
        muR = 0;
        for i = 2 : size(lengthSided1,1)
            muR = muR + abs(lengthSided1(i,3) - lengthSided1(i-1,3));
        end
        muR = muR/(ctrLengthSided1-1);
        % calc var
        sumSq = 0;
        for i = 2 : size(lengthSided1,1)
            ddist = abs(lengthSided1(i,3) - lengthSided1(i-1,3));
            sumSq = sumSq + (ddist-muR)^2;
        end
        varRL1 = sumSq/(ctrLengthSided1-2);
    elseif(ctrLengthSided1 > 0)
        varRL1 = lengthSided1(1,3);
    else
        varRL1 = 1000;
    end
    if(ctrLengthSided2 > 1)
        % calc mean
        muR = 0;
        for i = 2 : size(lengthSided2,1)
            muR = muR + abs(lengthSided2(i,3) - lengthSided2(i-1,3));
        end
        muR = muR/(ctrLengthSided2-1);
        % calc var
        sumSq = 0;
        for i = 2 : size(lengthSided2,1)
            ddist = abs(lengthSided2(i,3) - lengthSided2(i-1,3));
            sumSq = sumSq + (ddist-muR)^2;
        end
        varRL2 = sumSq/(ctrLengthSided2-2);
    elseif(ctrLengthSided2 > 0)
        varRL2 = lengthSided2(1,3);
    else
        varRL2 = 2000;
    end
    
    if(varRL1  < varRL2)
        lengthSided1Dominant = 1;
        lengthPoints = lengthSided1;
    else
        lengthPoints = lengthSided2;
    end
end
if(enablePlotRANSAC)
    scatter(lengthPoints(:,1), lengthPoints(:,2), 100, [1 .5 .5], 'o', 'Parent', axRAN, 'LineWidth', lw); hold on;
end
%% RANSAC to determine length edge
if(size(lengthPoints,1) < 5) % min 30 points needed for dependability
    success = 0;
    return;
end
% error bound
numIt = 5;
maxMargin = .5;
minMargin = .05;
stepMargin = (maxMargin-minMargin)/numIt;
% best estimation
maxNr = 30;
for i = 1 : numIt
   pBestLength = [0 0; 0 0];
   distMeanBest = 10^6;
   errorMargin = maxMargin - i*stepMargin;
   num = size(lengthPoints,1);
   if(num < maxNr)
       maxNr = num;
   end
   for j = 1 : maxNr
       % randomly choose 2 points
       index1 = round((size(lengthPoints,1)-1)*rand(1,1) + 1);
       index2 = round((size(lengthPoints,1)-1)*rand(1,1) + 1);
       while(index2 == index1)
           index2 = round((size(lengthPoints,1)-1)*rand(1,1) + 1);
       end
       % determine mean distance to this line
       % parameter form:
       x1 = lengthPoints(index1,1); x2 = lengthPoints(index2,1);
       y1 = lengthPoints(index1,2); y2 = lengthPoints(index2,2);
       gs = [x1;y1]; gr = [(x2-x1);(y2-y1)];
       gn = [gr(2,1); -gr(1,1)];
       distMean = 0;
       for k = 1 : size(lengthPoints,1)
           point = lengthPoints(k,1:2)';
           A = [gr(1,1), -gn(1,1); gr(2,1), -gn(2,1)];
           b = [point(1,1)-gs(1,1); point(2,1)-gs(2,1)];
           sol = A\b;
           s = gs + sol(1,1).*gr;
           distMean = distMean + (norm(s-point))^2;
       end
       distMean = distMean/size(lengthPoints,1)/2;
       if(distMean < distMeanBest)
          distMeanBest = distMean; 
          pBestLength = [gs, gr];
       end
   end
   % sort out
   index = true(1, size(lengthPoints,1));
   proceed = 0;
   gs = pBestLength(:,1); gr = pBestLength(:,2);
   gn = [gr(2,1); -gr(1,1)];
   for k = 1 : size(lengthPoints,1)
       point = lengthPoints(k,1:2)';
       A = [gr(1,1), -gn(1,1); gr(2,1), -gn(2,1)];
       b = [point(1,1)-gs(1,1);point(2,1)-gs(2,1)];
       sol = A\b;
       s = gs + sol(1,1).*gr;
       dist = norm(s-point);
       if(dist > errorMargin)
           index(k) = false;
       end
       if((dist > minMargin) && ~proceed)
           proceed = 1;
       end
   end
   if(~proceed)
       break;
   end
   outlier = lengthPoints(~index,:);
   lengthPoints = lengthPoints(index,:);
   % plot
   if(enablePlotRANSAC)
       scatter(outlier(:,1), outlier(:,2), 120, 'r', '.', 'Parent', axRAN); hold on; drawnow;
       scatter(lengthPoints(:,1), lengthPoints(:,2), 120, 'g', '.', 'Parent', axRAN); hold on; drawnow;
       p1 = pBestLength(:,1);
       p2 = pBestLength(:,1) + 3.* pBestLength(:,2); p3 = pBestLength(:,1) + -3.* pBestLength(:,2);
       xline = [p1(1,1); p2(1,1); p3(1,1)]; yline = [p1(2,1); p2(2,1); p3(2,1)];
       plot(xline,yline, 'Color', [.8 .8 .8], 'Parent', axRAN);
   end
end
if(enablePlotRANSAC || enablePlotNewton)
    p1 = pBestLength(:,1);
    p2 = pBestLength(:,1) + 3.* pBestLength(:,2); p3 = pBestLength(:,1) + -3.* pBestLength(:,2);
    xline = [p1(1,1); p2(1,1); p3(1,1)]; yline = [p1(2,1); p2(2,1); p3(2,1)];
    plot(xline,yline, 'Color', 'g', 'Parent', axRAN);
end
%% RANSAC to determine width edge
if(size(widthPoints,1) < 5) % min 30 points needed for dependability
    success = 0;
    return;
end
% error bound
numIt = 5;
maxMargin = .5;
minMargin = .05;
stepMargin = (maxMargin-minMargin)/numIt;
% best estimation
maxNr = 30;
for i = 1 : numIt
   pBestWidth = [0 0; 0 0];
   distMeanBest = 10^6;
   errorMargin = maxMargin - i*stepMargin;
   num = size(widthPoints,1);
   if(num < maxNr)
       maxNr = num;
   end
   for j = 1 : maxNr
       % randomly choose 2 points
       index1 = round((size(widthPoints,1)-1)*rand(1,1) + 1);
       index2 = round((size(widthPoints,1)-1)*rand(1,1) + 1);
       while(index2 == index1)
           index2 = round((size(widthPoints,1)-1)*rand(1,1) + 1);
       end
       % determine mean distance to this line
       % parameter form:
       x1 = widthPoints(index1,1);  y1 = widthPoints(index1,2);
       x2 = widthPoints(index2,1);  y2 = widthPoints(index2,2);
       gs = [x1;y1]; gr = [(x2-x1);(y2-y1)];
       gn = [gr(2,1); -gr(1,1)];
       distMean = 0;
       for k = 1 : size(widthPoints,1)
           point = widthPoints(k,1:2)';
           A = [gr(1,1), -gn(1,1); gr(2,1), -gn(2,1)];
           b = [point(1,1)-gs(1,1); point(2,1)-gs(2,1)];
           sol = A\b;
           s = gs + sol(1,1).*gr;
           distMean = distMean + norm(s-point);
       end
       distMean = distMean/size(widthPoints,1)/2;
       if(distMean < distMeanBest)
          distMeanBest = distMean; 
          pBestWidth = [gs, gr];
       end
   end
   % sort out
   % sort out
   index = true(1, size(widthPoints,1));
   proceed = 0;
   gs = pBestWidth(:,1); gr = pBestWidth(:,2);
   gn = [gr(2,1); -gr(1,1)];
   for k = 1 : size(widthPoints,1)
       point = widthPoints(k,1:2)';
       A = [gr(1,1), -gn(1,1); gr(2,1), -gn(2,1)];
       b = [point(1,1)-gs(1,1);point(2,1)-gs(2,1)];
       sol = A\b;
       s = gs + sol(1,1).*gr;
       dist = norm(s-point);
       if(dist > errorMargin)
           index(k) = false;
       end
       if((dist > minMargin) && ~proceed)
           proceed = 1;
       end
   end
   if(~proceed)
       break;
   end
   outlier = widthPoints(~index,:);
   widthPoints = widthPoints(index,:);
   % plot
   if(enablePlotRANSAC)
       scatter(outlier(:,1), outlier(:,2), 120, 'r', '.', 'Parent', axRAN); hold on; drawnow;
       scatter(widthPoints(:,1), widthPoints(:,2), 120, 'g', '.', 'Parent', axRAN); hold on; drawnow;
       p1 = pBestWidth(:,1);
       p2 = pBestWidth(:,1) + 3.* pBestWidth(:,2); p3 = pBestWidth(:,1) + -3.* pBestWidth(:,2);
       xline = [p1(1,1); p2(1,1); p3(1,1)]; yline = [p1(2,1); p2(2,1); p3(2,1)];
       plot(xline,yline, 'Color', [.8 .8 .8], 'Parent', axRAN);
   end
end
if(enablePlotRANSAC || enablePlotNewton)
    p1 = pBestWidth(:,1);
    p2 = pBestWidth(:,1) + 3.* pBestWidth(:,2); p3 = pBestWidth(:,1) + -3.* pBestWidth(:,2);
    xline = [p1(1,1); p2(1,1); p3(1,1)]; yline = [p1(2,1); p2(2,1); p3(2,1)];
    plot(xline,yline, 'Color', 'g', 'Parent', axRAN);
end

%% determine destinations
l = movableData.m_BBLength; w = movableData.m_BBWidth;
% sA is intersection of length and width edge
grLength = pBestLength(:,2); gsLength = pBestLength(:,1);
grWidth = pBestWidth(:,2);   gsWidth = pBestWidth(:,1);

A = [grLength(1,1), -grWidth(1,1); grLength(2,1), -grWidth(2,1)];
b = [gsWidth(1,1)-gsLength(1,1); gsWidth(2,1)-gsLength(2,1)];

try
    sol = A\b;
catch
    % no solution, stop
    success = 0;
    return;
end
sA = gsLength + sol(1,1).*grLength;
sAx = sA(1,1); sAy = sA(2,1);

% calculate angle of intersection between both lines to estimate consistency
angleOfIntersection = acos(abs(dot(grLength,grWidth))/(norm(grLength) * norm(grWidth))) * 180/pi(); % sharp angle

% sB lies on length edge, in distance of length mean
meanLength = mean(lengthPoints);
cen2Mean = meanLength(1,1:2) - sA';
scal = dot(cen2Mean, grLength);
sign = scal/abs(scal);
sB = sA + sign*grLength./norm(grLength) .* l;
sBx = sB(1,1); sBy = sB(2,1);

% sC lies on width edge, in distance of widh mean
meanWidth = mean(widthPoints);
cen2Mean = meanWidth(1,1:2) - sA';
scal = dot(cen2Mean, grWidth);
sign = scal/abs(scal);
sC = sA + sign*grWidth./norm(grWidth) .* w;
sCx = sC(1,1); sCy = sC(2,1);
if(lengthSided1Dominant)    % "left"
   if(widthSided1Dominant)  % "top"
       sigPAl =  1; sigPAw =  1;
       sigPBl = -1; sigPBw =  1;
       sigPCl =  1; sigPCw = -1;
   else                     % "bottom"
       sigPAl = -1; sigPAw =  1;
       sigPBl =  1; sigPBw =  1;
       sigPCl = -1; sigPCw = -1;
   end
else
    if(widthSided1Dominant)  % "top"
       sigPAl =  1; sigPAw = -1;
       sigPBl = -1; sigPBw = -1;
       sigPCl =  1; sigPCw =  1;
   else                     % "bottom"
       sigPAl = -1; sigPAw = -1;
       sigPBl =  1; sigPBw = -1;
       sigPCl = -1; sigPCw =  1;
    end
end

if(enablePlotNewton)
    lw = 3;
    scatter(sAx, sAy, 200, [0 1 1], 'o', 'Parent', axRAN, 'LineWidth', lw); hold on;
    scatter(sBx, sBy, 200, [0 1 1], 'o', 'Parent', axRAN, 'LineWidth', lw); hold on;
    scatter(sCx, sCy, 200, [0 1 1], 'o', 'Parent', axRAN, 'LineWidth', lw); hold on;
%     drawRectangle(axRAN, [1 0 0], movableData, 0); hold on;
end

% movableData.m_BBMiddle_x = movableData.m_BBMiddle_x + 2;
% movableData.m_BBMiddle_y = movableData.m_BBMiddle_y + 3;
% movableData.m_BBYaw = movableData.m_BBYaw - 45;
%% apply newtons algorithm for optimization
n = 25; % for drawing
fi = 500;

g = movableData.m_BBYaw * pi()/180;
x = movableData.m_BBMiddle_x;
y = movableData.m_BBMiddle_y;

if(enablePlotNewton)
   movableData_draw = copy(movableData); 
end

for i = 1 : fi
    if(enablePlotNewton && (i < 150) && ((mod(i,n)==0) || (i<3) || (i==fi)))
        color = [rand(1,1) rand(1,1) rand(1,1)];
        drawRectangle(axRAN, color, movableData_draw, 0); hold on; drawnow;
        pA(1,1) = (x + sigPAl*l/2*cos(g) + sigPAw*w/2*cos(g+pi()/2));
        pA(2,1) = (y + sigPAl*l/2*sin(g) + sigPAw*w/2*sin(g+pi()/2));
        pB(1,1) = (x + sigPBl*l/2*cos(g) + sigPBw*w/2*cos(g+pi()/2));
        pB(2,1) = (y + sigPBl*l/2*sin(g) + sigPBw*w/2*sin(g+pi()/2));
        pC(1,1) = (x + sigPCl*l/2*cos(g) + sigPCw*w/2*cos(g+pi()/2));
        pC(2,1) = (y + sigPCl*l/2*sin(g) + sigPCw*w/2*sin(g+pi()/2));
        scatter(pA(1,1), pA(2,1), 100, color, 'x', 'Parent', axRAN); hold on;
        scatter(pB(1,1), pB(2,1), 100, color, 'x', 'Parent', axRAN); hold on;
        scatter(pC(1,1), pC(2,1), 100, color, 'x', 'Parent', axRAN); hold on;
    end
    
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
    
    g11 = 6*x - 2*sBx - 2*sCx - 2*sAx + l*sigPAl*cos(g) + l*sigPBl*cos(g) + l*sigPCl*cos(g) + sigPAw*w*cos(g + pi/2) + sigPBw*w*cos(g + pi/2) + sigPCw*w*cos(g + pi/2);
    g12 = 6*y - 2*sBy - 2*sCy - 2*sAy + l*sigPAl*sin(g) + l*sigPBl*sin(g) + l*sigPCl*sin(g) + sigPAw*w*sin(g + pi/2) + sigPBw*w*sin(g + pi/2) + sigPCw*w*sin(g + pi/2);
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
    
    if(enablePlotNewton)
        movableData_draw.m_BBMiddle_x   = x;
        movableData_draw.m_BBMiddle_y   = y;
        movableData_draw.m_BBYaw = g*180/pi();
    end
end

if(enablePlotNewton)
    drawRectangle(axRAN, [0 1 0], movableData_draw, 0); hold on;
    axis equal
end

middleEstimated(1,1) = x;
middleEstimated(2,1) = y;
yawEstimated         = g * 180/pi();

end

