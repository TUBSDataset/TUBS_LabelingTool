function [ middleXY, yaw, success ] = estimateBoxPCEdgePoints( movableData, assignedPoints, axEdges)

enablePlot = 0;
if(~isempty(axEdges))
    enablePlot = 1;
end

middleXY = [];
yaw     = [];
success = 1;

%% build sub rectangles
lvec(1,1) = movableData.m_BBLength/2*cos(movableData.m_BBYaw*pi()/180);
lvec(2,1) = movableData.m_BBLength/2*sin(movableData.m_BBYaw*pi()/180);
wvec(1,1) = movableData.m_BBWidth/2*cos(movableData.m_BBYaw*pi()/180 + pi()/2);
wvec(2,1) = movableData.m_BBWidth/2*sin(movableData.m_BBYaw*pi()/180 + pi()/2);

subRect1 = cMovableData();
subRect1.m_BBMiddle_x = movableData.m_BBMiddle_x + lvec(1,1)/2 + wvec(1,1)/2;
subRect1.m_BBMiddle_y = movableData.m_BBMiddle_y + lvec(2,1)/2 + wvec(2,1)/2;
subRect1.m_BBYaw = movableData.m_BBYaw;
subRect1.m_BBLength = movableData.m_BBLength/2;
subRect1.m_BBWidth = movableData.m_BBWidth/2;

subRect2 = cMovableData();
subRect2.m_BBMiddle_x = movableData.m_BBMiddle_x - lvec(1,1)/2 + wvec(1,1)/2;
subRect2.m_BBMiddle_y = movableData.m_BBMiddle_y - lvec(2,1)/2 + wvec(2,1)/2;
subRect2.m_BBYaw = movableData.m_BBYaw;
subRect2.m_BBLength = movableData.m_BBLength/2;
subRect2.m_BBWidth = movableData.m_BBWidth/2;

subRect3 = cMovableData();
subRect3.m_BBMiddle_x = movableData.m_BBMiddle_x - lvec(1,1)/2 - wvec(1,1)/2;
subRect3.m_BBMiddle_y = movableData.m_BBMiddle_y - lvec(2,1)/2 - wvec(2,1)/2;
subRect3.m_BBYaw = movableData.m_BBYaw;
subRect3.m_BBLength = movableData.m_BBLength/2;
subRect3.m_BBWidth = movableData.m_BBWidth/2;

subRect4 = cMovableData();
subRect4.m_BBMiddle_x = movableData.m_BBMiddle_x + lvec(1,1)/2 - wvec(1,1)/2;
subRect4.m_BBMiddle_y = movableData.m_BBMiddle_y + lvec(2,1)/2 - wvec(2,1)/2;
subRect4.m_BBYaw = movableData.m_BBYaw;
subRect4.m_BBLength = movableData.m_BBLength/2;
subRect4.m_BBWidth = movableData.m_BBWidth/2;

%% get points in rectangles
BBTestMatrix1 = determineBBTestMatrix(subRect1,.5,0,.5,0);
BBTestMatrix2 = determineBBTestMatrix(subRect2,0,.5,.5,0);
BBTestMatrix3 = determineBBTestMatrix(subRect3,0,.5,0,.5);
BBTestMatrix4 = determineBBTestMatrix(subRect4,.5,0,0,.5);
PsRect1 = zeros(size(assignedPoints,1),2);
PsRect2 = zeros(size(assignedPoints,1),2);
PsRect3 = zeros(size(assignedPoints,1),2);
PsRect4 = zeros(size(assignedPoints,1),2);
ctr1 = 1; ctr2 = 1; ctr3 = 1; ctr4 = 1;
for i = 1 : size(assignedPoints,1)
    p = assignedPoints(i,1:2);
    % test 1
    D12 = BBTestMatrix1(1,1)*p(1,1)+BBTestMatrix1(1,2)*p(1,2)+BBTestMatrix1(1,3);
    D23 = BBTestMatrix1(1,4)*p(1,1)+BBTestMatrix1(1,5)*p(1,2)+BBTestMatrix1(1,6);
    D34 = BBTestMatrix1(1,7)*p(1,1)+BBTestMatrix1(1,8)*p(1,2)+BBTestMatrix1(1,9);
    D41 = BBTestMatrix1(1,10)*p(1,1)+BBTestMatrix1(1,11)*p(1,2)+BBTestMatrix1(1,12);
    if((D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0))
        PsRect1(ctr1,:) = p;
        ctr1 = ctr1 + 1;
        continue;
    end
    % test 2
    D12 = BBTestMatrix2(1,1)*p(1,1)+BBTestMatrix2(1,2)*p(1,2)+BBTestMatrix2(1,3);
    D23 = BBTestMatrix2(1,4)*p(1,1)+BBTestMatrix2(1,5)*p(1,2)+BBTestMatrix2(1,6);
    D34 = BBTestMatrix2(1,7)*p(1,1)+BBTestMatrix2(1,8)*p(1,2)+BBTestMatrix2(1,9);
    D41 = BBTestMatrix2(1,10)*p(1,1)+BBTestMatrix2(1,11)*p(1,2)+BBTestMatrix2(1,12);
    if((D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0))
        PsRect2(ctr2,:) = p;
        ctr2 = ctr2 + 1;
        continue;
    end
    % test 3
    D12 = BBTestMatrix3(1,1)*p(1,1)+BBTestMatrix3(1,2)*p(1,2)+BBTestMatrix3(1,3);
    D23 = BBTestMatrix3(1,4)*p(1,1)+BBTestMatrix3(1,5)*p(1,2)+BBTestMatrix3(1,6);
    D34 = BBTestMatrix3(1,7)*p(1,1)+BBTestMatrix3(1,8)*p(1,2)+BBTestMatrix3(1,9);
    D41 = BBTestMatrix3(1,10)*p(1,1)+BBTestMatrix3(1,11)*p(1,2)+BBTestMatrix3(1,12);
    if((D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0))
        PsRect3(ctr3,:) = p;
        ctr3 = ctr3 + 1;
        continue;
    end
    % test 4
    D12 = BBTestMatrix4(1,1)*p(1,1)+BBTestMatrix4(1,2)*p(1,2)+BBTestMatrix4(1,3);
    D23 = BBTestMatrix4(1,4)*p(1,1)+BBTestMatrix4(1,5)*p(1,2)+BBTestMatrix4(1,6);
    D34 = BBTestMatrix4(1,7)*p(1,1)+BBTestMatrix4(1,8)*p(1,2)+BBTestMatrix4(1,9);
    D41 = BBTestMatrix4(1,10)*p(1,1)+BBTestMatrix4(1,11)*p(1,2)+BBTestMatrix4(1,12);
    if((D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0))
        PsRect4(ctr4,:) = p;
        ctr4 = ctr4 + 1;
        continue;
    end
end
PsRect1 = PsRect1(1:(ctr1-1),:);
PsRect2 = PsRect2(1:(ctr2-1),:);
PsRect3 = PsRect3(1:(ctr3-1),:);
PsRect4 = PsRect4(1:(ctr4-1),:);
%% extrema
p1 = []; p2 = []; p3 = []; p4 = []; empty_ctr = 0;
if(~isempty(PsRect1))
    % outer front left corner, criteria for edge point: minimal distance to corner
    point(1,1) = movableData.m_BBMiddle_x + lvec(1,1) + wvec(1,1);
    point(2,1) = movableData.m_BBMiddle_y + lvec(2,1) + wvec(2,1);
    distFnc = @(v) sqrt((point(1,1)-v(1,1))^2 + (point(2,1)-v(1,2))^2);
    
    PsRect1 = PsRect1(1:ctr1-1,:);
    cellPsRect1 = num2cell(PsRect1, 2);
    distance = cellfun(distFnc, cellPsRect1);
    PsRect1(:,3) = distance;
    PsRect1 = sortrows(PsRect1,3);
    if(size(PsRect1,1) < 10)
        top = PsRect1;
    else
        top = PsRect1(1:10,:);
    end
    if(size(top,1) == 1)
        med = top;
    else
        med = median(top);
    end
    p1(1,1) = med(1,1); p1(2,1) = med(1,2);
else
   empty_ctr = empty_ctr + 1; 
end

if(~isempty(PsRect2))
    point(1,1) = movableData.m_BBMiddle_x - lvec(1,1) + wvec(1,1);
    point(2,1) = movableData.m_BBMiddle_y - lvec(2,1) + wvec(2,1);
    distFnc = @(v) sqrt((point(1,1)-v(1,1))^2 + (point(2,1)-v(1,2))^2);
    
    cellPsRect2 = num2cell(PsRect2, 2);
    distance = cellfun(distFnc, cellPsRect2);
    PsRect2(:,3) = distance;
    PsRect2 = sortrows(PsRect2,3);
    
    if(size(PsRect2,1) < 10)
        top = PsRect2;
    else
        top = PsRect2(1:10,:);
    end
    if(size(top,1) == 1)
        med = top;
    else
        med = median(top);
    end;
    p2(1,1) = med(1,1); p2(2,1) = med(1,2);
else
   empty_ctr = empty_ctr + 1; 
end

if(~isempty(PsRect3))
    point(1,1) = movableData.m_BBMiddle_x - lvec(1,1) - wvec(1,1);
    point(2,1) = movableData.m_BBMiddle_y - lvec(2,1) - wvec(2,1);
    distFnc = @(v) sqrt((point(1,1)-v(1,1))^2 + (point(2,1)-v(1,2))^2);
    
    cellPsRect3 = num2cell(PsRect3, 2);
    distance = cellfun(distFnc, cellPsRect3);
    PsRect3(:,3) = distance;
    PsRect3 = sortrows(PsRect3,3);
    
    if(size(PsRect3,1) < 10)
        top = PsRect3;
    else
        top = PsRect3(1:10,:);
    end
    if(size(top,1) == 1)
        med = top;
    else
        med = median(top);
    end
    p3(1,1) = med(1,1); p3(2,1) = med(1,2);
else
   empty_ctr = empty_ctr + 1; 
end

if(~isempty(PsRect4))
    point(1,1) = movableData.m_BBMiddle_x + lvec(1,1) - wvec(1,1);
    point(2,1) = movableData.m_BBMiddle_y + lvec(2,1) - wvec(2,1);
    distFnc = @(v) sqrt((point(1,1)-v(1,1))^2 + (point(2,1)-v(1,2))^2);
    
    cellPsRect4 = num2cell(PsRect4, 2);
    distance = cellfun(distFnc, cellPsRect4);
    PsRect4(:,3) = distance;
    PsRect4 = sortrows(PsRect4,3);
    
    if(size(PsRect4,1) < 10)
        top = PsRect4;
    else
        top = PsRect4(1:10,:);
    end
    if(size(top,1) == 1)
        med = top;
    else
        med = median(top);
    end
    p4(1,1) = med(1,1); p4(2,1) = med(1,2);
else
   empty_ctr = empty_ctr + 1; 
end

%% estimate middle
% min 1 diagonal needed
if((isempty(p1) || isempty(p3)) && ((isempty(p4) || isempty(p2))))
   success = 0; 
   return;
end

if(~isempty(p3) && ~isempty(p1))
    p13 = p3 - p1;
else
    p13 = [0;0];
end

if(~isempty(p2) && ~isempty(p4))
    p42 = p2 - p4;
else
    p42 = [0;0];
end
length13 = norm(p13);
length42 = norm(p42);

lenList = cell(2,3);
% 2. column is source, 3. column is destination
lenList(1,1) = {length13}; lenList(1,2) = {p1}; lenList(1,3) = {p3};
lenList(2,1) = {length42}; lenList(2,2) = {p4}; lenList(2,3) = {p2};
lenList = sortrows(lenList,-1);

% check if lines are amost equal -> movableData with 4 edge points available,
% intersect those lines
longer  = lenList{1,1};
shorter = lenList{2,1};
if((longer - shorter) < .05*longer) 
    % all 4 points available
    A = [p13(1,1) - p42(1,1);
         p13(2,1) - p42(2,1)];
    b = [p4(1,1) - p1(1,1); 
         p4(2,1) - p1(2,1)];
    X = A\b;
    middleXY = p1 + X(1,1) * p13;
else
    src = lenList{1,2}; dest = lenList{1,3};
    middleXY = src + (dest - src)/2;
end

%% estimate yaw
if(~isempty(p2) && ~isempty(p1))
    p12 = p2 - p1;
else
    p12 = [0;0];
end
if(~isempty(p3) && ~isempty(p4))
    p43 = p3 - p4;
else
    p43 = [0;0];
end

length12 = norm(p12);
length43 = norm(p43);

longer  = max([length12, length43]);
shorter = min([length12, length43]);

if((longer - shorter) < .05*longer) 
    % complete box with 4 edge points
    edgePoints(1,:) = p1';
    edgePoints(2,:) = p2';
    edgePoints(3,:) = p3';
    edgePoints(4,:) = p4';
else
    % more than 1 is missing (min 3 points needed)
    if(empty_ctr > 1)
       success = 0;
       return;
    end
    % 4th edge is occluded
    % estimate the 4th edge point 
    lenList = cell(2,3);
    % 2. column is source, 3. column ist destination
    lenList(1,1) = {length12}; lenList(1,2) = {p1}; lenList(1,3) = {p2};
    lenList(2,1) = {length43}; lenList(2,2) = {p4}; lenList(2,3) = {p3};
    lenList = sortrows(lenList,-1);
    edgePoints(1,:) = lenList{1,2}; % src
    edgePoints(2,:) = lenList{1,3}; % dest
    
    % check which one is to delete
    src = lenList{2,2};
    dest = lenList{2,3};
    
    if(~isempty(src))
        src2middle = norm(middleXY - src);
    else
        src2middle = 0;
    end
    
    if(~isempty(dest))
        dest2middle = norm(middleXY - dest);
    else
        dest2middle = 0;
    end
    
    if(src2middle < dest2middle)
        % src is outlier
        edgePoints(3,:) = dest';
        edgePoints(4,:) = (lenList{1,2} + (dest - lenList{1,3}))';
    else
        % dest ist outlier
        edgePoints(3,:) = (lenList{1,3} + (src - lenList{1,2}))';
        edgePoints(4,:) = src';
    end
end
% indicator
edgePoints(5,:) = middleXY + lvec*10;

[ estimatedMovableData, success2 ] = calculateBoxParameters( edgePoints );

if(success2 ~= 1)
   success = 0; % something went wrong
   return; 
end

middleXY(1,1) = estimatedMovableData.m_BBMiddle_x;
middleXY(2,1) = estimatedMovableData.m_BBMiddle_y;
yaw = estimatedMovableData.m_BBYaw;

%% test plot
if(enablePlot)
    scatter(assignedPoints(:,1), assignedPoints(:,2), 30, [0 0 0], '.', 'Parent', axEdges); hold on;
    drawRectangle(axEdges, [1 0 0], movableData, 0); hold on;
    lw = 3;
    drawRectangle(axEdges, [0 1 0], estimatedMovableData, 0, lw); hold on;
    sz = 200; lw = 3;
    if(~isempty(p1))
        scatter(p1(1,1), p1(2,1), sz, 'r', 'o', 'Parent', axEdges, 'LineWidth', lw); hold on;
        scatter(p1(1,1), p1(2,1), sz, 'r', 'x', 'Parent', axEdges, 'LineWidth', 1); hold on;
    end
    if(~isempty(p2))
        scatter(p2(1,1), p2(2,1), sz, 'r', 'o', 'Parent', axEdges, 'LineWidth', lw); hold on;
        scatter(p2(1,1), p2(2,1), sz, 'r', 'x', 'Parent', axEdges, 'LineWidth', 1); hold on;
    end
    if(~isempty(p3))
        scatter(p3(1,1), p3(2,1), sz, 'r', 'o', 'Parent', axEdges, 'LineWidth', lw); hold on;
        scatter(p3(1,1), p3(2,1), sz, 'r', 'x', 'Parent', axEdges, 'LineWidth', 1); hold on;
    end
    if(~isempty(p4))
        scatter(p4(1,1), p4(2,1), sz, 'r', 'o', 'Parent', axEdges, 'LineWidth', lw); hold on;
        scatter(p4(1,1), p4(2,1), sz, 'r', 'x', 'Parent', axEdges, 'LineWidth', 1); hold on;
    end
    bright = .9; dark = .6; sz = 20;
    color1 = [bright dark dark];
    lw = 3;
    drawRectangle(axEdges, color1, subRect1, 0); hold on;
    if(~isempty(PsRect1))
        scatter(PsRect1(:,1), PsRect1(:,2), sz, color1, 'o', 'Parent', axEdges);
    end
    color2 = [dark dark bright];
    drawRectangle(axEdges, color2, subRect2, 0); hold on;
    if(~isempty(PsRect2))
        scatter(PsRect2(:,1), PsRect2(:,2), sz, color2, 'o', 'Parent', axEdges);
    end
    color3 = [bright dark bright];
    drawRectangle(axEdges, [.5 .5 .7], subRect3, 0); hold on;
    if(~isempty(PsRect3))
        scatter(PsRect3(:,1), PsRect3(:,2), sz, color3, 'o', 'Parent', axEdges);
    end
    color4 = [dark bright bright];
    drawRectangle(axEdges, color4, subRect4, 0); hold on;
    if(~isempty(PsRect4))
        scatter(PsRect4(:,1), PsRect4(:,2), sz, color4, 'o', 'Parent', axEdges);
    end
    axis square;
end

end