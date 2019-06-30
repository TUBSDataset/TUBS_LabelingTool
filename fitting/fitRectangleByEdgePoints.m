function [vfMiddleXY, fYaw, bSuccess] = fitRectangleByEdgePoints(oPCMovableLabel, mfAssignedPoints, oAxEdges_h)
% ---------------------------------------------------------------------------------------------
% Function fitRectangleByEdgePoints fits a rectangle by estimating the edge points of a given point cloud.
% The edge points are then rectified using normal shooting as implemented in estimateRectangle(...). A rough
% estimation of the object's position is required (as done by the prediciton step)
%
% INPUT:
%   oPCMovableLabel:     Object of class cPCMovableLabel containing a rough yaw and middle estimation
%   mfAssignedPoints:    Points within the object's box 
%   oAxEdges_h           Plotting axes

% OUTPUT:
%   vfMiddleXY:          Estimated middle
%   fYaw:                Estimated yaw
%   bSuccess:            True if algorithm succeeded
% --------------------------------------------------------------------------------------------- 

bEnablePlot = 0;
if (~isempty(oAxEdges_h))
    bEnablePlot = 1;
end

vfMiddleXY = [];
fYaw     = [];
bSuccess = 1;

%% Build sub rectangles
lvec(1,1) = oPCMovableLabel.m_fBBLength/2*cos(oPCMovableLabel.m_fBBYaw*pi()/180);
lvec(2,1) = oPCMovableLabel.m_fBBLength/2*sin(oPCMovableLabel.m_fBBYaw*pi()/180);
wvec(1,1) = oPCMovableLabel.m_fBBWidth/2*cos(oPCMovableLabel.m_fBBYaw*pi()/180 + pi()/2);
wvec(2,1) = oPCMovableLabel.m_fBBWidth/2*sin(oPCMovableLabel.m_fBBYaw*pi()/180 + pi()/2);

subRect1 = cPCMovableLabel();
subRect1.m_fBBMiddle_x = oPCMovableLabel.m_fBBMiddle_x + lvec(1,1)/2 + wvec(1,1)/2;
subRect1.m_fBBMiddle_y = oPCMovableLabel.m_fBBMiddle_y + lvec(2,1)/2 + wvec(2,1)/2;
subRect1.m_fBBYaw = oPCMovableLabel.m_fBBYaw;
subRect1.m_fBBLength = oPCMovableLabel.m_fBBLength/2;
subRect1.m_fBBWidth = oPCMovableLabel.m_fBBWidth/2;

subRect2 = cPCMovableLabel();
subRect2.m_fBBMiddle_x = oPCMovableLabel.m_fBBMiddle_x - lvec(1,1)/2 + wvec(1,1)/2;
subRect2.m_fBBMiddle_y = oPCMovableLabel.m_fBBMiddle_y - lvec(2,1)/2 + wvec(2,1)/2;
subRect2.m_fBBYaw = oPCMovableLabel.m_fBBYaw;
subRect2.m_fBBLength = oPCMovableLabel.m_fBBLength/2;
subRect2.m_fBBWidth = oPCMovableLabel.m_fBBWidth/2;

subRect3 = cPCMovableLabel();
subRect3.m_fBBMiddle_x = oPCMovableLabel.m_fBBMiddle_x - lvec(1,1)/2 - wvec(1,1)/2;
subRect3.m_fBBMiddle_y = oPCMovableLabel.m_fBBMiddle_y - lvec(2,1)/2 - wvec(2,1)/2;
subRect3.m_fBBYaw = oPCMovableLabel.m_fBBYaw;
subRect3.m_fBBLength = oPCMovableLabel.m_fBBLength/2;
subRect3.m_fBBWidth = oPCMovableLabel.m_fBBWidth/2;

subRect4 = cPCMovableLabel();
subRect4.m_fBBMiddle_x = oPCMovableLabel.m_fBBMiddle_x + lvec(1,1)/2 - wvec(1,1)/2;
subRect4.m_fBBMiddle_y = oPCMovableLabel.m_fBBMiddle_y + lvec(2,1)/2 - wvec(2,1)/2;
subRect4.m_fBBYaw = oPCMovableLabel.m_fBBYaw;
subRect4.m_fBBLength = oPCMovableLabel.m_fBBLength/2;
subRect4.m_fBBWidth = oPCMovableLabel.m_fBBWidth/2;

%% Get points in rectangles
BBTestMatrix1 = determineBBTestMatrix(subRect1,.5,0,.5,0);
BBTestMatrix2 = determineBBTestMatrix(subRect2,0,.5,.5,0);
BBTestMatrix3 = determineBBTestMatrix(subRect3,0,.5,0,.5);
BBTestMatrix4 = determineBBTestMatrix(subRect4,.5,0,0,.5);

PsRect1 = zeros(size(mfAssignedPoints,1),2);
PsRect2 = zeros(size(mfAssignedPoints,1),2);
PsRect3 = zeros(size(mfAssignedPoints,1),2);
PsRect4 = zeros(size(mfAssignedPoints,1),2);

ctr1 = 1; ctr2 = 1; ctr3 = 1; ctr4 = 1;

% Testing done in loop without further function calls for runtime reasons
for i = 1 : size(mfAssignedPoints,1)
    p = mfAssignedPoints(i,1:2);
    
    % Test 1
    D12 = BBTestMatrix1(1,1)*p(1,1)+BBTestMatrix1(1,2)*p(1,2)+BBTestMatrix1(1,3);
    D23 = BBTestMatrix1(1,4)*p(1,1)+BBTestMatrix1(1,5)*p(1,2)+BBTestMatrix1(1,6);
    D34 = BBTestMatrix1(1,7)*p(1,1)+BBTestMatrix1(1,8)*p(1,2)+BBTestMatrix1(1,9);
    D41 = BBTestMatrix1(1,10)*p(1,1)+BBTestMatrix1(1,11)*p(1,2)+BBTestMatrix1(1,12);
    
    if((D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0))
        PsRect1(ctr1,:) = p;
        ctr1 = ctr1 + 1;
        continue;
    end
    
    % Test 2
    D12 = BBTestMatrix2(1,1)*p(1,1)+BBTestMatrix2(1,2)*p(1,2)+BBTestMatrix2(1,3);
    D23 = BBTestMatrix2(1,4)*p(1,1)+BBTestMatrix2(1,5)*p(1,2)+BBTestMatrix2(1,6);
    D34 = BBTestMatrix2(1,7)*p(1,1)+BBTestMatrix2(1,8)*p(1,2)+BBTestMatrix2(1,9);
    D41 = BBTestMatrix2(1,10)*p(1,1)+BBTestMatrix2(1,11)*p(1,2)+BBTestMatrix2(1,12);
    
    if((D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0))
        PsRect2(ctr2,:) = p;
        ctr2 = ctr2 + 1;
        continue;
    end
    
    % Test 3
    D12 = BBTestMatrix3(1,1)*p(1,1)+BBTestMatrix3(1,2)*p(1,2)+BBTestMatrix3(1,3);
    D23 = BBTestMatrix3(1,4)*p(1,1)+BBTestMatrix3(1,5)*p(1,2)+BBTestMatrix3(1,6);
    D34 = BBTestMatrix3(1,7)*p(1,1)+BBTestMatrix3(1,8)*p(1,2)+BBTestMatrix3(1,9);
    D41 = BBTestMatrix3(1,10)*p(1,1)+BBTestMatrix3(1,11)*p(1,2)+BBTestMatrix3(1,12);
    
    if((D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0))
        PsRect3(ctr3,:) = p;
        ctr3 = ctr3 + 1;
        continue;
    end
    
    % Test 4
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

%% Find extrema 
p1 = []; p2 = []; p3 = []; p4 = []; empty_ctr = 0;
if (~isempty(PsRect1))
    % Outer front left corner
    point(1,1) = oPCMovableLabel.m_fBBMiddle_x + lvec(1,1) + wvec(1,1);
    point(2,1) = oPCMovableLabel.m_fBBMiddle_y + lvec(2,1) + wvec(2,1);
    
    p1 = getEdgePoint(PsRect1, ctr1, point);
else
   empty_ctr = empty_ctr + 1; 
end

if (~isempty(PsRect2))
    point(1,1) = oPCMovableLabel.m_fBBMiddle_x - lvec(1,1) + wvec(1,1);
    point(2,1) = oPCMovableLabel.m_fBBMiddle_y - lvec(2,1) + wvec(2,1);
    p2 = getEdgePoint(PsRect2, ctr2, point);
else
   empty_ctr = empty_ctr + 1; 
end

if (~isempty(PsRect3))
    point(1,1) = oPCMovableLabel.m_fBBMiddle_x - lvec(1,1) - wvec(1,1);
    point(2,1) = oPCMovableLabel.m_fBBMiddle_y - lvec(2,1) - wvec(2,1);
    p3 = getEdgePoint(PsRect3, ctr3, point);
else
   empty_ctr = empty_ctr + 1; 
end

if (~isempty(PsRect4))
    point(1,1) = oPCMovableLabel.m_fBBMiddle_x + lvec(1,1) - wvec(1,1);
    point(2,1) = oPCMovableLabel.m_fBBMiddle_y + lvec(2,1) - wvec(2,1);
    p4 = getEdgePoint(PsRect4, ctr4, point);
else
   empty_ctr = empty_ctr + 1; 
end

%% Estimate middle
%  At least one diagonal is needed
if (isempty(p1) || isempty(p3)) && ((isempty(p4) || isempty(p2)))
   bSuccess = 0; 
   return;
end

if (~isempty(p3) && ~isempty(p1))
    p13 = p3 - p1;
else
    p13 = [0;0];
end

if (~isempty(p2) && ~isempty(p4))
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

% Check if lines are amost equal -> Object with 4 edge points available
% Intersect those lines
longer  = lenList{1,1};
shorter = lenList{2,1};
if (longer - shorter) < .05*longer
    % All 4 points available
    A = [p13(1,1) - p42(1,1);
         p13(2,1) - p42(2,1)];
    b = [p4(1,1) - p1(1,1); 
         p4(2,1) - p1(2,1)];
    X = A\b;
    vfMiddleXY = p1 + X(1,1) * p13;
else
    src = lenList{1,2}; dest = lenList{1,3};
    vfMiddleXY = src + (dest - src)/2;
end

%% Estimate yaw
if (~isempty(p2) && ~isempty(p1))
    p12 = p2 - p1;
else
    p12 = [0;0];
end
if (~isempty(p3) && ~isempty(p4))
    p43 = p3 - p4;
else
    p43 = [0;0];
end

length12 = norm(p12);
length43 = norm(p43);

longer  = max([length12, length43]);
shorter = min([length12, length43]);

if (longer - shorter) < .05*longer
    % Complete box with 4 edge points
    edgePoints(1,:) = p1';
    edgePoints(2,:) = p2';
    edgePoints(3,:) = p3';
    edgePoints(4,:) = p4';
else
    % More than 1 is missing (min 3 points needed)
    if (empty_ctr > 1)
       bSuccess = 0;
       return;
    end
    % 4th edge is occluded
    % Estimate the 4th edge point 
    lenList = cell(2,3);
    % 2. column is source, 3. column is destination
    lenList(1,1) = {length12}; lenList(1,2) = {p1}; lenList(1,3) = {p2};
    lenList(2,1) = {length43}; lenList(2,2) = {p4}; lenList(2,3) = {p3};
    lenList = sortrows(lenList,-1);
    edgePoints(1,:) = lenList{1,2}; % src
    edgePoints(2,:) = lenList{1,3}; % dest
    
    % Check which one is to delete
    src = lenList{2,2};
    dest = lenList{2,3};
    
    if (~isempty(src))
        src2middle = norm(vfMiddleXY - src);
    else
        src2middle = 0;
    end
    
    if (~isempty(dest))
        dest2middle = norm(vfMiddleXY - dest);
    else
        dest2middle = 0;
    end
    
    if (src2middle < dest2middle)
        % src is outlier
        edgePoints(3,:) = dest';
        edgePoints(4,:) = (lenList{1,2} + (dest - lenList{1,3}))';
    else
        % dest ist outlier
        edgePoints(3,:) = (lenList{1,3} + (src - lenList{1,2}))';
        edgePoints(4,:) = src';
    end
end

% Indicator point
edgePoints(5,:) = vfMiddleXY + lvec*10;

[oMovableLabel_estm, success2] = calculateBoxParameters(edgePoints);

if(success2 ~= 1)
   bSuccess = 0; % something went wrong
   return; 
end

vfMiddleXY(1,1) = oMovableLabel_estm.m_fBBMiddle_x;
vfMiddleXY(2,1) = oMovableLabel_estm.m_fBBMiddle_y;
fYaw = oMovableLabel_estm.m_fBBYaw;

%% Debug plot
if (bEnablePlot)
    scatter(mfAssignedPoints(:,1), mfAssignedPoints(:,2), 30, [0 0 0], '.', 'Parent', oAxEdges_h); hold on;
    drawRectangle(oAxEdges_h, [1 0 0], oPCMovableLabel, 0); hold on;
    lw = 3;
    drawRectangle(oAxEdges_h, [0 1 0], oMovableLabel_estm, 0, lw); hold on;
    sz = 200; lw = 3;
    if (~isempty(p1))
        scatter(p1(1,1), p1(2,1), sz, 'r', 'o', 'Parent', oAxEdges_h, 'LineWidth', lw); hold on;
        scatter(p1(1,1), p1(2,1), sz, 'r', 'x', 'Parent', oAxEdges_h, 'LineWidth', 1); hold on;
    end
    if (~isempty(p2))
        scatter(p2(1,1), p2(2,1), sz, 'r', 'o', 'Parent', oAxEdges_h, 'LineWidth', lw); hold on;
        scatter(p2(1,1), p2(2,1), sz, 'r', 'x', 'Parent', oAxEdges_h, 'LineWidth', 1); hold on;
    end
    if (~isempty(p3))
        scatter(p3(1,1), p3(2,1), sz, 'r', 'o', 'Parent', oAxEdges_h, 'LineWidth', lw); hold on;
        scatter(p3(1,1), p3(2,1), sz, 'r', 'x', 'Parent', oAxEdges_h, 'LineWidth', 1); hold on;
    end
    if (~isempty(p4))
        scatter(p4(1,1), p4(2,1), sz, 'r', 'o', 'Parent', oAxEdges_h, 'LineWidth', lw); hold on;
        scatter(p4(1,1), p4(2,1), sz, 'r', 'x', 'Parent', oAxEdges_h, 'LineWidth', 1); hold on;
    end
    bright = .9; dark = .6; sz = 20;
    color1 = [bright dark dark];
    drawRectangle(oAxEdges_h, color1, subRect1, 0); hold on;
    if(~isempty(PsRect1))
        scatter(PsRect1(:,1), PsRect1(:,2), sz, color1, 'o', 'Parent', oAxEdges_h);
    end
    
    color2 = [dark dark bright];
    drawRectangle(oAxEdges_h, color2, subRect2, 0); hold on;
    if(~isempty(PsRect2))
        scatter(PsRect2(:,1), PsRect2(:,2), sz, color2, 'o', 'Parent', oAxEdges_h);
    end
    
    color3 = [bright dark bright];
    drawRectangle(oAxEdges_h, [.5 .5 .7], subRect3, 0); hold on;
    if(~isempty(PsRect3))
        scatter(PsRect3(:,1), PsRect3(:,2), sz, color3, 'o', 'Parent', oAxEdges_h);
    end
    
    color4 = [dark bright bright];
    drawRectangle(oAxEdges_h, color4, subRect4, 0); hold on;
    if(~isempty(PsRect4))
        scatter(PsRect4(:,1), PsRect4(:,2), sz, color4, 'o', 'Parent', oAxEdges_h);
    end
    axis square;
end

end

% This function estimates the edge point from points within a subrectangle.
function p_edge = getEdgePoint(PsRect, ctr, target_point)
    % Edge point estimated by taking the median of the closest points to target point (edge of roughly estimated
    % rectangle position)
    distFnc = @(v) sqrt((target_point(1,1)-v(1,1))^2 + (target_point(2,1)-v(1,2))^2);

    PsRect = PsRect(1:ctr-1,:);
    cellPsRect1 = num2cell(PsRect, 2);
    distance = cellfun(distFnc, cellPsRect1);
    PsRect(:,3) = distance;
    PsRect = sortrows(PsRect,3);

    if (size(PsRect,1) < 10)
        top = PsRect;
    else
        top = PsRect(1:10,:);
    end

    if (size(top,1) == 1)
        med = top;
    else
        med = median(top);
    end

    p_edge(1,1) = med(1,1); p_edge(2,1) = med(1,2);
end