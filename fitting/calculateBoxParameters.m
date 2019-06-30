function [oMovableLabel, bSuccess] = calculateBoxParameters(mfEdgePoints)
% ---------------------------------------------------------------------------------------------
% Function calculateBoxParameters(...) rectifies a set of given edge points to an perpendicular rectangle. 
%
% INPUT:
%   mfEdgePoints:       Matrix of rectified edge points.
%   
%
% OUTPUT:
%   oMovableLabel:      Object label with calculated size parameters.  
%   bSuccess:           Boolean, false if no success.
% ---------------------------------------------------------------------------------------------

oMovableLabel = cPCMovableLabel();
bSuccess = -1;

ind = mfEdgePoints(5,:)';
p1 = mfEdgePoints(1,:)';
p2 = mfEdgePoints(2,:)';
p3 = mfEdgePoints(3,:)';
p4 = mfEdgePoints(4,:)';

%% Calculate BBMiddle
cross1 = [0;0]; src1 = [0;0]; 
cross2 = [0;0]; src2 = [0;0];  

% Check for intersection
p13 = p3 - p1;
p42 = p2 - p4;
res = p13./p42;
if(abs((res(2,1) - res(1,1))) > 0.001) % intersecting
    cross1 = p13; src1 = p1; 
    cross2 = p42; src2 = p4; 
else
   p12 = p2 - p2;
   p43 = p3 - p4;
   res = p12./p43;
   if(abs((res(2,1) - res(1,1))) > 0.001)
       cross1 = p12; src1 = p1; 
       cross2 = p43; src2 = p4; 
   else
       return; % something went wrong here
   end 
end

A = [cross1(1,1) -cross2(1,1);
     cross1(2,1) -cross2(2,1)];
b = [src2(1,1)-src1(1,1); src2(2,1)-src1(2,1)];
X = A\b;
intersec = src2 + X(2,1)*cross2;
oMovableLabel.m_fBBMiddle_x = intersec(1,1);
oMovableLabel.m_fBBMiddle_y = intersec(2,1);

%% Calculate yaw
p12 = p2 - p1; p23 = p3 - p2; p34 = p4 - p3; p41 = p1- p4;
mid12 = p1 + p12/2; mid23 = p2 + p23/2; mid34 = p3 + p34/2; mid41 = p4 + p41/2;

distList = cell(4,3);
distList(1,1) = {'dist12'}; distList(1,2) = {norm(ind-mid12)}; distList(1,3) = {mid12}; 
distList(2,1) = {'dist23'}; distList(2,2) = {norm(ind-mid23)}; distList(2,3) = {mid23}; 
distList(3,1) = {'dist34'}; distList(3,2) = {norm(ind-mid34)}; distList(3,3) = {mid34}; 
distList(4,1) = {'dist41'}; distList(4,2) = {norm(ind-mid41)}; distList(4,3) = {mid41}; 
distList = sortrows(distList,2);

lvec = distList{1,3} - intersec;
wvec = distList{3,3} - intersec;
oMovableLabel.m_fBBYaw = atan2(lvec(2,1),lvec(1,1))*180/pi();
if(oMovableLabel.m_fBBYaw < 0)
   oMovableLabel.m_fBBYaw = oMovableLabel.m_fBBYaw + 360; 
end
oMovableLabel.m_fBBLength = norm(lvec)*2;
oMovableLabel.m_fBBWidth = norm(wvec)*2;
oMovableLabel.m_fBBHeight = 0;

bSuccess = 1;
end

