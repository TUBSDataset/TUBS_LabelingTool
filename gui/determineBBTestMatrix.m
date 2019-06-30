function [mfBBTestMatrix] = determineBBTestMatrix(voPCMovableLabel, fMarginLP, fMarginLN, fMarginWP, fMarginWN )
% ---------------------------------------------------------------------------------------------
% Function determineBBTestMatrix(...) determines a coefficient matrix in order to test a point for its
% position within a bounding box.
%
% INPUT:
%   voPCMovabelLabel:   Vector of class cPCMovableLabel containing bounding box information
%   fMarginLP:          Margin in positive length direction (in bounding box yaw orientation)
%   fMarginLN:          Margin in negative length direction
%   fMarginWP:          Margin in positive width direction (perpendicular to the length vector (counterclockwise)
%   fMarginWN:          Margin in negative width direction
% ---------------------------------------------------------------------------------------------

nNumObjects     = size(voPCMovableLabel, 1);
mfBBTestMatrix  = zeros(nNumObjects, 12);

for i = 1 : nNumObjects
    m_fBBYaw = voPCMovableLabel(i,1).m_fBBYaw * pi()/180;
    % Create edge points
    bbMiddlePlane(1,1) = voPCMovableLabel(i,1).m_fBBMiddle_x;
    bbMiddlePlane(2,1) = voPCMovableLabel(i,1).m_fBBMiddle_y;
    
    lvec(1,1) = (voPCMovableLabel(i,1).m_fBBLength)/2*cos(m_fBBYaw);
    lvec(2,1) = (voPCMovableLabel(i,1).m_fBBLength)/2*sin(m_fBBYaw);
    wvec(1,1) = (voPCMovableLabel(i,1).m_fBBWidth)/2*cos(m_fBBYaw + pi()/2);
    wvec(2,1) = (voPCMovableLabel(i,1).m_fBBWidth)/2*sin(m_fBBYaw + pi()/2);
    
    % Make margin vectors, same direction as lvec and wvec
    marginLPvec = fMarginLP .* lvec ./ norm(lvec);
    marginLNvec = fMarginLN .* lvec ./ norm(lvec);
    marginWPvec = fMarginWP .* wvec ./ norm(wvec);
    marginWNvec = fMarginWN .* wvec ./ norm(wvec);
    
    frontLeft = lvec + wvec + bbMiddlePlane;
    backLeft = -lvec + wvec + bbMiddlePlane;
    backRight = -lvec - wvec + bbMiddlePlane;
    frontRight = lvec - wvec + bbMiddlePlane;
    
    p1 = frontLeft + marginLPvec + marginWPvec;
    p2 = backLeft - marginLNvec + marginWPvec;
    p3 = backRight - marginLNvec - marginWNvec;
    p4 = frontRight + marginLPvec - marginWNvec;
    
    % Edge 1
    [A12, B12, C12] = ABCFromPoint(p1,p2);
    % Edge 2
    [A23, B23, C23] = ABCFromPoint(p2,p3);
    % Edge 3
    [A34, B34, C34] = ABCFromPoint(p3,p4);
    % Edge 4
    [A41, B41, C41] = ABCFromPoint(p4,p1);
    
    mfBBTestMatrix(i,1) = A12;
    mfBBTestMatrix(i,2) = B12;
    mfBBTestMatrix(i,3) = C12;
    mfBBTestMatrix(i,4) = A23;
    mfBBTestMatrix(i,5) = B23;
    mfBBTestMatrix(i,6) = C23;
    mfBBTestMatrix(i,7) = A34;
    mfBBTestMatrix(i,8) = B34;
    mfBBTestMatrix(i,9) = C34;
    mfBBTestMatrix(i,10) = A41;
    mfBBTestMatrix(i,11) = B41;
    mfBBTestMatrix(i,12) = C41;
end

end

