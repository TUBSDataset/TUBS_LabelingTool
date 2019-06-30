function [Dx, Dy, fYawRegistered, vfMiddleRegistered, dqs, bYawTrustable, mfRegisteredPoints] = ...
    registerBoxesICP(   mfCurrentPoints, ...
                        mfPreviousPoints, ...
                        vfPreviousMiddleXY_CSRF, ...
                        fPreviousBBYaw_CSRF, ...
                        P2C, ...
                        nMaxit, ...
                        oAxRegis1_h, oAxRegis2, oCur_h, oPrev_h)
% ---------------------------------------------------------------------------------------------
% Function registerBoxesICP(...) registeres two point clouds and provides the transformation between both point clouds.
% Two registration directions are possible. Empirically, registering fewer points (current points) with more points
% (previous points) yield better results (P2C = 0). For both directions, an estimation of the current point cloud's
% poistion and orientation is given. However, this estimation accumulates erros because ICP only provided a relative
% transformation.
%
% INPUT:
%   mfCurrentPoints:            n-by-3 matrix containing current points
%   mfPreviousPoints:           n-by-3 matrix containing previous points
%   vfPreviousMiddleXY_CSRF:    Object's middle in previous frame, rotated into current reference frame
%   fPreviousBBYaw_CSRF:        Object's yaw in previous frame, rotated into current reference frame
%   P2C:                        Boolean, specifying registration direction (0: current points are registered to previous
%                               points), impacts the way the transformation from previous to current point cloud is computed
%   nMaxit:                     Maximum number of iterations
%   axRegis1:                   Axes for computation plots
%   axRegis2:                   Axes for result plots
%   oCur_h:                     Handle to current points' scatter plot
%   oPrev_h:                    Handle to previous point's scatter plot

% OUTPUT:
%   Dx:                         Translation in x direction between both point clouds
%   Dy:                         Translation in y direction between both point clouds
%   fYawRegistered:             The current point cloud's yaw, estimated by tracking the transformations done by ICP
%   vfMiddleRegistered:         The current point cloud's middle, estimated by tracking the transformations done by ICP
%   dqs:                        Mean squared distances between points (confidence metric)
%   bYawTrustable:              False if rotation matrix is invalid
%   mfRegisteredPoints:         Transformed previous point cloud
% --------------------------------------------------------------------------------------------- 

if ~isempty(oAxRegis1_h)
   if P2C
       reg_h = oPrev_h;
   else
       reg_h = oCur_h;
   end
end

Dx = 0; Dy = 0; fYawRegistered = 0; vfMiddleRegistered = [0;0;0]; dqs = 1000; bYawTrustable = 0; mfRegisteredPoints = [];

% lenPss:       Two points, used to track the point cloud's orientation
% pointTrack_o: Original point within source point cloud, used to track the PC's position
% pointTrack:   Transformed original point

if(P2C)
    % Previous point cloud is source, registered to current point cloud
    lvec(1,1) = cos(fPreviousBBYaw_CSRF*pi/180);    
    lvec(2,1) = sin(fPreviousBBYaw_CSRF*pi/180);
    lvec(3,1) = 0;
    lenPss(:,1)     = vfPreviousMiddleXY_CSRF;          % First point (orientation track): middle
    lenPss(:,2)     = vfPreviousMiddleXY_CSRF + lvec;   % Second point (orientation track): middle + length
    pointTrack_o    = vfPreviousMiddleXY_CSRF;          % Original point (position track): middle
    pointTrack      = pointTrack_o;
else
    % Current point cloud is registered to previous point cloud 
    lenPss(:,1) = mfCurrentPoints(1,:);                 % First point (orientation track): arbitrary
    lenPss(:,2) = mfCurrentPoints(2,:);                 % Second point (orientation track): arbitrary
    lvec = lenPss(:,2) - lenPss(:,1);                   
    yaw_o = atan2(lvec(2,1),lvec(1,1))*180/pi();        % Original yaw of length vector between two arbitrary points
    pointTrack_o = mfCurrentPoints(1,:)';               % Original point (position track): arbitrary
    pointTrack   = pointTrack_o;
end


%% Settings
threshold_on    = 0;        % Activate thresholding
threshold_scale = 2;        % Scaling factor for threshold
abortdist       = 10^-6;    % Mean distance threshold below which to abort registration

%% Registration
meandistvec = zeros(nMaxit,2);  % Vector, storing the mean distance for every iteration
stop = 0;                       % True if convergence is reached
Racc = eye(3,3);                % Accumulated rotation matrix
bYawTrustable = 1;

if(P2C)
    Pb               = [mfCurrentPoints(:,1),mfCurrentPoints(:,2),mfCurrentPoints(:,3)];      % destination
    pointsToRegister = [mfPreviousPoints(:,1),mfPreviousPoints(:,2),mfPreviousPoints(:,3)];   % source to be registered with Pb
else
    Pb               = [mfPreviousPoints(:,1),mfPreviousPoints(:,2),mfPreviousPoints(:,3)];
    pointsToRegister = [mfCurrentPoints(:,1),mfCurrentPoints(:,2),mfCurrentPoints(:,3)];
end

for numit = 1:nMaxit
    Pr = pointsToRegister;
    % Find nearest neighbour (brute force)
    for i = 1 : size(Pr,1)
        pdist = 10^6;
        for j = 1 : size(Pb,1)
            pdistcur = (Pb(j,1)-Pr(i,1))^2 + (Pb(j,2)-Pr(i,2))^2 + (Pb(j,3)-Pr(i,3))^2;
            if pdistcur < pdist
                Pr(i,4) = j;
                Pr(i,5) = pdistcur;
                pdist = pdistcur;
            end
        end
    end
    
    % Compute threshold, mean squared distance and check for convergence
    Mb = mean(Pb,1);
    Mr = mean(Pr,1);
    
    dqs = Mr(1,5);      % Mean of squared distances
    meandistvec(numit,1) = numit;
    meandistvec(numit,2) = dqs;
    
    % Check for convergence
    if numit > 2
        if ((meandistvec(numit-1,2)-(meandistvec(numit,2)) < abortdist))
            stop = 1;
        end
    end
    
    %% Thresholding
    threshold = threshold_scale*Mr(1,5);
    for i = 1 : size(Pr,1)
        if ((Pr(i,5) > threshold) && threshold_on)
            Pr(i,6) = -1;
        else
            Pr(i,6) = 0;
        end
    end
    
    %% Calculate centroids, qq = model, pq = data to align
    qq = [Mb(:,1) Mb(:,2) Mb(:,3)];
    pq = [Mr(:,1) Mr(:,2) Mr(:,3)];
    
    %% Build point pair matrix
    count = 0;
    for i=1:size(Pr,1)
        if Pr(i,6) == 0
            count = count + 1;
        end
    end
    PP = zeros(count, 6);
    index = 1;
    for i=1:size(Pr,1)
        if Pr(i,6) == 0
            PP(index,1) = Pr(i,1);
            PP(index,2) = Pr(i,2);
            PP(index,3) = Pr(i,3);
            indexb = Pr(i,4);
            PP(index,4) = Pb(indexb,1);
            PP(index,5) = Pb(indexb,2);
            PP(index,6) = Pb(indexb,3);
            index = index + 1;
        end
    end
    
    %% Rotation matrix R
    %  build pi, qi Matrices
    pei = zeros(count,3);
    qi = zeros(count,3);
    for i=1:count
        pei(i,1) = PP(i,1)-pq(1,1);
        pei(i,2) = PP(i,2)-pq(1,2);
        pei(i,3) = PP(i,3)-pq(1,3);
        
        qi(i,1) = PP(i,4)-qq(1,1);
        qi(i,2) = PP(i,5)-qq(1,2);
        qi(i,3) = PP(i,6)-qq(1,3);
    end
    % build N-Matrix
    N = zeros(3,3);
    for i=1:count
        Nacc = transpose(pei(i,:))*qi(i,:);
        N = N + Nacc;
    end
    % singular value decomposition
    [U, ~, V] = svd(N);
    
    R = V*transpose(U);
    
    %% Apply transformations
    %  Refine R
    %  Deny roll and pitch -> calc and only apply yaw
    pRot = R*pointTrack;
    yaw  = atan2(pRot(2,1), pRot(1,1)) - atan2(pointTrack(2,1), pointTrack(1,1));
    R = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    Rinv = [cos(-yaw) -sin(-yaw) 0; sin(-yaw) cos(-yaw) 0; 0 0 1];
    Racc = Rinv*Racc;   % Accumulate backwards transformation
    
    % Calculate current translation
    T = transpose(qq) - R*transpose(pq);
    
    % Deny vertical translation
    T(3,1) = 0;
    
    % Transform
    pointsToRegister = pointsToRegister';
    pointsToRegister = R*pointsToRegister +  repmat(T, [1, size(pointsToRegister,2)]);
    pointsToRegister = pointsToRegister';
    
    % Transform the length vector (two defined points, respectively), orientation track
    lenPss = R*lenPss + repmat(T,[1, size(lenPss,2)]);
    
    % Tranfsform the position track 
    pointTrack = R*pointTrack + T;
    
    % Plot
    if(~isempty(oAxRegis1_h))
       reg_h.XData =  pointsToRegister(:,1);
       reg_h.YData =  pointsToRegister(:,2);
       drawnow;
       pause(0.1);
    end
    
    % Stop if convergence is reached
    if stop == 1
        break;
    end
end

%% Compute difference between actual and result transformations

% Check for valid rotation matrix
if abs(det(Racc(:,:))+1)<0.05
    bYawTrustable = 0; 
end

% Compute length vector from orientation track points
lvec = lenPss(:,2) - lenPss(:,1); 

if P2C
    % Previous point cloud registered to current point cloud
    fYawRegistered     = atan2(lvec(2,1), lvec(1,1))*180/pi();   % estimated yaw is transformed orientation track
    mfRegisteredPoints = pointsToRegister;

    Tacc = pointTrack - pointTrack_o;   % Accumulated translation: distance between position track (here: middle of previous (rotated) point cloud)
    Dx = Tacc(1,1);
    Dy = Tacc(2,1);
    vfMiddleRegistered = pointTrack;    % pointTrack = vfPreviousMiddleXY_CSRF
else
    % Current point cloud reigstered to previous point cloud
    % Apply inverse transformation to registered point cloud (accumulated during iterations)
    mfRegisteredPoints = Racc*(mfPreviousPoints');
    
    % Rotate point track back to calculate overall translation
    pointTrack = Racc * pointTrack;     
    Tacc       = pointTrack_o - pointTrack;
    
    % Translate previous points according to overall translation
    mfRegisteredPoints = mfRegisteredPoints + repmat(Tacc, [1, size(mfRegisteredPoints,2)]);
    mfRegisteredPoints = mfRegisteredPoints';

    % Compute estimation for the current point cloud's middle
    vfMiddleRegistered  = Racc*vfPreviousMiddleXY_CSRF + Tacc;
    DMiddle             = vfMiddleRegistered - vfPreviousMiddleXY_CSRF;
    Dx = DMiddle(1,1);
    Dy = DMiddle(2,1);
    
    % Compute estimatation for the current point cloud's orientation
    % Calculate difference in orientation before and after registration of the current point cloud
    yaw_cur = atan2(lvec(2,1),lvec(1,1))*180/pi();
    if yaw_cur < 0
        yaw_cur = yaw_cur + 360;
    end
    Dyaw = calculateDeltaYaw360(yaw_cur, yaw_o);
    
    % Apply inverse delta to previous point cloud
    fYawRegistered = fPreviousBBYaw_CSRF - Dyaw;
end

% Match to 0 ... 360
if fYawRegistered > 360
    fYawRegistered = fYawRegistered - 360;
elseif(fYawRegistered < 0)
    fYawRegistered = fYawRegistered + 360;
end

% Plot 
if ~isempty(oAxRegis1_h)
    set(reg_h, 'MarkerEdgeColor', [0 .9 .1], 'MarkerFaceColor', [0 .9 .1]);
    
    % scatter3(previousPoints(:,1), previousPoints(:,2), previousPoints(:,3), 20, [.3 .3 .3], '.', 'Parent', axRegis2);
    set(oAxRegis2, 'Projection', 'perspective', 'DataAspectRatio', [1 1 1.2], 'NextPlot', 'add');
    scatter3(mfRegisteredPoints(:,1), mfRegisteredPoints(:,2), mfRegisteredPoints(:,3), 20, [0 .6 .1], '.', 'Parent', oAxRegis2);
    scatter3(mfCurrentPoints(:,1),    mfCurrentPoints(:,2),    mfCurrentPoints(:,3),    20, [0  1  0], '.', 'Parent', oAxRegis2);
end

end

