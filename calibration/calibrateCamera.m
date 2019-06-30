function [] = calibrateCamera(clPointContainer, voPCMovableLabel, oCalibration, oAxes_h)
% ---------------------------------------------------------------------------------------------
% Function calibrateCamera(...) recalibrates a camera's intrinsics and extrinsics according to given
% point correspondences. Parameters are optimized using a stochastic algorithm that minimizes the projection error.
% Samples minimizing the error around the current parameter set are averaged and applied after each iteration according
% to a specified learning rate.
%
% INPUT:
%   clPointContainer:       N-by-2 cell array containing point correspondences (n: object. 1. column: pixels, 2. column: 3D)
%   oCalibration:           Object of class cCalibration containing an initial parameter guess
%   oAxes_h:                Saved editor graphic for visualization.
%
% OUTPUT:
%   -

% DETAILS:

% Order of point correspondeces in clPointContainer: 1. top left, bottom left, bottom right, top right
% Requirements: MATLAB's robotics toolbox for rotm2eul.
% ---------------------------------------------------------------------------------------------

% Settings
bDemoMode = 1;

nNumIterations  = 3000;             % Number of optimization steps
nNumSamples     = 100;              % Number of samples to be averaged around the current parameter set
fLearningRate   = 1;                % Learning rate

sig_int     = 1;        % standard deviation intrinsics
sig_dist    = 0.005;    % standard deviation distortion
sig_trans   = 0.005;    % standard deviation translation
sig_rot     = 0.005;    % standard deviation rotation

if ~bDemoMode
    nProgressMonitor    = 100;  % Plot and monitor progress every n steps
    nPlotMonitor        = 100;
else
    nProgressMonitor    = 1;  
    nPlotMonitor        = 1;
end

%% Get calibration parameters
K   = oCalibration.m_mfK;
R   = oCalibration.m_mfR;
cs  = oCalibration.m_vfc;
d   = oCalibration.m_vfd;

if bDemoMode
    K(2,3) = K(2,3) + 50; % change inital guess
end

% Intrinsics
fx = K(1,1);
u0 = K(1,3);
fy = K(2,2);
v0 = K(2,3);

k1 = d(1,1);
k2 = d(1,2);

% Extrinsics
cs_x = cs(1,1);
cs_y = cs(2,1);
cs_z = cs(3,1);

% Get Euler angles
eul = rotm2eul(R);  % Z Y X

rot_z = eul(1,1);
rot_y = eul(1,2);
rot_x = eul(1,3);


%% Generate point correspondences
nObj = size(clPointContainer, 1);    % number of target objects
clCorrespondences = cell(nObj, 4);  % 2D, 3D, color

for i = 1 : nObj
    % Get points from container
    clCorrespondences{i,1} = clPointContainer{i,1};
    
    % Compute 3D points from bounding box
    oLabel = voPCMovableLabel(i,1);
    fBBYaw = oLabel.m_fBBYaw * pi()/180;
    fBBHeight = oLabel.m_fBBHeight;
    vfBBMiddle = [oLabel.m_fBBMiddle_x; oLabel.m_fBBMiddle_y];
    
    lvec(1,1) = oLabel.m_fBBLength/2*cos(fBBYaw);
    lvec(2,1) = oLabel.m_fBBLength/2*sin(fBBYaw);
    wvec(1,1) = oLabel.m_fBBWidth/2*cos(fBBYaw + pi()/2);
    wvec(2,1) = oLabel.m_fBBWidth/2*sin(fBBYaw + pi()/2);
    
    vfFrontLeft = vfBBMiddle + lvec + wvec;
    vfBackLeft  = vfBBMiddle - lvec + wvec;
    p_3D_1 = [vfFrontLeft;  oLabel.m_fBBMiddle_z + fBBHeight/2];
    p_3D_2 = [vfFrontLeft;  oLabel.m_fBBMiddle_z - fBBHeight/2];
    p_3D_3 = [vfBackLeft;   oLabel.m_fBBMiddle_z - fBBHeight/2];
    p_3D_4 = [vfBackLeft;   oLabel.m_fBBMiddle_z + fBBHeight/2];
    
    clCorrespondences{i,2} = [p_3D_1'; p_3D_2'; p_3D_3'; p_3D_4'];
    
    % Generate color
    clCorrespondences{i,4} = [rand(1,1) rand(1,1) rand(1,1)];
end

%% Project and plot correspondences initially

voProjections_h = gobjects(nObj*4,1);
voLines_h       = gobjects(nObj*4,1);
nCtr = 0;
for i = 1 : nObj
    col  = clCorrespondences{i,4};
    mf2D = clCorrespondences{i,1};
    mf3D = clCorrespondences{i,2};
    
    vf2DProjections = zeros(4,2);
    
    for j = 1 : 4
        nCtr = nCtr + 1;
        % Project
        w = ones(4,1);
        w(1:3,1) = mf3D(j,:)';
        w = w(1:3,1);
        
        % Build rotation
        R_z = [ 
                cos(rot_z)  -sin(rot_z)  0;
                sin(rot_z)  cos(rot_z)   0;
                0           0           1];
        
        R_y = [ 
                cos(rot_y)  0   sin(rot_y);
                0           1   0;
                -sin(rot_y) 0   cos(rot_y)];
        
        R_x = [ 
                1   0           0;
                0   cos(rot_x)  -sin(rot_x);
                0   sin(rot_x)  cos(rot_x)];
        
        R = R_z * R_y * R_x;
        
        % Distort
        w = R*(w-cs);
        w = w./w(3,1);
        x = w(1,1);
        y = w(2,1);
        k1 = d(1,1);
        k2 = d(1,2);
        
        r  = x^2 + y^2;
        % x = x * (1 + k1*r^2 + k2*r^4);
        % y = y * (1 + k1*r^2 + k2*r^4);
        x = x * (1 + k1*r + k2*r^2);
        y = y * (1 + k1*r + k2*r^2);
        pix = K * [x; y; 1];
        
        vf2DProjections(j,:) = pix(1:2,1)';
        
        % Plot
        scatter(mf2D(j,1), mf2D(j,2), 50, col, 'o', 'Parent', oAxes_h);
        voProjections_h(nCtr,1) = scatter(pix(1,1), pix(2,1), 50, col, 'x', 'Parent', oAxes_h);
        voLines_h(nCtr,1) = plot([pix(1,1); mf2D(j,1)], [pix(2,1); mf2D(j,2)], 'r', 'Parent', oAxes_h);
    end
    
    clCorrespondences{i,3} = vf2DProjections;
end

%% Stochastic optimization

nPoints = nObj * 4;
% nPoints = size(vnSelection,1)*4;

% Convergence
con_fx = zeros(nNumIterations,1);
con_u0 = zeros(nNumIterations,1);
con_fy = zeros(nNumIterations,1);
con_v0 = zeros(nNumIterations,1);

con_k1 = zeros(nNumIterations,1);
con_k2 = zeros(nNumIterations,1);

con_cs_x = zeros(nNumIterations,1);
con_cs_y = zeros(nNumIterations,1);
con_cs_z = zeros(nNumIterations,1);

con_rot_z = zeros(nNumIterations,1);
con_rot_y = zeros(nNumIterations,1);
con_rot_x = zeros(nNumIterations,1);

con_err = zeros(nNumIterations,1);

con_fx(1,1) = fx;
con_u0(1,1) = u0;
con_fy(1,1) = fy;
con_v0(1,1) = v0;
con_k1(1,1) = k1;
con_k2(1,1) = k2;
con_cs_x(1,1) = cs_x;
con_cs_y(1,1) = cs_y;
con_cs_z(1,1) = cs_z;
con_rot_x(1,1) = rot_x;
con_rot_y(1,1) = rot_y;
con_rot_z(1,1) = rot_z;

fx_best = fx;
u0_best = u0;
fy_best = fy;
v0_best = v0;
k1_best = k1;
k2_best = k2;
cs_x_best = cs_x;
cs_y_best = cs_y;
cs_z_best = cs_z;
rot_x_best = rot_x;
rot_y_best = rot_y;
rot_z_best = rot_z;

% Compute inital projection error
fErrorSum_min = 0.0;
for i = 1 : nObj
    mf2D_GT = clCorrespondences{i,1};
    mf2D_pr = clCorrespondences{i,3};
    
    for j = 1 : 4
        fErrorSum_min = fErrorSum_min + (mf2D_GT(j,1) - mf2D_pr(j,1))^2 + (mf2D_GT(j,2) - mf2D_pr(j,2))^2;
    end
end
fErrorSum_min = fErrorSum_min/nPoints;

con_err(1,1) = fErrorSum_min;

%% Optimization loop
drawnow;
disp('Optimization started.');

for i = 2 : nNumIterations
    % Monitor progress
    if mod(i, nProgressMonitor) == 0
        fprintf('Iteration: %d / %d \n', i, nNumIterations);
    end
    
    % Create variatons
    v_fx = repmat(fx_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_int;
    v_u0 = repmat(u0_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_int;
    v_fy = repmat(fy_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_int;
    v_v0 = repmat(v0_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_int;
    
    v_k1 = repmat(k1_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_dist;
    v_k2 = repmat(k2_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_dist;
    
    v_cs_x = repmat(cs_x_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_trans;
    v_cs_y = repmat(cs_y_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_trans;
    v_cs_z = repmat(cs_z_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_trans;
    
    v_rot_x = repmat(rot_x_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_rot;
    v_rot_y = repmat(rot_y_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_rot;
    v_rot_z = repmat(rot_z_best, nNumSamples,1) + randn(nNumSamples, 1) .* sig_rot;
    
    % Project: Iterate all samples
    nAverageCtr = 0;
    
    % Updates
    ufx = 0;
    uu0 = 0;
    ufy = 0;
    uv0 = 0;
    uk1 = 0;
    uk2 = 0;
    u_cs_x = 0;
    u_cs_y = 0;
    u_cs_z = 0;
    u_rot_x = 0;
    u_rot_y = 0;
    u_rot_z = 0;
    
    for k = 1 : nNumSamples
        % Iterate all objects and points
        fErrorSum = 0.0;
        
        % Get sampled parameters
        pfx = v_fx(k,1);
        pu0 = v_u0(k,1);
        pfy = v_fy(k,1);
        pv0 = v_v0(k,1);
        
        pk1 = v_k1(k,1);
        pk2 = v_k2(k,1);
        
        pcs_x = v_cs_x(k,1);
        pcs_y = v_cs_y(k,1);
        pcs_z = v_cs_z(k,1);
        
        prot_x = v_rot_x(k,1);
        prot_y = v_rot_y(k,1);
        prot_z = v_rot_z(k,1);
        
        for o = 1 : nObj
            mf2D_GT = clCorrespondences{o,1};
            mf3D    = clCorrespondences{o,2};
            
            % Iterate all points and project
            for j = 1 : 4
                w = ones(4,1);
                w(1:3,1) = mf3D(j,:)';
                w = w(1:3,1);
                
                % Build rotation
                R_z = [ cos(prot_z)     -sin(prot_z)    0;
                    sin(prot_z)     cos(prot_z)     0;
                    0               0               1];
                
                R_y = [ cos(prot_y)     0       sin(prot_y);
                    0               1       0;
                    -sin(prot_y)    0       cos(prot_y)];
                
                R_x = [ 1       0               0;
                    0       cos(prot_x)     -sin(prot_x);
                    0       sin(prot_x)     cos(prot_x)];
                
                R = R_z * R_y * R_x;
                
                % Build K
                K = [pfx    0       pu0;
                    0      pfy     pv0;
                    0      0       1];
                
                % Translate
                cs = [pcs_x; pcs_y; pcs_z];
                
                w = R*(w-cs);
                w = w./w(3,1);
                x = w(1,1);
                y = w(2,1);
                
                % Distort
                r  = x^2 + y^2;
                % x = x * (1 + pk1*r^2 + pk2*r^4);
                % y = y * (1 + pk1*r^2 + pk2*r^4);
                x = x * (1 + k1*r + k2*r^2);
                y = y * (1 + k1*r + k2*r^2);
                
                pix = K * [x; y; 1];
                
                % Sum up error
                fErrorSum = fErrorSum + (mf2D_GT(j,1) - pix(1,1))^2 + (mf2D_GT(j,2) - pix(2,1))^2;
                
                % Plot
                % scatter(pix(1,1), pix(2,1), 50, [1 0 0], 'x', 'Parent', oAxes_h);
            end
        end
        fErrorSum = fErrorSum/nPoints;
        
        % Average all sets that minimize the error function
        if fErrorSum < fErrorSum_min
            nAverageCtr = nAverageCtr + 1;
            
            ufx = ufx * (nAverageCtr-1)/nAverageCtr + pfx / nAverageCtr;
            uu0 = uu0 * (nAverageCtr-1)/nAverageCtr + pu0 / nAverageCtr;
            ufy = ufy * (nAverageCtr-1)/nAverageCtr + pfy / nAverageCtr;
            uv0 = uv0 * (nAverageCtr-1)/nAverageCtr + pv0 / nAverageCtr;
            uk1 = uk1 * (nAverageCtr-1)/nAverageCtr + pk1 / nAverageCtr;
            uk2 = uk2 * (nAverageCtr-1)/nAverageCtr + pk2 / nAverageCtr;
            
            u_cs_x = u_cs_x * (nAverageCtr-1)/nAverageCtr + pcs_x / nAverageCtr;
            u_cs_y = u_cs_y * (nAverageCtr-1)/nAverageCtr + pcs_y / nAverageCtr;
            u_cs_z = u_cs_z * (nAverageCtr-1)/nAverageCtr + pcs_z / nAverageCtr;
            
            u_rot_x = u_rot_x * (nAverageCtr-1)/nAverageCtr + prot_x / nAverageCtr;
            u_rot_y = u_rot_y * (nAverageCtr-1)/nAverageCtr + prot_y / nAverageCtr;
            u_rot_z = u_rot_z * (nAverageCtr-1)/nAverageCtr + prot_z / nAverageCtr;
        end
    end
    
    % Better sets found: Compute update
    if nAverageCtr > 0
        fx_best = fx_best + (ufx - fx_best) / fLearningRate;
        u0_best = u0_best + (uu0 - u0_best) / fLearningRate;
        fy_best = fy_best + (ufy - fy_best) / fLearningRate;
        v0_best = v0_best + (uv0 - v0_best) / fLearningRate;
        k1_best = k1_best + (uk1 - k1_best) / fLearningRate;
        k2_best = k2_best + (uk2 - k2_best) / fLearningRate;
        
        cs_x_best = cs_x_best + (u_cs_x - cs_x_best) / fLearningRate;
        cs_y_best = cs_y_best + (u_cs_y - cs_y_best) / fLearningRate;
        cs_z_best = cs_z_best + (u_cs_z - cs_z_best) / fLearningRate;
        
        rot_x_best = rot_x_best + (u_rot_x - rot_x_best) / fLearningRate;
        rot_y_best = rot_y_best + (u_rot_y - rot_y_best) / fLearningRate;
        rot_z_best = rot_z_best + (u_rot_z - rot_z_best) / fLearningRate;
    end
    
    % Set convergence
    con_fx(i,1) = fx_best;
    con_u0(i,1) = u0_best;
    con_fy(i,1) = fy_best;
    con_v0(i,1) = v0_best;
    con_k1(i,1) = k1_best;
    con_k2(i,1) = k2_best;
    
    con_cs_x(i,1) = cs_x_best;
    con_cs_y(i,1) = cs_y_best;
    con_cs_z(i,1) = cs_z_best;
    
    con_rot_x(i,1) = rot_x_best;
    con_rot_y(i,1) = rot_y_best;
    con_rot_z(i,1) = rot_z_best;
    
    % Compute current projection error
    fErrorSum = 0.0;
    nCtr = 0;
    for o = 1 : nObj
        mf2D_GT = clCorrespondences{o,1};
        mf3D    = clCorrespondences{o,2};
        
        % Get current parameters (best)
        pfx = con_fx(i,1);
        pu0 = con_u0(i,1);
        pfy = con_fy(i,1);
        pv0 = con_v0(i,1);
        
        pk1 = con_k1(i,1);
        pk2 = con_k2(i,1);
        
        pcs_x = con_cs_x(i,1);
        pcs_y = con_cs_y(i,1);
        pcs_z = con_cs_z(i,1);
        
        prot_x = con_rot_x(i,1);
        prot_y = con_rot_y(i,1);
        prot_z = con_rot_z(i,1);
        
        % Iterate all points
        for j = 1 : 4
            nCtr = nCtr + 1;
            
            % Project
            w = ones(4,1);
            w(1:3,1) = mf3D(j,:)';
            w = w(1:3,1);
            
            % Build rotation
            R_z = [ ...
                cos(prot_z)     -sin(prot_z)    0;
                sin(prot_z)     cos(prot_z)     0;
                0               0               1];
            
            R_y = [ ...
                cos(prot_y)     0       sin(prot_y);
                0               1       0;
                -sin(prot_y)    0       cos(prot_y)];
            
            R_x = [ ...
                1       0               0;
                0       cos(prot_x)     -sin(prot_x);
                0       sin(prot_x)     cos(prot_x)];
            
            R = R_z * R_y * R_x;
            
            % Build K
            K = [...
                pfx    0       pu0;
                0      pfy     pv0;
                0      0       1];
            
            % Translate
            cs = [pcs_x; pcs_y; pcs_z];
            
            w = R*(w-cs);
            w = w./w(3,1);
            x = w(1,1);
            y = w(2,1);
            
            % Distort
            r  = x^2 + y^2;
            % x = x * (1 + pk1*r^2 + pk2*r^4);
            % y = y * (1 + pk1*r^2 + pk2*r^4);
            x = x * (1 + pk1*r + pk2*r^2);
            y = y * (1 + pk1*r + pk2*r^2);
            pix = K * [x; y; 1];
            
            % Sum up error
            fErrorSum = fErrorSum + (mf2D_GT(j,1) - pix(1,1))^2 + (mf2D_GT(j,2) - pix(2,1))^2;
            
            % Plot progress
            if mod(i, nPlotMonitor) == 0
                voProjections_h(nCtr,1).XData = pix(1,1);
                voProjections_h(nCtr,1).YData = pix(2,1);
                
                voLines_h(nCtr,1).XData = [pix(1,1); mf2D_GT(j,1)];
                voLines_h(nCtr,1).YData = [pix(2,1); mf2D_GT(j,2)];
            end
        end
    end
    fErrorSum = fErrorSum/nPoints;
    % Save error
    con_err(i,1) = fErrorSum;
    
    if fErrorSum < fErrorSum_min
        fErrorSum_min = fErrorSum;
    end
    
    % Plot
    if mod(i, nPlotMonitor) == 0
        drawnow;
    end
end

% Normalize
% con_fx = con_fx ./ con_fx(1,1);
% con_u0 = con_u0 ./ con_u0(1,1);
% con_fy = con_fy ./ con_fy(1,1);
% con_v0 = con_v0 ./ con_v0(1,1);
% con_k1 = con_k1 ./ con_k1(1,1);
% con_k2 = con_k2 ./ con_k2(1,1);
% 
% con_cs_x = con_cs_x ./ con_cs_x(1,1);
% con_cs_y = con_cs_y ./ con_cs_y(1,1);
% con_cs_z = con_cs_z ./ con_cs_z(1,1);
% 
% con_rot_x = con_rot_x ./ con_rot_x(1,1);
% con_rot_y = con_rot_y ./ con_rot_y(1,1);
% con_rot_z = con_rot_z ./ con_rot_z(1,1);

con_fx = con_fx - repmat(con_fx(1,1), nNumIterations, 1);
con_u0 = con_u0 - repmat(con_u0(1,1), nNumIterations, 1);
con_fy = con_fy - repmat(con_fy(1,1), nNumIterations, 1);
con_v0 = con_v0 - repmat(con_v0(1,1), nNumIterations, 1);
con_k1 = con_k1 - repmat(con_k1(1,1), nNumIterations, 1);
con_k2 = con_k2 - repmat(con_k2(1,1), nNumIterations, 1);

con_cs_x = con_cs_x - repmat(con_cs_x(1,1), nNumIterations, 1);
con_cs_y = con_cs_y - repmat(con_cs_y(1,1), nNumIterations, 1);
con_cs_z = con_cs_z - repmat(con_cs_z(1,1), nNumIterations, 1);

con_rot_x = con_rot_x - repmat(con_rot_x(1,1), nNumIterations, 1);
con_rot_y = con_rot_y - repmat(con_rot_y(1,1), nNumIterations, 1);
con_rot_z = con_rot_z - repmat(con_rot_z(1,1), nNumIterations, 1);

% Create convergence plots
figure;
subplot(5,1,1);
plot(con_err);
title('Error');

subplot(5,1,2);
plot(con_fx); hold on;
plot(con_u0); hold on;
plot(con_fy); hold on;
plot(con_v0); hold on;
title('Intrinsics');
legend('fx', 'u0', 'fy', 'v0');

subplot(5,1,3)
plot(con_k1); hold on;
plot(con_k2); hold on;
title('Distortion');
legend('k1', 'k2');

subplot(5,1,4)
plot(con_cs_x); hold on;
plot(con_cs_y); hold on;
plot(con_cs_z); hold on;
title('Translation');
legend('cx', 'cy', 'cz');

subplot(5,1,5)
plot(con_rot_x); hold on;
plot(con_rot_y); hold on;
plot(con_rot_z); hold on;
title('Rotation');
legend('x', 'y', 'z');

fprintf('Error: %0.3f', fErrorSum_min);

%% Visualize coordinate system

vfX = [1; 0; 0];    % red
vfY = [0; 1; 0];    % green
vfZ = [0; 0; 1];    % blue

% Transform into camera coordinate system

% Original: C1
% Build rotation
R_z = [ 
    cos(rot_z)     -sin(rot_z)    0;
    sin(rot_z)     cos(rot_z)     0;
    0               0               1];

R_y = [ 
    cos(rot_y)     0       sin(rot_y);
    0               1       0;
    -sin(rot_y)    0       cos(rot_y)];

R_x = [ 
    1       0               0;
    0       cos(rot_x)     -sin(rot_x);
    0       sin(rot_x)     cos(rot_x)];

R = R_z * R_y * R_x;

% Translation
cs = [cs_x; cs_y; cs_z];

% Rotate
vfOrigin_C1     = cs;
vfX_C1          = R\vfX + cs;
vfY_C1          = R\vfY + cs;
vfZ_C1          = R\vfZ + cs;

% Optimized
% Build rotation
R_z = [ 
    cos(rot_z_best)     -sin(rot_z_best)    0;
    sin(rot_z_best)     cos(rot_z_best)     0;
    0              0              1];

R_y = [ 
    cos(rot_y_best)     0       sin(rot_y_best);
    0                   1       0;
    -sin(rot_y_best)    0       cos(rot_y_best)];

R_x = [ 
    1       0               0;
    0       cos(rot_x_best)     -sin(rot_x_best);
    0       sin(rot_x_best)     cos(rot_x_best)];

R = R_z * R_y * R_x;

% Translation
cs = [cs_x_best; cs_y_best; cs_z_best];

% Transform
vfOrigin_C2     = cs;
vfX_C2          = R\vfX + cs;
vfY_C2          = R\vfY + cs;
vfZ_C2          = R\vfZ + cs;

% Plot
figure
oAx1_h = subplot(1,2,1);

plot3([vfOrigin_C1(1,1); vfX_C1(1,1)], [vfOrigin_C1(2,1); vfX_C1(2,1)], [vfOrigin_C1(3,1); vfX_C1(3,1)], ...
    'r', 'LineWidth', 2); hold on;
plot3([vfOrigin_C1(1,1); vfY_C1(1,1)], [vfOrigin_C1(2,1); vfY_C1(2,1)], [vfOrigin_C1(3,1); vfY_C1(3,1)], ...
    'g', 'LineWidth', 2); hold on;
plot3([vfOrigin_C1(1,1); vfZ_C1(1,1)], [vfOrigin_C1(2,1); vfZ_C1(2,1)], [vfOrigin_C1(3,1); vfZ_C1(3,1)], ...
    'b', 'LineWidth', 2); hold on;

drawSRF(oAx1_h);

oAx2_h = subplot(1,2,2); 
plot3([vfOrigin_C2(1,1); vfX_C2(1,1)], [vfOrigin_C2(2,1); vfX_C2(2,1)], [vfOrigin_C2(3,1); vfX_C2(3,1)], ...
    'r', 'LineWidth', 2); hold on;
plot3([vfOrigin_C2(1,1); vfY_C2(1,1)], [vfOrigin_C2(2,1); vfY_C2(2,1)], [vfOrigin_C2(3,1); vfY_C2(3,1)], ...
    'g', 'LineWidth', 2); hold on;
plot3([vfOrigin_C2(1,1); vfZ_C2(1,1)], [vfOrigin_C2(2,1); vfZ_C2(2,1)], [vfOrigin_C2(3,1); vfZ_C2(3,1)], ...
    'b', 'LineWidth', 2); hold on;

drawSRF(oAx2_h);

%% Output parameters
K = [
      fx_best 0         u0_best;
      0       fy_best   v0_best;
      0       0         1];
  
dist = [k1_best, k2_best];

save('CalibrationResults.mat', 'K', 'R', 'cs', 'dist');
  
 

end

