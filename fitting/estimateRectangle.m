function [mfEdgePoints, bSuccess] = estimateRectangle(mfEdgePoints)
% ---------------------------------------------------------------------------------------------
% Function estimateRectangle(...) rectifies a set of given edge points to an perpendicular rectangle. 
%
% INPUT:
%   mfEdgePoints:   Matrix of user defined edge points.
%   
%
% OUTPUT:
%   mfEdgePoints:   Rectified edge points.
%   bSuccess:       Boolean, false if no success.
% ---------------------------------------------------------------------------------------------
        p1 = mfEdgePoints(1,1:2)';
        p2 = mfEdgePoints(2,1:2)';
        p3 = mfEdgePoints(3,1:2)';
        p4 = mfEdgePoints(4,1:2)';
        
        %% Sorting
        distList = cell(4,2);
        distList(1,1) = {'dist12'}; distList(1,2) = {norm(p2-p1)}; distList(1,3) = {p1}; distList(1,4) = {p2};
        distList(2,1) = {'dist23'}; distList(2,2) = {norm(p3-p2)+0.0001}; distList(2,3) = {p2}; distList(2,4) = {p3};
        distList(3,1) = {'dist34'}; distList(3,2) = {norm(p4-p3)}; distList(3,3) = {p3}; distList(3,4) = {p4};
        distList(4,1) = {'dist41'}; distList(4,2) = {norm(p1-p4)+0.0002}; distList(4,3) = {p4}; distList(4,4) = {p1};
        distList = sortrows(distList,-2);
        
        % Renumbering
        p1 = distList{1,4}; p4 = distList{1,3};
        listOfUsedPoints(1,:) = distList{1,4}; listOfUsedPoints(2,:) = distList{1,3};
        
        % Check dist list
        secondOk = 1;
        for i = 1 : length(listOfUsedPoints)
            if(isequal(listOfUsedPoints(i,:),distList{2,3}') || isequal(listOfUsedPoints(i,:),distList{2,4}'))
                secondOk = 0;
            end
        end
        if(secondOk)
            p2 = distList{2,3}; p3 = distList{2,4};
        else
            thirdOk = 1;
            for i = 1 : length(listOfUsedPoints)
                if(isequal(listOfUsedPoints(i,:),distList{3,3}') || isequal(listOfUsedPoints(i,:),distList{3,4}'))
                    thirdOk = 0;
                end
            end
            
            if(thirdOk)
                p2 = distList{3,3}; p3 = distList{3,4};
            else
                fourthOk = 1;
                for i = 1 : length(listOfUsedPoints)
                    if(isequal(listOfUsedPoints(i,:),distList{4,3}') || isequal(listOfUsedPoints(i,:),distList{4,4}'))
                        fourthOk = 0;
                    end
                end
                
                if(fourthOk)
                    p2 = distList{4,3}; p3 = distList{4,4};
                else
                    bSuccess = -1;
                    return;
                end
            end
        end
        
        %% Normal shooting p1 to g23
        p14 = p4 - p1;
        n1x = 0; n1y = 0;
        if(p14(1,1) == 0)
            n1x = 1;
        elseif(p14(2,1) == 0)
            n1y = 1;
        else
            n1x = 1;
            n1y = (-p14(1,1)*n1x)/(p14(2,1)+eps);
        end
        n1 = [n1x; n1y];
        
        % Intersection with g23
        % g23 = p2 + u*p23;
        % n12 = p1 + l*n1;
        p23 = p3-p2;
        A = [p23(1,1) -n1(1,1);
             p23(2,1) -n1(2,1)];
        b = [p1(1,1)-p2(1,1); p1(2,1)-p2(2,1)];
        X = A\b;
        sp23 = p1 + X(2,1)*n1;
        n1_norm = norm(sp23 - p1);
        
        % Correct p2
        vp2sp23 = sp23 - p2;
        p2n = p2 + vp2sp23/2;
        
        %% Normal shooting p2 to g14
        n2x = 0; n2y = 0;
        if(p23(1,1) == 0)
            n2x = 1;
        elseif(p23(2,1) == 0)
            n2y = 1;
        else
            n2x = 1;
            n2y = (-p23(1,1)*n2x)/(p23(2,1)+eps);
        end
        n2 = [n2x; n2y];
        
        % Intersection with g14
        % g14 = p1 + u*p14;
        % n21 = p2 + l*n2;
        A = [p14(1,1) -n2(1,1);
             p14(2,1) -n2(2,1)];
        b = [p2(1,1)-p1(1,1); p2(2,1)-p1(2,1)];
        X = A\b;
        sp14 = p2 + X(2,1)*n2;
        n2_norm = norm(sp14 - p2);
        % correct p1
        vp1sp14 = sp14 - p1;
        p1n = p1 + vp1sp14/2;
        
        %% Normal shooting p4 to g23
        n4 = n1;
        % intersection with g23
        % g23 = p2 + u*p23;
        % n43 = p4 + l*n4;
        A = [p23(1,1) -n4(1,1);
             p23(2,1) -n4(2,1)];
        b = [p4(1,1)-p2(1,1); p4(2,1)-p2(2,1)];
        X = A\b;
        sp23 = p4 + X(2,1)*n4;
        n4_norm = norm(sp23 - p4);
        % Correct p3
        vp3sp23 = sp23 - p3;
        p3n = p3 + vp3sp23/2;
        
        %% Normal shooting p3 to g14
        n3 = n2;
        % intersection with g14
        % g14 = p1 + u*p14;
        % n31 = p3 + l*n3;
        A = [p14(1,1) -n3(1,1);
             p14(2,1) -n3(2,1)];
        b = [p3(1,1)-p1(1,1); p3(2,1)-p1(2,1)];
        X = A\b;
        sp14 = p3 + X(2,1)*n3;
        n3_norm = norm(sp14 - p3);
        % correct p4
        vp4sp14 = sp14 - p4;
        p4n = p4 + vp4sp14/2;
        
        %% Correct distance
        n_norm = (n1_norm + n2_norm + n3_norm + n4_norm)/4;
        % correct p1 and p2
        p12n = p2n - p1n;
        p12n_norm = norm(p12n);
        p12n_dir = p12n ./ p12n_norm;
        p1n = p1n - p12n_dir*(n_norm - p12n_norm)/2;
        p2n = p2n + p12n_dir*(n_norm - p12n_norm)/2;
        % correct p4 and p3
        p43n = p3n - p4n;
        p43n_norm = norm(p43n);
        p43n_dir = p43n ./ p43n_norm;
        p4n = p4n - p43n_dir*(n_norm - p43n_norm)/2;
        p3n = p3n + p43n_dir*(n_norm - p43n_norm)/2;
        
        %% Correct angle
        %  determine intersection line
        i1 = p1n + (p2n - p1n)/2;
        i2 = p4n + (p3n - p4n)/2;
        i21 = i1 - i2;
        p12n = p2n - p1n;
        p43n = p3n - p4n;
        % rotate p1 and p2
        alpha12n_sharp = acos(abs(dot(p12n,i21))/(norm(p12n) * norm(i21)));
        phi = pi()/2 - alpha12n_sharp;
        R12 = [cos(phi) -sin(phi); sin(phi) cos(phi)];
        % determine rotation direction;
        p2n_temp = i1 + R12*(p2n-i1);
        p1n_temp = i1 + R12*(p1n-i1);
        p12_temp = p2n_temp - p1n_temp;
        alpha12n_temp = acos(abs(dot(p12_temp,i21))/(norm(p12_temp) * norm(i21)));
        phi_temp = abs(pi()/2 - alpha12n_temp);
        if(phi_temp < phi)
            p1n = p1n_temp;
            p2n = p2n_temp;
        else
            R12 = [cos(-phi) -sin(-phi); sin(-phi) cos(-phi)];
            p2n = i1 + R12*(p2n-i1);
            p1n = i1 + R12*(p1n-i1);
        end
        % rotate p3 and p4
        alpha43n_sharp = acos(abs(dot(p43n,i21))/(norm(p43n) * norm(i21)));
        phi = pi()/2 - alpha43n_sharp;
        R43 = [cos(phi) -sin(phi); sin(phi) cos(phi)];
        % determine rotation direction;
        p3n_temp = i2 + R43*(p3n-i2);
        p4n_temp = i2 + R43*(p4n-i2);
        p43_temp = p3n_temp - p4n_temp;
        alpha43n_temp = acos(abs(dot(p43_temp,i21))/(norm(p43_temp) * norm(i21)));
        phi_temp = abs(pi()/2 - alpha43n_temp);
        if(phi_temp < phi)
            p3n = p3n_temp;
            p4n = p4n_temp;
        else
            R43 = [cos(-phi) -sin(-phi); sin(-phi) cos(-phi)];
            p3n = i2 + R43*(p3n-i2);
            p4n = i2 + R43*(p4n-i2);
        end
        p12n = p2n - p1n;
        p43n = p4n - p3n;
        alpha12n_sharp = acos(abs(dot(p12n,i21))/(norm(p12n) * norm(i21)))*180/pi(); % control value, must be 90°
        alpha43n_sharp = acos(abs(dot(p43n,i21))/(norm(p43n) * norm(i21)))*180/pi();
        
        % test result
        if(((alpha12n_sharp - 90) > 0.001) || ((alpha43n_sharp - 90) > 0.001))
            bSuccess = -2;
            return;
        end
        
        p14n = p4n - p1n;
        p23n = p3n - p2n; % "normal"
        res = p14n./p23n;
        if(abs((res(2,1) - res(1,1))) > 0.001)
            bSuccess = -3; % longest sides are intersecting
           return;
        end
        
        bSuccess = 1;
        mfEdgePoints(1,1:2) = p1n';
        mfEdgePoints(2,1:2) = p2n';
        mfEdgePoints(3,1:2) = p3n';
        mfEdgePoints(4,1:2) = p4n';
end

