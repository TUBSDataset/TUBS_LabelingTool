function [voCalibration, nCode] = readCalibrationData(sConfigDir)
% ---------------------------------------------------------------------------------------------
% Function readCalibrationData(...) reads the intrinsic and extrinsic camera parameters. 
%
% INPUT:
%   sConfigDir:     Directory containing CalibrationParameters.txt
%
% OUTPUT:
%   voCalibration:  Parsed calibration data
%   nCode:          Error code
% ---------------------------------------------------------------------------------------------

nCode = 0;
voCalibration(4,1) = cCalibration();

sConfigDir = strcat(sConfigDir, '\CalibrationParameters.txt');
nFileID  = fopen(sConfigDir);
if nFileID == -1
    nCode = 1;
    return
end

% Parse text file
sLine = fgetl(nFileID);
nCameraCtr  = 0;
while ischar(sLine)
    sLine = fgetl(nFileID);
    if strfind(sLine, '######')
        nCameraCtr = nCameraCtr + 1;
        sType = '';
        switch nCameraCtr
            case 1
                sType = 'Front';
            case 2
                sType = 'Right';
            case 3
                sType = 'Rear';
            case 4
                sType = 'Left';
        end
        voCalibration(nCameraCtr,1).m_sType = sType;      
    end
    if strcmp(sLine, 'Intrinsics')
        sLine = fgetl(nFileID);
        sLine = sLine(1, 6:end);
        voCalibration(nCameraCtr,1).m_mfK = parseLineIntoMatrix(sLine);
    end
    if strcmp(sLine, 'Radial Distortion')
        sLine = fgetl(nFileID);
        sLine = sLine(1, 6:end);
        voCalibration(nCameraCtr,1).m_vfd = parseLineIntoMatrix(sLine);
    end
    if strcmp(sLine, 'Position in Velodyne Reference Frame')
        sLine = fgetl(nFileID);
        sLine = sLine(1, 6:end);
        voCalibration(nCameraCtr,1).m_vfc = parseLineIntoMatrix(sLine);
    end
    if strcmp(sLine, 'Rotation into Camera Reference Frame')
        sLine = fgetl(nFileID);
        sLine = sLine(1, 6:end);
        voCalibration(nCameraCtr,1).m_mfR = parseLineIntoMatrix(sLine);
    end
end

fclose(nFileID);
    function mfValues = parseLineIntoMatrix(sLine)
        nDigitCtr = 0;
        sNum = repmat(' ', 1, 1000);
        mfValues    = zeros(8,8);
        nColumnCtr  = 1;
        nRowCtr     = 1;
        for i = 1 : size(sLine,2)
            nDigitCtr = nDigitCtr + 1;
            sNum(1,nDigitCtr) = sLine(1,i);
            if      strcmp(sLine(1,i), ' ')
                mfValues(nRowCtr, nColumnCtr) = str2double(sNum);
                nColumnCtr  = nColumnCtr + 1;
                sNum        = repmat(' ', 1, 1000);
                nDigitCtr   = 0;
            elseif  strcmp(sLine(1,i), ';')
                mfValues(nRowCtr, nColumnCtr) = str2double(sNum(1,1:nDigitCtr-1));
                nRowCtr     = nRowCtr + 1;
                nColumnCtr  = 1;
                sNum        = repmat(' ', 1, 1000);
                nDigitCtr   = 0;
            elseif  strcmp(sLine(1,i), ']')
                mfValues(nRowCtr, nColumnCtr) = str2double(sNum(1,1:nDigitCtr-1));
            end
        end
        mfValues = mfValues(1:nRowCtr, 1:nColumnCtr);
    end
end

