function [clData] = fillMetadataTable(oPCMetadata)
% ---------------------------------------------------------------------------------------------
% Function fillMetadataTable(...) fills the according GUI table by passed PCMetadata
%
% INPUT:
%   oPCMetadata:    Object of class cPCMetadata containing the information to display
%
% OUTPUT:
%   clData:         Cell array of displayed data
% ---------------------------------------------------------------------------------------------

if nargin == 1
    fEgoAx = oPCMetadata.m_fEgoAx;	  
    fEgoAy = oPCMetadata.m_fEgoAy;  
    
    fEgoVx = oPCMetadata.m_fEgoVx;
    fEgoVy = oPCMetadata.m_fEgoVy;
    fEgoYawrate = oPCMetadata.m_fEgoYawRate;

    if  (oPCMetadata.m_bIsLastOfSequence)
        DT_succ = 0;
    else
        DT_succ = (oPCMetadata.m_nSuccessor_Timestamp_us - oPCMetadata.m_nTimestamp_us)/1000;
    end
else
    fEgoAx = 0;
    fEgoAy = 0;
    fEgoVx = 0;
    fEgoVy = 0;
    fEgoYawrate = 0;
    DT_succ = 0;
end

clData = {'Vx ego',   sprintf('%.2f m/s',   fEgoVx),                            'Vy ego',   sprintf('%.2f m/s',   fEgoVy); 
          'Ax ego',   sprintf('%.2f m/s%s', fEgoAx, char(hex2dec('B2'))),       'Ay ego',   sprintf('%.2f m/s%s', fEgoAy, char(hex2dec('B2')));
          'yaw rate', sprintf('%.3f %s/s',  fEgoYawrate, char(hex2dec('B0'))),  'DT succ.', sprintf('%.0f ms',    DT_succ)};
            
end

