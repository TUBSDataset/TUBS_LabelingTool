function [sDir] = buildPath(sBaseDir, nSequenceID, nPCID, nIdentifier)
% ---------------------------------------------------------------------------------------------
% Function buildPath(...) returns the path to a file specified by an identifier. 
%
% INPUT:
%   sBaseDir:    String containing the specified base directory
%   nSequenceID: Current sequenceID as int
%   nPCID:       Current PCID       as int
%   nIdentifier: See below
% OUTPUT:
%   sDir:   
% DETAILS:
%   nIdentifier = 1:    ImageData_Front
%   nIdentifier = 2:    ImageData_Left
%   nIdentifier = 3:    ImageData_Rear
%   nIdentifier = 4:    ImageData_Right
%
%   nIdentifier = 5:    ImageLabels_Front
%   nIdentifier = 6:    ImageLabels_Left
%   nIdentifier = 7:    ImageLabels_Rear
%   nIdentifier = 8:    ImageLabels_Right

%   nIdentifier = 9:    PCDataMatrices
%   nIdentifier = 10:   PCMetadata
%   nIdentifier = 11:   PCMovableLabels_Edited
%   nIdentifier = 12:   PCMovableLabels_Prelabeled
%   nIdentifier = 13:   PCMovableMatrices_Edited
%   nIdentifier = 14:   PCMovableMatrices_Prelabeled

%   nIdentifier = 15:   PCMovableLabels_Edited directory
%   nIdentifier = 16:   PCMovableLabels_Prelabeled directory
% --------------------------------------------------------------------------------------------- 

if      (nIdentifier == 1)
    sDir = strcat(sBaseDir, '\ImageData_Front\',                'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_Front.jpg');
elseif  (nIdentifier == 2)
    sDir = strcat(sBaseDir, '\ImageData_Left\',                 'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_Left.jpg');
elseif  (nIdentifier == 3)
    sDir = strcat(sBaseDir, '\ImageData_Rear\',                 'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_Rear.jpg');
elseif  (nIdentifier == 4)
    sDir = strcat(sBaseDir, '\ImageData_Right\',                'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_Right.jpg');
elseif  (nIdentifier == 5)
    sDir = strcat(sBaseDir, '\ImageLabels_Front\',              'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_ImageLabels_Front.xml');
elseif  (nIdentifier == 6)
    sDir = strcat(sBaseDir, '\ImageLabels_Left\',               'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_ImageLabels_Left.xml');
elseif  (nIdentifier == 7)
    sDir = strcat(sBaseDir, '\ImageLabels_Rear\',               'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_ImageLabels_Rear.xml');
elseif  (nIdentifier == 8)
    sDir = strcat(sBaseDir, '\ImageLabels_Right\',              'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_ImageLabels_Right.xml');
elseif  (nIdentifier == 9)
    sDir = strcat(sBaseDir, '\PCDataMatrices\',                 'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_PCDataMatrices.bin');
elseif  (nIdentifier == 10)
    sDir = strcat(sBaseDir, '\PCMetadata\',                     'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_PCMetadata.xml');
elseif  (nIdentifier == 11)
    sDir = strcat(sBaseDir, '\PCMovableLabels_Edited\',         'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_PCMovableLabels_Edited.xml');
elseif  (nIdentifier == 12)
    sDir = strcat(sBaseDir, '\PCMovableLabels_Prelabeled\',     'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_PCMovableLabels_Prelabeled.xml');
elseif  (nIdentifier == 13)
    sDir = strcat(sBaseDir, '\PCMovableMatrices_Edited\',       'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_PCMovableMatrices_Edited.bin');
elseif  (nIdentifier == 14)
    sDir = strcat(sBaseDir, '\PCMovableMatrices_Prelabeled\',   'Seq_', num2str(nSequenceID, '%010u'), '\', num2str(nPCID, '%010u'), '_PCMovableMatrices_Prelabeled.bin');
elseif  (nIdentifier == 15)
    sDir = strcat(sBaseDir, '\PCMovableLabels_Edited\',       'Seq_', num2str(nSequenceID, '%010u'));
elseif  (nIdentifier == 16)
    sDir = strcat(sBaseDir, '\PCMovableLabels_Prelabeled\',     'Seq_', num2str(nSequenceID, '%010u'));
end

end

