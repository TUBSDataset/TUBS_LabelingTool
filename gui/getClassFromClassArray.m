function [oClass, nCode] = getClassFromClassArray(voClasses, sName)
% ---------------------------------------------------------------------------------------------
% Function getClassFromClassArray processes an input struct array of class structs and returns the demanded class.
%
% INPUT:
%   voClasses:  Struct array of classes. Included in cPCMovableLabel.m_voProbabilityVector.Class or cEditorConfig.m_voMovableClasses
%   sName:      Name of the class to be returned
% OUTPUT:
%   oClass:     Class struct    
%   nCode:      Error code. 1: Not found, 2: General error. Possibly due to wrong input.
% ---------------------------------------------------------------------------------------------
nCode = 1; oClass = [];
try
    for i = 1 : size(voClasses,1)
       if strcmp(voClasses(i,1).Name, sName)
           oClass = voClasses(i,1);
           nCode  = 0;
           return;
       end
    end
catch
    nCode = 2;
end
end

