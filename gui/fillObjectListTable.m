function [] = fillObjectListTable(oGUIData, oGUIObjects)
% ---------------------------------------------------------------------------------------------
% Function fillObjectListTable(...) fills the object list table.
%
% Examples:
% fillMovableDataVectorTable(gui, movableDataVec)
% fillMovableDataVectorTable(gui movableDataVec, indexToEdit) <- changes only specific entry
%
% INPUT:
%   oGUIData:       Object of class cGUIData containing handle information
%   oGUIObjects:    Object of class cGUIObjects for object information
%
% OUTPUT:
%   -
% ---------------------------------------------------------------------------------------------

voPCMovableLabel    = oGUIObjects.m_voPCMovableLabel;

nIndexCurrentObject = oGUIData.m_nIndexCurrentObject;
ObjectListTable     = oGUIData.m_oObjectListTable_h;
clData              = cell(size(voPCMovableLabel, 1),7);

for i = 1 : size(clData,1)
   if(i == nIndexCurrentObject)
       clData(i,1) = {'<html><table><td bgcolor="FF0000">'};
   else
       clData(i,1) = {'<html><table><td bgcolor="000000">'};
   end
   clData(i,2) = {sprintf('%d', i)};
   clData(i,3) = {voPCMovableLabel(i,1).m_sClassification};
   if(voPCMovableLabel(i,1).m_bIsActive)
       clData(i,4) = {true};
   else
       clData(i,4) = {false};
   end
   if(voPCMovableLabel(i,1).m_bIsCorrected)
       clData(i,5) = {'<html><table><td bgcolor="00FF00">'};
   else
       clData(i,5) = {'<html><table><td bgcolor="FF0000">'};
   end
   if(voPCMovableLabel(i,1).m_bIsNew)
       clData(i,6) = {'<html><table><td bgcolor="FF0000">'};
   else
       clData(i,6) = {'<html><table><td bgcolor="000000">'};
   end
   if(voPCMovableLabel(i,1).m_bIsPredicted)
       clData(i,7) = {'<html><table><td bgcolor="000000">'};
   else
       clData(i,7) = {'<html><table><td bgcolor="FF0000">'};
   end

end

ObjectListTable.Data = clData;

end
