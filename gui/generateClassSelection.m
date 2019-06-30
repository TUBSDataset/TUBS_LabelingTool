function [oClassesButtonGroup_h] = generateClassSelection(oObjectPanel_h, oEditorConfig, classesGroup_Callback)
% ---------------------------------------------------------------------------------------------
% Function generateClassSelection(...) builds the class selection GUI according to the defined classes
% in the EditorConfig.xml file.
%
% INPUT:
%   oObjectPanel_h:         Handle to object panel
%   oEditorConfig:          Object of class cEditorConfig containing the class definition
%   classesGroup_Callback:  Definition group callback
%
% OUTPUT:
%   oClassesButtonGroup_h:  Generated class definition button group
% ---------------------------------------------------------------------------------------------

voClasses   = oEditorConfig.getAssignableClasses();
oClassUndef = oEditorConfig.getClassUndefined();

% Sort movable to the end
index = true(size(voClasses,1) ,1);
for i = 1 : size(voClasses,1)
    if strcmp(voClasses(i,1).Name, 'Movable')
        index(i,1) = false;
        oClassMov = voClasses(i,1);
    end
end

voClasses = voClasses(index);
voClasses = [voClasses; oClassMov; oClassUndef];

fClassesTopPos      = .785;
fClassesHeight      = .51;
fClassesBottom      = (fClassesTopPos - fClassesHeight);

fClassesLeftDist    = .03;
fStaticOffset       = -.002;
fVertOffset         = .02;
fHeightClass        = .05;
fVertDist           = .04;      % change fVertDist to reduce vertical distance between classes
fWidth              = .26;
fLeftOffset         = .2;

oClassesButtonGroup_h = uibuttongroup(oObjectPanel_h, 'SelectionChangedFcn', classesGroup_Callback, ...
    'Position', [0 fClassesBottom fWidth fClassesHeight], 'BorderType', 'line', 'SelectedObject', []);

for i = 1 : size(voClasses)
    sClassName = voClasses(i,1).Name;
    if i < 10
        sTag = sprintf('%d', i);
        sHotKey = sprintf('[%d]', i);
    else
        sHotKey = '[-]';
        sTag = '-';
    end
    
    if strcmp(sClassName, 'Undefined')
        sHotKey = '[-]';
        sTag = '-';
    end

    % Hot key text
    uicontrol(oClassesButtonGroup_h, 'Style', 'text', 'String', sHotKey, 'HorizontalAlignment', 'left', 'units', 'normalized', ...
        'Position', [fClassesLeftDist, 1-fHeightClass-fVertOffset-(i-1)*(fHeightClass+fVertDist), .2, fHeightClass]);
    
    % Radiobutton
    uicontrol(oClassesButtonGroup_h, 'Style', 'radiobutton', 'String', sClassName, 'units', 'normalized', 'Enable', 'off', ...
        'Position', [fClassesLeftDist+fLeftOffset, 1-fHeightClass-fVertOffset-fStaticOffset-(i-1)*(fHeightClass+fVertDist), ...
        .8, fHeightClass], 'Tag', sTag);
end

end

