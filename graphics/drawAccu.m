function [guiBoxes] = drawAccu(PC_h, guiBoxes, gui)

run COLOR_CONSTANTS.m;
for i = 1 : size(guiBoxes.m_RegisteredPointMatrices,1)
    points = guiBoxes.m_RegisteredPointMatrices{i,1};
    if(~isempty(points))
        switch guiBoxes.m_MovableData(i,1).m_Classification
            case 'Car'
                color = CAR_COLOR;
            case 'Truck'
                color = TRUCK_COLOR;
            case 'Van'
                color = VAN_COLOR;
            case 'Pedestrian'
                color = PEDESTRIAN_COLOR;
            case 'Bicycle'
                color = BICYCLE_COLOR;
            case 'Motorbike'
                color = MOTORBIKE_COLOR;
            case 'Movable'
                color = MOVABLE_COLOR;
            case 'Undefined'
                color = [1 0 0];
            otherwise
                color = [1 0 0];
        end
        if(getappdata(gui, 'showAccu') && getappdata(gui, 'enableICP')) % ICP must be enabled, otherwise old registered point matrix is shown (outdated cause not registered)
            guiBoxes.m_Accu_h(i,1) = scatter3(points(:,1), points(:,2), points(:,3), 20, color, '.', 'Visible', 'on', 'Parent', PC_h);
        else
            guiBoxes.m_Accu_h(i,1) = scatter3(points(:,1), points(:,2), points(:,3), 20, color, '.', 'Visible', 'off','Parent', PC_h);
        end
    end
end

end

