function [setting] = settingByMenu(menuOption)

setting = 0;
if(strcmp(menuOption.Checked, 'on'))
    setting = 1;
end

end

