
function Draw_robot(Init_Poses)
Chain = SRD_get('Chain');
DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', Chain);
SRD__make_default_scene('STL')

for q = Init_Poses
    DrawRobot_function(q, []);
end
end
