
q0 = Handler_IK_Model.get_Task(InitialPosition);
Handler_IK_Model.get_Task(q0)
Chain = SRD_get('Chain');

DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', Chain);
 DrawRobot_function(q0, []);
% P = [];
% for q = IK_Table'
%     task = Handler_IK_Model.get_Task(q);
% %     P = [P,task];
% 
%     DrawRobot_function(q, []);
%  
% end
SRD__make_default_scene('STL')
