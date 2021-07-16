close all;


Handler_IK_Solution = SRD_get('Handler_IK_Solution');
Handler_IK_task = SRD_get('Handler_IK_task');
Chain = SRD_get('Chain');

figure('Color', 'w')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Time = Handler_IK_Solution.TimeExpiration;
dt = 0.3;
Count = floor(Time / dt);

P = zeros(3, Count);

for i = 1:Count
    
%     q = Handler_IK_Solution.get_position(i*dt);
%     alpha = 0.8 * (Count - i)/Count + 0.05 * i/Count;

    index = Count - i;
    q = Handler_IK_Solution.get_position(index*dt);
    alpha = 0.05 * (Count - index)/Count + 1 * index/Count;
    
    
P(:, i) = Handler_IK_task.get_Task(index*dt);
    
SRD_SetupVisuals('AxisLimits', [-1; 1; -1; 1; -1; 1], ...
    'ViewAngle', [-37.5, 30], ...
    'ToDrawMeshes', true, ...
    'Animation_ToUseGrid', true, ...
    'Animation_ToUseGridMinor', true, ...
    'DrawRobot_Default_RobotColor', [0.6 0.3 0], ...
    'DrawRobot_Default_EdgeAlpha', 0.3, ...
    'DrawRobot_Default_FaceAlpha', 1, ...
    'DrawRobot_Default_LineWidth', 0.5, ...
    'DrawRobot_STL_FaceColor', [0.8 0.8 1.0], ...
    'DrawRobot_STL_EdgeColor', 'none', ...
    'DrawRobot_STL_FaceAlpha', alpha, ...
    'DrawRobot_STL_EdgeAlpha', 0, ...
    'DrawRobot_STL_FaceLighting', 'gouraud', ...
    'DrawRobot_STL_AmbientStrength', 0.15, ...
    'DrawRobot_STL_camlight', 'headlight', ...
    'DrawRobot_STL_material', 'metal', ... %shiny dull metal
    'ToDrawFrames', false, ...
    'DrawRobot_Frame_Scale', 0.2, ...
    'DrawRobot_Frame_LineWidth', 1, ...
    'FileName_visuals_config', 'datafile_visuals_config_custom.mat');


DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', Chain, ...
    'FileName_visuals_config', 'datafile_visuals_config_custom.mat');    
    
    
    DrawRobot_function(q, []); hold on;
end

plot3(P(1, :), P(2, :), P(3, :), 'LineWidth', 3);

SRD__make_default_scene('STL');






