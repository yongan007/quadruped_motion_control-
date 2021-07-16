close all;

InitialPosition = SRD_get('InitialPosition');

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
    'DrawRobot_STL_FaceAlpha', 1, ...
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


figure('Color', 'w')


q1 = [   -1.4796
    -1.0512
    1.8088
    1.3890
    0.3158
    0.1883
    1.5589];
q2 = [   -0.0822
    0.2522
    0.5616
    0.2612
   -0.2445
    0.6001
   -0.9244];
q3 = [   -0.9243
   -1.3363
    1.2256
    0.3712
   -1.5790
   -1.1246
    1.0234];
q4 = [    -1.3243
    -1.8146
    0.9429
   1.4395
    0.9382
   -1.3071
   -1.0065];
q5 = [-0.6287
    0.7805
    0.2621
    1.1572
    0.4794
   -1.2424
    0.4302];
q6 = [   -1.2185
    0.1778
    0.4863
   -1.0543
    1.1619
   -0.6630
   -1.3247];

Q = [q1, q2, q3, q4, q5, q6];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:6
    subplot(2, 3, i);
    DrawRobot_function(Q(:, i), []);
    SRD__make_default_scene('STL');
end




