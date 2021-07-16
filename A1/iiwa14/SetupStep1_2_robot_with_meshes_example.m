close all; clear;
SRD = SRDuserinterface;

% LinkArray = UP_GetLinkArrayFromURDF('UrdfFilePath','./laika/urdf/laikago.urdf','ParseSTL',true);
% InitialPosition = zeros(1,21);
% 
% LinkArray = UP_GetLinkArrayFromURDF('UrdfFilePath','./iiwa/iiwa14.urdf','ParseSTL',true);
% InitialPosition = [ pi/2; pi/2; pi/2; pi/2; pi/2; pi/2; pi/2; pi/2; pi/2; 0; 0];
% 
% AxisLimits = []; %Set axis limits for the snapshot
% ViewAngle = [45, 45];
% DrawFrames = true;
% DrawMeshes = true;
% SRD.CreateRobotStructure(LinkArray, InitialPosition, AxisLimits, ViewAngle,DrawFrames,DrawMeshes); %Create the robot
% axis equal;

%Set generalised coordinates; Use negative numbers to make clock-wise
%rotations be positive
% Shin.SetUsedGenCoordinates(1);
% Hip.SetUsedGenCoordinates(2);
% Torso.SetUsedGenCoordinates(3);


LinkArray = SRD_GetLinkArrayFromURDF('UrdfFilePath','./iiwa/iiwa14.urdf','ParseSTL',true);

Color_0 = [1, 0, 0];
Color_1 = [0.95, 0.5, 0.3];
Color_2 = [0.00, 0.00, 0.00];
SRD_LinkSet_Color(LinkArray, 'iiwa_link_0', Color_0);
SRD_LinkSet_Color(LinkArray, 'iiwa_link_1', Color_1);
SRD_LinkSet_Color(LinkArray, 'iiwa_link_2', Color_2);
SRD_LinkSet_Color(LinkArray, 'iiwa_link_3', Color_1);
SRD_LinkSet_Color(LinkArray, 'iiwa_link_4', Color_2);
SRD_LinkSet_Color(LinkArray, 'iiwa_link_5', Color_1);
SRD_LinkSet_Color(LinkArray, 'iiwa_link_6', Color_2);
SRD_LinkSet_Color(LinkArray, 'iiwa_link_7', Color_1);

InitialPosition = [ pi/3; pi/2; pi/6; pi/3; 0; pi/3; pi/6];
% InitialPosition = [ pi/2; 0.2*pi/2; 0.2*pi/2; -1.0*pi/2; 1.2*pi/2; 1.2*pi/2; pi/2;];
% InitialPosition =zeros(7,1);

SRD_save(LinkArray, 'LinkArray');
SRD_save(InitialPosition, 'InitialPosition');

Chain = SRD_Chain(LinkArray);
SRD_save(Chain, 'Chain');
%Chain.Update(InitialPosition)


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
    'DrawRobot_STL_FaceLighting', 'gouraud', ...
    'DrawRobot_STL_AmbientStrength', 0.15, ...
    'DrawRobot_STL_camlight', 'headlight', ...
    'DrawRobot_STL_material', 'metal', ... %shiny dull metal
    'ToDrawFrames', false, ...
    'DrawRobot_Frame_Scale', 0.2, ...
    'DrawRobot_Frame_LineWidth', 1);

DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', Chain);

DrawRobot_function(InitialPosition, [])
SRD__make_default_scene('STL')
            