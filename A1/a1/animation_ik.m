close; clear;

InitialPosition = [0, pi/6, pi/8, 0, -pi/4, pi/4, 0, pi/6, pi/8, 0, pi/6, pi/8]';



Handler_State = SRD_get_handler__state('InitialPosition', InitialPosition, ...
    'InitialVelocity', zeros(size(InitialPosition)));

Handler_IK_Solution = SRD_get('Handler_IK_Solution');%
Handler_IK_Model = SRD_get('Handler_IK_Model');

% 
% dt = 0.01;
% tf = Handler_IK_Solution.TimeExpiration;
% % % tf = 0.021;

t0 = 0;
tf = 2;
dt = 0.1;
time_table = Handler_IK_Solution.State.TimeTable;

Handler_Simulation = SRD_get_handler__Simulation(...
    'TimeLog',time_table );

%%



DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', [], ...
    'FileName_visuals_config', 'datafile_visuals_config.mat'); %use default visuals

SRD__animate__position('Handler_Simulation', Handler_Simulation, ...
    'Handler_Logger_position', Handler_IK_Solution, ...
    'AnimationTimeLog', 0:10*dt:(tf-dt), ...
    'DrawRobot_function', DrawRobot_function, ...
    'NewFigure', true, ...
    'FigureName', 'Animation', ...
    'FileName_visuals_config', []);

Obs_pose  = [0.5;-0.1325;-0.164];   
IK_Table = Handler_IK_Solution.State.IK_Table;
task_traj = zeros(3,length(IK_Table));
for i=1:length(IK_Table)

task_traj(:,i) = Handler_IK_Model.get_Task(IK_Table(i,:)');
end

hold on
plot3(task_traj(1,:),task_traj(2,:),task_traj(3,:))
% plot3(Obs_pose(1),Obs_pose(2),Obs_pose(3),'.','MarkerSize', 30)
% 
