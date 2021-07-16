
function draw_Animation(TimeTable,IK_Table,type_pose,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,DrawRobot_Type)

tf = TimeTable(end);
dt = 0.1;
% 
% Cube_origin = [0.3 -0.2 -0.3138];
% Cube_size = [-0.45 0.5 -0.2];

Handler_Simulation = SRD_get_handler__Simulation(...
    'TimeLog',TimeTable );
DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', DrawRobot_Type, ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', [], ...
    'FileName_visuals_config', 'datafile_visuals_config.mat'); %use default visuals

figure('Color', 'w')

if type_pose == "init"
DrawRobot_function(IK_Table(1,:)',[])
elseif type_pose == "fin"
    DrawRobot_function(IK_Table(end,:)',[])
end

Obs_pose = Task_params(:,2:end);

task_traj = zeros(Handler_IK_Model.dof_Task,length(IK_Table));
for i=1:length(IK_Table)
task = Handler_IK_Model.get_Task(IK_Table(i,:)');
task_traj(:,i)=task;
end

if Cube~=0 & size(Cube,1)==2
%     if size(Cube,2)==2
        plotcube(Cube(1,:),Cube(2,:),.8,[1 0 0]);
elseif Cube~=0 & size(Cube,1)==4
        plotcube(Cube(1,:),Cube(2,:),.8,[1 0 0]);
         plotcube(Cube(3,:),Cube(4,:),.8,[1 0 0]);

end

hold on
plot3(task_traj(1,:),task_traj(2,:),task_traj(3,:))
plot3(task_traj(4,:),task_traj(5,:),task_traj(6,:))
plot3(task_traj(7,:),task_traj(8,:),task_traj(9,:))
plot3(task_traj(10,:),task_traj(11,:),task_traj(12,:))

plot3(Obs_pose(1,:),Obs_pose(2,:),Obs_pose(3,:),'.','MarkerFaceColor','r',...
    'MarkerEdgeColor' ,'r','MarkerSize', 30)


visuals_config.ViewAngle =  [-10,20];
visuals_config.AxisLimits = [-0.5;0.75;-0.5;0.5;-0.4;0.5];
view(visuals_config.ViewAngle);
axis(visuals_config.AxisLimits);
grid minor;
grid on;
saveas(gcf,'Barchart.png')


end




