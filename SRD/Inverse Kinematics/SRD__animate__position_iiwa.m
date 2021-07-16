function SRD__animate__position_iiwa(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD__animate__vanilla';
Parser.addOptional('Handler_Simulation', []);
Parser.addOptional('Handler_Logger_position', []);
Parser.addOptional('AnimationTimeLog', []);
Parser.addOptional('Handler_IK_Model', []);
Parser.addOptional('IK_Table', []);
Parser.addOptional('Cube', []);
Parser.addOptional('Task_params', []);
Parser.addOptional('Type', 'Default'); %'Default', 'STL', 'Custom'
Parser.addOptional('DrawRobot_function', []);
Parser.addOptional('NewFigure', true);
Parser.addOptional('FigureName', 'Animation');
Parser.addOptional('FileName_visuals_config', []);
% Parser.addOptional('for_testing_Cotroller', false);


Parser.parse(varargin{:});

SRD = SRDinterface();

if isempty(Parser.Results.FileName_visuals_config)
    FileName_visuals_config = SRD.FileName_visuals_config;
else
    FileName_visuals_config = Parser.Results.FileName_visuals_config;
end
    
visuals_config = load(FileName_visuals_config);
visuals_config = visuals_config.visuals_config;


NewFigure = Parser.Results.NewFigure;
if NewFigure
    figure('Color', 'w', 'Name', Parser.Results.FigureName);
end

visuals_config.ViewAngle =  [-10,20];
visuals_config.AxisLimits = [-1;1;-1;1;-0.2;1.3];
view(visuals_config.ViewAngle);
axis(visuals_config.AxisLimits);

if visuals_config.Animation_ToUseGrid
    grid on;
end
if visuals_config.Animation_ToUseGridMinor
    grid minor;
end

switch Parser.Results.Type
    case 'Default'
    case 'STL'
        if ~isempty(visuals_config.DrawRobot_STL_camlight)
            camlight(visuals_config.DrawRobot_STL_camlight);
        end
        if ~isempty(visuals_config.DrawRobot_STL_material)
            material(visuals_config.DrawRobot_STL_material);
        end
end
h = [];

Obs_pose = Parser.Results.Task_params(:,2:end);
task_traj = zeros(3,length(Parser.Results.IK_Table));
for i=1:length(Parser.Results.IK_Table)
task = Parser.Results.Handler_IK_Model.get_Task(Parser.Results.IK_Table(i,:)');
task_traj(:,i)=task;
end

if Parser.Results.Cube~=0 & size(Parser.Results.Cube,1)==2
%     if size(Parser.Results.Cube,2)==2
        plotcube(Parser.Results.Cube(1,:),Parser.Results.Cube(2,:),.8,[1 0 0]);
elseif Parser.Results.Cube~=0 & size(Parser.Results.Cube,1)==4
        plotcube(Parser.Results.Cube(1,:),Parser.Results.Cube(2,:),.8,[1 0 0]);
         plotcube(Parser.Results.Cube(3,:),Parser.Results.Cube(4,:),.8,[1 0 0]);

end

% plotcube([ 0.3 0.3 (task_traj(3,end)+0.29)],[0.45 -0.16 -0.3138],.8,[1 0 0])
hold on
plot3(task_traj(1,:),task_traj(2,:),task_traj(3,:))
plot3(Obs_pose(1,:),Obs_pose(2,:),Obs_pose(3,:),'.','MarkerFaceColor','r',...
    'MarkerEdgeColor' ,'r','MarkerSize', 30)


for i = 1:length(Parser.Results.Handler_Simulation.TimeLog)
    
    target_time = Parser.Results.Handler_Simulation.TimeLog(i);
    
%     [~, index] = min(abs(Parser.Results.Handler_Simulation.TimeLog - target_time));
%     if Parser.Results.for_testing_Cotroller
%          q = Parser.Results.Handler_Logger_position.get_position(target_time)';
%     else     
         q = Parser.Results.IK_Table(i,:);
%     end
    
    [az, el] = view; %remember the user-set view
    
    h = Parser.Results.DrawRobot_function(q, h); %draw new frame
    view([az, el]); %restore the user-set view
    axis equal;
    xlabel("x");
    axis(visuals_config.AxisLimits);
 
    drawnow;
    if (i + 1) < length(Parser.Results.AnimationTimeLog)
        pause(Parser.Results.AnimationTimeLog(i+1) - Parser.Results.AnimationTimeLog(i));
    end
end


end