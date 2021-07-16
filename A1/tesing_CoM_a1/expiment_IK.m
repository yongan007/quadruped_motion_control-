clear ; close ;

tStart = tic;

num_experiments = 100;

min_range = 0.1;
max_range = 0.15;
r = (max_range-min_range).*rand(num_experiments,1) + min_range;

Handler_IK_Model = SRD_get('Handler_IK_Model');
Init_Position = SRD_get('InitialPosition');
IC_Task = Handler_IK_Model.get_Task(Init_Position);


TimeTable = linspace(0,2,50);

experiment_case = 1;

switch experiment_case
    case 1 
        Goal_task = [0.55;...
                     IC_Task(2);...
                     -0.04];

        Obs_pose  = [[0.50;IC_Task(2);-0.1],...
                     [0.393;IC_Task(2);-0.185],...
                     [0.39;IC_Task(2);-0.2],...
                     [0.38;IC_Task(2);-0.215],...
                     [0.37;IC_Task(2);-0.23]];
        Wieght=[3,0.3];
        Task_params = [Goal_task,Obs_pose];
        IK_solver = 'obstracle_avoidance' ;
        Cube_origin = [0.50 IC_Task(2) -0.3138];
        Cube_size = [0.50 0.3 (Goal_task(3)-0.02-Cube_origin(3))];
        Cube = [Cube_size;Cube_origin];        
        
    case 2
        Goal_task = [0.5;...
                     IC_Task(2);...
                     -0.2];
        Obs_pose  = [[0.33;IC_Task(2);-0.30],[0.35;IC_Task(2);-0.27],[0.4;IC_Task(2);-0.25]]; 
        Wieght=[3,0.1];
        IK_solver = 'obstracle_avoidance' ;
        Task_params = [Goal_task,Obs_pose]; 
        Cube_origin = [0.45 -0.16 -0.32];
        Cube_size = [0.3 0.3 (Goal_task(3)-0.02-Cube_origin(3))];
        Cube = [Cube_size;Cube_origin];
        
    case 3
            Goal_task = [0.35;...
                         IC_Task(2);...
                         -0.15];
            IK_solver = 'line' ;
            Task_params = [Goal_task,[]]; 
            Cube = 0;
            
    case 4
            Init_Position = [Init_Position(1:4);-pi/3;pi/2;Init_Position(7:12)];
            Goal_task = [0.5;...
                         IC_Task(2);...
                         0.23];
            IK_solver = 'line' ;
            Task_params = [Goal_task,[]]; 
            Cube = 0;
end

Init_Poses =zeros(length(Init_Position),num_experiments);

Max_Error_x = zeros(length(Init_Position),num_experiments);
Max_cost_u = zeros(length(Init_Position),num_experiments);
cost_qr = zeros(num_experiments,1);

for i=1:num_experiments
    InitialPosition = [Init_Position(1:4);Init_Position(5:6)+r(i);Init_Position(7:12)];
%     [Init_Position(1:10);Init_Position(11:12)+r(i)];%
    Init_Poses(:,i) = InitialPosition;
    
switch IK_solver 
    case 'line'
            Handler_IK_Solution = Get_IK_task_line('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'TimeTable',TimeTable,...
                                                  'Enable_tester',false);
            SRD_save(Handler_IK_Solution, 'IK_Solution_line')
            Handler_IK_Solution = SRD_get('IK_Solution_line');

    case 'obstracle_avoidance'
            Handler_IK_Solution = Get_Ik_Solution('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'Obs_pose',Obs_pose,...
                                                  'TimeTable',TimeTable,...
                                                  'Wieght',Wieght,...
                                                  'Enable_tester',false);
                                              
%             SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
end

[Max_Error_x(:,i),Max_cost_u(:,i),cost_qr(i)] = Get_Simulation_Cost('InitialPosition',InitialPosition,...
                                                    'Handler_IK_Solution',Handler_IK_Solution,... 
                                                     'Enable_tester',false);
disp("Completed: "+i)

end
tEnd = toc(tStart);

file_name = 'Costs_experiment_case'+string(experiment_case)+'.mat';
save(file_name,'IK_solver','num_experiments',...
    'Max_Error_x','Max_cost_u','cost_qr','tEnd');


% load('Costs_experiment_case3.mat')
std_Max_Error_x= std(Max_Error_x,0,2);
std_Max_cost_u = std(Max_cost_u,0,2);
std_cost_qr = std(cost_qr);

mean_Max_Error_x= mean(Max_Error_x,2);
mean_Max_cost_u = mean(Max_cost_u,2);
mean_cost_qr = mean(cost_qr);

disp("<strong>Trajectory</strong>: "+ IK_solver)
disp("<strong>Number of experiment:</strong> "+ num_experiments)

 T = table([std_Max_Error_x(4:6)';mean_Max_Error_x(4:6)'],...
     [std_Max_cost_u(4:6)';mean_Max_cost_u(4:6)'],...
     [std_cost_qr;mean_cost_qr],...
     'VariableNames',{'Max error x','Max cost u','cost QR'},'RowName',{'Std','Mean'}); 
disp(T)

disp("Execution time:"+tEnd+" seconds")

% 

% Draw_robot(Init_Poses)
              
% Cube_origin = [0.3 -0.2 -0.3138];
% Cube_size = [-0.45 0.5 -0.2];
% Cube = [Cube_size;Cube_origin];
% Task_params = [Goal_task,Obs_pose]; 
% 
% Make_Animation(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model)
% 
