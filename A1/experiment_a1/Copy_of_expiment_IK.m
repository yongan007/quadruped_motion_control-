clear ; close ;

num_experiments = 2;

min_range = 0.1;
max_range = 0.15;
r = (max_range-min_range).*rand(num_experiments,1) + min_range;

Handler_IK_Model = SRD_get('Handler_IK_Model');
Init_Position = SRD_get('InitialPosition');
IC_Task = Handler_IK_Model.get_Task(Init_Position);
Goal_task = [-0.2;...
             IC_Task(2);...
             -0.35];
Obs_pose  = [-0.15;IC_Task(2);-0.3138];

Init_Poses =zeros(length(Init_Position),num_experiments);

TimeTable = linspace(0,2,50);

Max_Error_x = zeros(length(Init_Position),num_experiments);
Max_cost_u = zeros(length(Init_Position),num_experiments);
cost_qr = zeros(num_experiments,1);

for i=1:num_experiments
    InitialPosition = [Init_Position(1:10);Init_Position(11:12)+r(i)];%
%     Init_Poses(:,i) = InitialPosition;
    trajectory = 'obstracle_avoidance' ;
    
switch trajectory 
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
                                                  'Enable_tester',false);                                                                    
%             SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
    otherwise
end 

[Max_Error_x(:,i),Max_cost_u(:,i),cost_qr(i)] = Get_Simulation_Cost('InitialPosition',InitialPosition,...
                                                    'Handler_IK_Solution',Handler_IK_Solution,... 
                                                     'Enable_tester',false);
disp("Completed: "+i)

end


save('Cost_experiment_result.mat','Max_Error_x','Max_cost_u','cost_qr');

std_Max_Error_x= std(Max_Error_x,0,2);
std_Max_cost_u = std(Max_cost_u,0,2);
std_cost_qr = std(cost_qr);

mean_Max_Error_x= mean(Max_Error_x,2);
mean_Max_cost_u = mean(Max_cost_u,2);
mean_cost_qr = mean(cost_qr);

disp("<strong>Trajectory</strong>: "+ trajectory)
disp("<strong>Numeer of experiment QR:</strong> "+ num_experiments)

 T = table([std_Max_Error_x(10:12)';mean_Max_Error_x(10:12)'],...
     [std_Max_cost_u(10:12)';mean_Max_cost_u(10:12)'],...
     [std_cost_qr;mean_cost_qr],...
     'VariableNames',{'Max error x','Max cost u','cost QR'},'RowName',{'Std','Mean'}); 
disp(T) 

 
% 

% Draw_robot(Init_Poses)
              
% Cube_origin = [0.3 -0.2 -0.3138];
% Cube_size = [-0.45 0.5 -0.2];
% Cube = [Cube_size;Cube_origin];
% Task_params = [Goal_task,Obs_pose]; 
% 
% Make_Animation(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model)
% 
