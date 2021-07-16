clear ; close all;


InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');
IC_Task = Handler_IK_Model.get_Task(InitialPosition);
TimeTable = linspace(0,2,50);   

experiment_case = 3;

switch experiment_case
    
    case 1
%         TimeTable =0:0.1:2;   

        Goal_task = [IC_Task(1); IC_Task(2); IC_Task(3);...
                     IC_Task(4); IC_Task(5); IC_Task(6);...
                     IC_Task(7); IC_Task(8); IC_Task(9);...
                     IC_Task(10); IC_Task(11); IC_Task(12);...
                     IC_Task(13); IC_Task(14);  IC_Task(15)+0.05];
        IK_solver = 'line' ;
        Task_params = [Goal_task,[]]; 
%         Cube = 0;
          
case 2

    Goal_task = [IC_Task(1); IC_Task(2); IC_Task(3);...
                 IC_Task(4); IC_Task(5); IC_Task(6);...
                 IC_Task(7); IC_Task(8); IC_Task(9);...
                 IC_Task(10)-0.3; IC_Task(11); IC_Task(12)-0.1;...
                 IC_Task(13); IC_Task(14);  IC_Task(15)];
    IK_solver = 'line' ;
    Task_params = [Goal_task,[]]; 
%         Cube = 0;
case 3

        Goal_task = [IC_Task(1); IC_Task(2); IC_Task(3);...  %leg1
                     IC_Task(4); IC_Task(5); IC_Task(6);...  %leg2
                     IC_Task(7); IC_Task(8); IC_Task(9);...
                     -0.3; IC_Task(11); -0.35;... %leg left
                     IC_Task(13); IC_Task(14);  IC_Task(15)];
                 

        Obs_pose = [Goal_task(1:9);-0.2;IC_Task(11);-0.28;Goal_task(13:15)];
        Wieght=[2.5,0.5];
        Task_params = [Goal_task,[-0.2;IC_Task(11);-0.28;zeros(12,1)]];
        IK_solver = 'obstracle_avoidance' ; 
   
case 4

        Goal_task = [IC_Task(1); IC_Task(2); IC_Task(3);...  %leg1
                     IC_Task(4); IC_Task(5); IC_Task(6);...  %leg2
                     IC_Task(7); IC_Task(8); IC_Task(9);...
                     -0.3; IC_Task(11); -0.35;... %leg left
                     IC_Task(13); IC_Task(14);  IC_Task(15)];
                 

        Obs_pose = [Goal_task(1:9);-0.2;IC_Task(11);-0.28;Goal_task(13:15)];
        Wieght=[2.5,0.5];
        Task_params = [Goal_task,[-0.2;IC_Task(11);-0.28;zeros(12,1)]];
        IK_solver = 'obstracle_avoidance' ;   
end


switch IK_solver 
    case 'line'
            Handler_IK_Solution = Get_IK_task_line_flaoting('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'TimeTable',TimeTable,...
                                                  'Enable_tester',false);
            SRD_save(Handler_IK_Solution, 'IK_Solution_line')
            Handler_IK_Solution = SRD_get('IK_Solution_line');

            [time_table_ode,x_table] = Get_Table_Simulation_flaoting('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',false);

    case 'obstracle_avoidance'
            Handler_IK_Solution = Get_Ik_Solution_floating('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'Obs_pose',Obs_pose,...
                                                  'TimeTable',TimeTable,...
                                                  'Wieght',Wieght,...
                                                  'Enable_tester',false);
                                              
            [time_table_ode,x_table] = Get_Table_Simulation_flaoting('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',false);
%             SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
end 

% Make_Animation(time_table_ode,x_table(:,1:12),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model)



Cube_origin1 = [0.3 -0.2 -0.18];
Cube_size1 = [-0.17 0.4 0.18-0.35];

Cube_origin2 = [0.13 -0.2 -0.265];
Cube_size2 = [-0.13-0.2 0.4 -0.265+0.18];
Cube = [Cube_size1;Cube_origin1;Cube_size2;Cube_origin2]; 

Make_Animation_floating(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
Make_Animation_floating(time_table_ode,x_table(:,1:18),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')


% name = ["x","e_x","e_u"];
% 
% for i=1:3
%     figure(i)
%     saveas(gcf, "/home/yongann/Pictures/exp1/"+name(i)+string(experiment_case)+".png")
% end
% 

% Draw_robot(InitialPosition)
% Make_Animation(time_table_ode,x_table(1,1:12),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')


% Make_Animation(time_table_ode,x_table(:,1:12),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')

% close all;
% draw_Animation(time_table_ode,x_table(:,1:18),"fin",Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% saveas(gcf, "/home/yongann/Pictures/exp1/fin"+string(experiment_case)+".png")
% % 
% close all;
% draw_Animation(time_table_ode,x_table(:,1:18),"init",Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% saveas(gcf, "/home/yongann/Pictures/exp1/init"+string(experiment_case)+".png")
% close all;












