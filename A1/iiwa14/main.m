clear ; close all;


InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');
IC_Task = Handler_IK_Model.get_Task(InitialPosition);
TimeTable = linspace(0,2,50);   

experiment_case = 3;

switch experiment_case
    case 2
        Goal_task = [IC_Task(1)-1;... %x
                     IC_Task(2);...  %y
                     IC_Task(3)];    %z

        Obs_pose  = [[0.3;IC_Task(2)+0.02;IC_Task(3)],...
                     [0;IC_Task(2)+0.01;IC_Task(3)]];
        Wieght=[2,0];
        Task_params = [Goal_task,Obs_pose];
        IK_solver = 'obstracle_avoidance' ;
        Cube = 0;
        
    case 3
        Goal_task = [IC_Task(1)-1;... %x
                     IC_Task(2);...  %y
                     IC_Task(3)];    %z

        Obs_pose  = [[0.3;IC_Task(2)+0.02;IC_Task(3)],...
                     [0;IC_Task(2)+0.01;IC_Task(3)]];
        Wieght=[2,1];
        Task_params = [Goal_task,Obs_pose];
        IK_solver = 'obstracle_avoidance' ;

        Cube = 0;           
        
    case 1
        Goal_task = [IC_Task(1)-1;... %x
                     IC_Task(2);...  %y
                     IC_Task(3)];    %z

        Task_params = [Goal_task,[]];
        IK_solver = 'line' ;
        Cube = 0;   
 
end

switch IK_solver 
    case 'line'
            Handler_IK_Solution = Get_IK_task_line('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'TimeTable',TimeTable,...
                                                  'Enable_tester',false);
            SRD_save(Handler_IK_Solution, 'IK_Solution_line')
            Handler_IK_Solution = SRD_get('IK_Solution_line');

            [time_table_ode,x_table] = Get_Table_Simulation_iiwa('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',false);

    case 'obstracle_avoidance'
            Handler_IK_Solution = Get_Ik_Solution('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'Obs_pose',Obs_pose,...
                                                  'TimeTable',TimeTable,...
                                                  'Wieght',Wieght,...
                                                  'Enable_tester',false);
                                              
            [time_table_ode,x_table] = Get_Table_Simulation_iiwa('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',false);
            SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
end 


% % 
Make_Animation_iiwa(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% 
Make_Animation_iiwa(time_table_ode,x_table(:,1:7),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')

% save_img ="true" ;
% if save_img =="true" 
% name = ["J","ik","x","e_x","e_u"];
% 
% for i=1:5
%     figure(i)
%     saveas(gcf, "/home/yongann/Pictures/exp3/"+name(i)+string(experiment_case)+".png")
% end
% 
% close all;
% draw_Animation_iiwa(time_table_ode,x_table(:,1:7),"fin",Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% % figure(1)
% saveas(gcf, "/home/yongann/Pictures/exp3/fin"+string(experiment_case)+".png")
% % 
% close all;
% draw_Animation_iiwa(time_table_ode,Handler_IK_Solution.State.IK_Table,"init",Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% % figure(2)
% saveas(gcf, "/home/yongann/Pictures/exp3/init"+string(experiment_case)+".png")
% % close all;
% 
% end

