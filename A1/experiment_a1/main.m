clear ; close all ;


InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');
IC_Task = Handler_IK_Model.get_Task(InitialPosition);
TimeTable = linspace(0,2,50);   

experiment_case = 2;

switch experiment_case
    case 3 
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
        
    case 4
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
        
    case 1
            Goal_task = [0.35;...
                         IC_Task(2);...
                         -0.15];
            IK_solver = 'line' ;
            Task_params = [Goal_task,[]]; 
            Cube = 0;
            
    case 2
            InitialPosition = [InitialPosition(1:4);-pi/3;pi/2;InitialPosition(7:12)];
            Goal_task = [0.5;...
                         IC_Task(2);...
                         0.23];%0.23
            IK_solver = 'line' ;
            Task_params = [Goal_task,[]]; 
            Cube = 0;
end


switch IK_solver 
    case 'line'
            Handler_IK_Solution = Get_IK_task_line('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'TimeTable',TimeTable,...
                                                  'Enable_tester',true);
            SRD_save(Handler_IK_Solution, 'IK_Solution_line')
            Handler_IK_Solution = SRD_get('IK_Solution_line');

            [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',true);

    case 'obstracle_avoidance'
            Handler_IK_Solution = Get_Ik_Solution('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'Obs_pose',Obs_pose,...
                                                  'TimeTable',TimeTable,...
                                                  'Wieght',Wieght,...
                                                  'Enable_tester',true);
                                              
            [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',true);
%             SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
end

% name = ["x","e_x","e_u"];
% % name = ["J","ik","x","e_x","e_u"];
% % 
% for i=1:3
%     figure(i)
%     saveas(gcf, "/home/yongann/Pictures/exp/"+name(i)+string(experiment_case)+".png")
% end


% Draw_robot(InitialPosition)
% Make_Animation(time_table_ode,x_table(1,1:12),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')


% Make_Animation(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')

% Make_Animation(time_table_ode,x_table(:,1:12),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% 
% close all;
% draw_Animation(time_table_ode,x_table(:,1:12),"fin",Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% saveas(gcf, "/home/yongann/Pictures/exp/fin"+string(experiment_case)+".png")
% % 
% close all;
% draw_Animation(time_table_ode,x_table(:,1:12),"init",Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')
% saveas(gcf, "/home/yongann/Pictures/exp/init"+string(experiment_case)+".png")
% close all;





