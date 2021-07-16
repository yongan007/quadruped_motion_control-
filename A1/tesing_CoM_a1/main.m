clear ; close all;


InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');
IC_Task = Handler_IK_Model.get_Task(InitialPosition);
TimeTable = linspace(0,2,50);   

experiment_case = 2 ;

switch experiment_case
    
    case 1
        TimeTable =0:0.1:2;   

        Goal_task = [IC_Task(1);IC_Task(2);IC_Task(3);...
                     IC_Task(4)+0.01;IC_Task(5);IC_Task(6)];
        IK_solver = 'line' ;
        Task_params = [Goal_task,[]]; 
%         Cube = 0;
            
    case 2
        Goal_task = [IC_Task(1);IC_Task(2);IC_Task(3);...
                     IC_Task(4)+0.01;IC_Task(5);IC_Task(6)];

        Obs_pose  = [-0.2;IC_Task(2);-0.3;0;0;0];
        Wieght=[3,0];
        Task_params = [Goal_task,Obs_pose];
        IK_solver = 'obstracle_avoidance' ;    
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
% 
%             [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
%                                                             'Handler_IK_Solution',Handler_IK_Solution,... 
%                                                              'Enable_tester',false);

    case 'obstracle_avoidance'
            Handler_IK_Solution = Get_Ik_Solution('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'Obs_pose',Obs_pose,...
                                                  'TimeTable',TimeTable,...
                                                  'Wieght',Wieght,...
                                                  'Enable_tester',false);
                                              
%             [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
%                                                             'Handler_IK_Solution',Handler_IK_Solution,... 
%                                                              'Enable_tester',true);
%             SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
end 

% Make_Animation(time_table_ode,x_table(:,1:12),Task_params,Cube,Handler_IK_Solution,Handler_IK_Model)

Cube_origin1 = [0.3 -0.2 -0.18];
Cube_size1 = [-0.17 0.4 0.18-0.35];

Cube_origin2 = [0.13 -0.2 -0.265];
Cube_size2 = [-0.13-0.2 0.4 -0.265+0.18];
Cube = [Cube_size1;Cube_origin1;Cube_size2;Cube_origin2]; 
Make_Animation(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model)

