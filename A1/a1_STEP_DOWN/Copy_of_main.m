clear ; close ;


InitialPosition = SRD_get('InitialPosition');


Handler_IK_Model = SRD_get('Handler_IK_Model');

IC_Task = Handler_IK_Model.get_Task(InitialPosition);

Goal_task = [IC_Task(1)+0.01;...
             IC_Task(2);...
             IC_Task(3)];
 
Obs_pose  = [-0.15;IC_Task(2);-0.3138];
TimeTable = linspace(0,2,50);

IK_solver = 'obstracle_avoidance' ;


switch IK_solver 
    case 'line'
            Handler_IK_Solution = Get_IK_task_line('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'TimeTable',TimeTable,...
                                                  'Enable_tester',false);
            SRD_save(Handler_IK_Solution, 'IK_Solution_line')
            Handler_IK_Solution = SRD_get('IK_Solution_line');

            [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',false);

    case 'obstracle_avoidance'
            Handler_IK_Solution = Get_Ik_Solution('Handler_IK_Model',Handler_IK_Model,...
                                                  'InitialPosition',InitialPosition,...
                                                  'Goal_task',Goal_task,...
                                                  'Obs_pose',Obs_pose,...
                                                  'TimeTable',TimeTable,...
                                                  'Enable_tester',false);
                                              
            [time_table_ode,x_table] = Get_Table_Simulation('InitialPosition',InitialPosition,...
                                                            'Handler_IK_Solution',Handler_IK_Solution,... 
                                                             'Enable_tester',false);
%             SRD_save(Handler_IK_Solution, 'Handler_IK_Solution_obs')
end 

                 
              
Cube_origin = [0.3 -0.2 -0.3138];
Cube_size = [-0.45 0.5 -0.2];
Cube = [Cube_size;Cube_origin];
Task_params = [Goal_task,Obs_pose]; 

Make_Animation(TimeTable,Handler_IK_Solution.State.IK_Table,Task_params,Cube,Handler_IK_Solution,Handler_IK_Model,'STL')

