clear; close all;

%Get initial position
InitialPosition = SRD_get('InitialPosition');
Handler_IK_Model = SRD_get('Handler_IK_Model');

% gaols_task = [0, 0, pi/4, 0, 0, pi/2, 0, 0, pi/2, 0, 0, pi/2]';


IC_Task = Handler_IK_Model.get_Task(InitialPosition);

%exp1
ZeroOrderDerivativeNodes = {IC_Task(1), 0.35;
                            IC_Task(2), IC_Task(2);  
                            IC_Task(3), -0.15}; 
%exp2
% ZeroOrderDerivativeNodes = {IC_Task(1), 0.6;
%                             IC_Task(2), IC_Task(2);  
%                             IC_Task(3), 0.3};  
%                             IC_Task(4), IC_Task(4);  
%                             IC_Task(5), IC_Task(5);  %IC_Task(5)-IC_Task(5)*2
%                             IC_Task(6), IC_Task(6)}; %IC_Task(6)+IC_Task(5)*2};
%                         
                        
FirstOrderDerivativeNodes = {0, 0; 
                             0, 0;  
                             0, 0;  
                             0, 0;  
                             0, 0;  
                             0, 0}; 
SecondOrderDerivativeNodes = {0, 0; 
                              0, 0; 
                              0, 0;  
                              0, 0;  
                              0, 0;  
                              0, 0}; 

TimeOfOneStage = 2;
TimeEnd = (size(ZeroOrderDerivativeNodes, 2) - 1)*TimeOfOneStage;
NodeTimes = (0:TimeOfOneStage:TimeEnd)';

Handler_IK_task = SRD_get_handler__IK_task__splines('NodeTimes', NodeTimes, ...
    'ZeroOrderDerivativeNodes', ZeroOrderDerivativeNodes, ...
    'FirstOrderDerivativeNodes', FirstOrderDerivativeNodes, ...
    'SecondOrderDerivativeNodes', SecondOrderDerivativeNodes, ...
    'OutOfBoundariesBehaviour', 'LastValue');

SRD_save(Handler_IK_task, 'Handler_IK_task');

% check resulting task
% Handler_IK_task.State.Spline.Plot;

%%%%%%%%

TimeTable = Handler_IK_task.TimeStart:0.1:Handler_IK_task.TimeExpiration;

IK_Table = SRD_InverseKinematics_GenerateTable(...
    'Handler_IK_Model', Handler_IK_Model, ...
    'Handler_IK_task', Handler_IK_task, ...
    'InitialGuess', InitialPosition, ...
    'method', @SRD_InversePositionProblemSolver_lsqnonlin, ...
    'TimeTable', TimeTable);

SRD_InverseKinematics_GenerateTable_tester(...
    'Handler_IK_Model', Handler_IK_Model, ...
    'TimeTable', TimeTable, ...
    'IK_Table', IK_Table);

Handler_IK_Solution = SRD_get_handler__IK_solution__interp1(...
    'Handler_IK_Model_name', 'Handler_IK_Model', ...
    'Handler_IK_task_name', 'Handler_IK_task', ...
    'IK_Table', IK_Table, ...
    'TimeTable', TimeTable, ...
    'method', 'linear');%linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'
SRD_save(Handler_IK_Solution, 'Handler_IK_Solution')

disp("goal pose: "+ZeroOrderDerivativeNodes(:,2))
disp("Reaeched pose: "+Handler_IK_Model.get_Task(IK_Table(end,:)'))


