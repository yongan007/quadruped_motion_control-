close all; clear; clc;

InitialPosition = SRD_get('InitialPosition');
Handler_IK_Solution = SRD_get('Handler_IK_Solution_Ikdyn_m');

x0 = Handler_IK_Solution.get_position_velocity_acceleration(0);
Handler_State = SRD_get_handler__state('InitialPosition', InitialPosition, ...
    'InitialVelocity', x0(:,2));

Handler_Time = SRDHandler_Time();

Handler_Desired_State = SRD_get_handler__desired_state(...
    'Handler_ControlInput', Handler_IK_Solution, ...
    'Handler_Time',   Handler_Time);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Handler_dynamics_generalized_coordinates_model = SRD_get('Handler_dynamics_generalized_coordinates_model');
Handler_dynamics_Linearized_Model = SRD_get('Handler_dynamics_Linearized_Model');
Handler_Constraints_Model = SRD_get('Handler_Constraints_Model');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = Handler_IK_Solution.State.TimeTable(end);
% tf = 0.4;
time_table = Handler_IK_Solution.State.TimeTable;

n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;


% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model


[A_table, B_table, c_table, x_table, u_table, dx_table] = ...
    SRD_LinearModel_GenerateTable('Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'Handler_dynamics_Linearized_Model', Handler_dynamics_Linearized_Model, ...
    'Handler_IK_Solution', Handler_IK_Solution, ...
    'TimeTable', time_table);

% n_constrained = 5;
[N_table, G_table, F_table] = SRD_ConstraintsModel_GenerateTable(...
    'Handler_Constraints_Model', Handler_Constraints_Model, ...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'x_table', x_table, ...
    'new_dimentions', []);
% n_constrained = size(N_table, 2);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controllers
% 
% Q = 100*eye(2 * n);
% R = 0.01*eye(Handler_dynamics_generalized_coordinates_model.dof_control);
% Count = size(A_table, 3);
% K_table = SRD_LQR_GenerateTable(A_table, B_table, repmat(Q, [1, 1, Count]), repmat(R, [1, 1, Count]));


% K_table = SRD_CTC_GenerateTable('Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
%     'Handler_IK_Solution', Handler_IK_Solution, ...
%     'Kp', 200*eye(n), 'Kd', 100*eye(n), 'TimeTable', time_table);


Q = 10*eye(2 * n);
R = 1*eye(Handler_dynamics_generalized_coordinates_model.dof_control);
Count = size(A_table, 3);
K_table = SRD_CLQR_GenerateTable(A_table, B_table, repmat(Q, [1, 1, Count]), repmat(R, [1, 1, Count]), N_table);


% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[AA_table, cc_table] = SRD_CloseLoop_GenerateTable(A_table, B_table, c_table, K_table, x_table, u_table);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ode_fnc_handle = SRD_get_ode_fnc_from_ClosedLoopLinearSystem(AA_table, cc_table, time_table);

ode_fnc_handle = SRD_get_ode_fnc_from_ClosedLoopConstrainedLinearSystem...
    (AA_table, cc_table, G_table, F_table, time_table);


x0 = [InitialPosition; zeros(size(InitialPosition))];

[time_table_0, solution_tape] = ode45(ode_fnc_handle, time_table, x0);


Error_x = solution_tape-x_table';
[Max_Error_x,indx] = max(abs(Error_x));
% Max_Error_x = Max_Error_x.*sign(Error_x(indx));


cost_u = zeros(n,length(time_table_0));
cost_qr = 0;
for i=1:length(time_table_0)
cost_u(:,i)= K_table(:,:,i)*Error_x(i,:)';
cost_qr =+ Error_x(i,:)*Q*Error_x(i,:)'+cost_u(:,i)'*R*cost_u(:,i);
end
[Max_cost_u,indx] = max(abs(cost_u));

% Max_cost_u = Max_cost_u.*sign(cost_u(indx));
disp("Max Error of each state:")
disp(Max_Error_x(4:6))
disp(Max_Error_x(17:19))
disp("Max Cost U:")
disp(Max_cost_u(4:6))
disp("Max Cost QR: "+cost_qr)



% figure('Color', 'w', 'Name', 'Solution Tape')
% % % solution_tape ===actual state
% subplot(2, 1, 1)
% plot(time_table_0, solution_tape(:,4:4+2), 'LineWidth', 3)
% title('Solution tape $q$','interpreter','latex')
% legend('$q_1$','$q_2$','$q_3$','interpreter','latex')
% subplot(2, 1, 2)
% plot(time_table_0, solution_tape(:,17:17+2), 'LineWidth', 3);
% title('Solution tape $\dot{q}$','interpreter','latex')
% legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','interpreter','latex')


figure('Color', 'w')
subplot(2, 1, 1)
SRDgraphic_PlotGeneric(time_table_0', x_table(4:6,:)', ...
    'NewFigure', false, ...
    'Title', 'Joint position', ...
    'LableVariable', 'q');
hold on
plot(time_table_0, solution_tape(:,4:4+2), 'LineWidth', 3)
legend('${q_{des1}}$','$q_{des2}$','$q_{des3}$',...
    '$q_{sol1}$','$q_{sol2}$','$q_{sol3}$','interpreter','latex')
hold off

subplot(2, 1, 2)
SRDgraphic_PlotGeneric(time_table_0', x_table(17:19,:)', ...
    'NewFigure', false, ...
    'Title', 'Joint Velocity', ...
    'LableVariable', '\dot{q}');
hold on
plot(time_table_0, solution_tape(:,17:19), 'LineWidth', 3)
legend('$\dot{q}_{des1}$','$\dot{q}_{des2}$','$\dot{q}_{des3}$',...
    '$\dot{q}_{sol1}$','$\dot{q}_{sol2}$','$\dot{q}_{sol3}$','interpreter','latex')
hold off
drawnow;


figure('Color', 'w')
subplot(2, 1, 1)
SRDgraphic_PlotGeneric(time_table_0', Error_x(:,4:6), ...
    'NewFigure', false, ...
    'Title', 'Joints position Erorr', ...
    'LableVariable', 'e');
subplot(2, 1, 2)
SRDgraphic_PlotGeneric(time_table_0', Error_x(:,17:19), ...
    'NewFigure', false, ...
    'Title', 'Joints velocity Erorr', ...
    'LableVariable', '\dot{e}');
drawnow;

SRDgraphic_PlotGeneric(time_table_0', cost_u(4:6,:)', ...
    'NewFigure', true, ...
    'Title', 'Controler cost |u-u*|', ...
    'LableVariable', '{e_u}');
drawnow;


% 
% figure('Color', 'w', 'Name', 'Erorr of X')
% % % solution_tape ===actual state
% subplot(2, 1, 1)
% plot(time_table_0, Error_x(:,1:n), 'LineWidth', 3)
% title('Erorr of $q$','interpreter','latex')
% legend('$e_1$','$e_2$','$e_3$','interpreter','latex')
% subplot(2, 1, 2)
% plot(time_table_0, Error_x(:,n+1:n*2), 'LineWidth', 3);
% title('Erorr of $\dot{q}$','interpreter','latex')
% legend('$\dot{e}_1$','$\dot{e}_2$','$\dot{e}_3$','interpreter','latex')


% % % desired
% figure('Color', 'w')
% subplot(2, 1, 1)
% plot(time_table', x_table(1:3,:)', '--', 'LineWidth', 1);
% subplot(2, 1, 2)
% plot(time_table', x_table(4:6,:)', '--', 'LineWidth', 1);

% figure('Color', 'w')
% plot(time_table, cc_table, 'LineWidth', 3); hold on;




