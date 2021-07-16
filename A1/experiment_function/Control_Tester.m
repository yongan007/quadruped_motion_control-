

function [Max_Error_x,Max_cost_u,cost_qr]=Control_Tester(time_table,x_table, K_table, solution_tape,Q,R) 

n = size(x_table,1)/2;
Error_x = solution_tape-x_table';
Max_Error_x = max(abs(Error_x(:,1:n)));

cost_u = zeros(n,length(time_table));
cost_qr = 0;
for i=1:length(time_table)
cost_u(:,i)= K_table(:,:,i)*Error_x(i,:)';
cost_qr =+ Error_x(i,:)*Q*Error_x(i,:)'+cost_u(:,i)'*R*cost_u(:,i);
end
Max_cost_u = max(abs(cost_u'));

disp("Max Error of each state:")
disp(Max_Error_x)
disp("Max Cost U:")
disp(Max_cost_u)
disp("Max Cost QR: "+cost_qr)

figure('Color', 'w','Position', [10 10 1000 700])

subplot(2, 1, 1)
plot(time_table, x_table(1:n,:),'--', 'LineWidth', 3);
hold on

plot(time_table, solution_tape(:,1:n), 'LineWidth', 3)
title('Joint position')

L = legend('${q_{ref1}}$','$q_{ref2}$','$q_{ref3}$',...
    '${q_{ref4}}$','$q_{ref5}$','$q_{ref6}$',...
    '${q_{ref7}}$','$q_{ref8}$','$q_{ref9}$',...
    '${q_{ref10}}$','$q_{ref11}$','$q_{ref12}$',...
    '${q_{sol1}}$','$q_{sol2}$','$q_{sol3}$',...
    '${q_{sol4}}$','$q_{sol5}$','$q_{sol6}$',...
    '${q_{sol7}}$','$q_{sol8}$','$q_{sol9}$',...
    '${q_{sol10}}$','$q_{sol11}$','$q_{sol12}$','interpreter','latex');
L.NumColumns = 8;
L.FontSize = 9;
L.Location = 'southoutside';
grid on; grid minor;
ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 16;


subplot(2, 1, 2)
plot(time_table, x_table(n+1:2*n,:),'--', 'LineWidth', 3)
hold on
plot(time_table, solution_tape(:,n:2*n), 'LineWidth', 3)
title('Joint Velocity')
L = legend('$\dot{q}_{ref1}$','$\dot{q}_{ref2}$','$\dot{q}_{ref3}$',...
    '$\dot{q}_{ref4}$','$\dot{q}_{ref5}$','$\dot{q}_{ref6}$',...
    '$\dot{q}_{ref7}$','$\dot{q}_{ref8}$','$\dot{q}_{ref9}$',...
    '$\dot{q}_{ref10}$','$\dot{q}_{ref11}$','$\dot{q}_{ref12}$',...
    '$\dot{q}_{sol1}$','$\dot{q}_{sol2}$','$\dot{q}_{sol3}$',...
    '$\dot{q}_{sol4}$','$\dot{q}_{sol5}$','$\dot{q}_{sol6}$',...
    '$\dot{q}_{sol7}$','$\dot{q}_{sol8}$','$\dot{q}_{sol9}$',...
    '$\dot{q}_{sol10}$','$\dot{q}_{sol11}$','$\dot{q}_{sol12}$','interpreter','latex');
L.NumColumns = 8;
L.FontSize = 9;
L.Location = 'southoutside';
grid on; grid minor;
ax = gca;
ax.FontName = 'Times New Roman';
ax.FontSize = 16;

hold off
drawnow;


figure('Color', 'w','Position', [10 10 1000 700])
subplot(2, 1, 1)
SRDgraphic_PlotGeneric(time_table', Error_x(:,1:n), ...
    'NewFigure', false, ...
    'Title', 'Joints position Erorr', ...
    'LableVariable', 'e');
subplot(2, 1, 2)
SRDgraphic_PlotGeneric(time_table', Error_x(:,n+1:2*n), ...
    'NewFigure', false, ...
    'Title', 'Joints velocity Erorr', ...
    'LableVariable', '\dot{e}');
drawnow;


SRDgraphic_PlotGeneric(time_table', cost_u(:,:)', ...
    'NewFigure', true, ...
    'Title', 'Controler cost |u-u*|', ...
    'LableVariable', '{e_u}');
drawnow;
end 