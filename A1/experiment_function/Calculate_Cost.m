function [Max_Error_x,Max_cost_u,cost_qr]=Calculate_Cost(time_table,x_table, K_table, solution_tape,Q,R) 
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
end