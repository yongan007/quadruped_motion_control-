function q = SRD_InversePositionProblemSolver_lsqnonlin(Task, TaskJacobian, Value, InitialGuess, opts)

if nargin < 5
    opts = optimoptions(@lsqnonlin,'SpecifyObjectiveGradient',true, 'Algorithm', 'levenberg-marquardt');
end

% [q,resnorm,res,eflag,output] = lsqnonlin(@Objective, InitialGuess, [], [], opts);
q = lsqnonlin(@Objective, InitialGuess, [], [], opts);

    function [F,J] = Objective(q)
%         disp(size(q))
%         disp(size(Value))
        F = Task(q) - Value;
        J = TaskJacobian(q);
    end

end