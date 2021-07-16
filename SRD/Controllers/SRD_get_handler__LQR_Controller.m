function Handler_LQR = SRD_get_handler__LQR_Controller(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__LQR_Controller';
Parser.addOptional('Handler_State_StateSpace', []);
Parser.addOptional('Handler_ControlInput_StateSpace', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('Handler_Simulation', []);
Parser.addOptional('Handler_InverseDynamics', []);
Parser.addOptional('Q', []);
Parser.addOptional('R', []);

Parser.parse(varargin{:});

Handler_LQR = SRDHandler_Controller;

Handler_LQR.Update = @() Update(...
    Handler_LQR, ...
    Parser.Results.Handler_State_StateSpace, ...
    Parser.Results.Handler_ControlInput_StateSpace, ...
    Parser.Results.Handler_dynamics_Linearized_Model, ...
    Parser.Results.Handler_Simulation, ...
    Parser.Results.Handler_InverseDynamics, ...
    Parser.Results.Q, ...
    Parser.Results.R);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_LQR.SerializationPrepNeeded = true;
Handler_LQR.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_ComputedTorqueController; create a new one on the fly instead')
    end


    function Update(Handler_LQR, ...
            Handler_State_StateSpace, ...
            Handler_ControlInput_StateSpace, ...
            Handler_dynamics_Linearized_Model, ...
            Handler_Simulation, ...
            Handler_InverseDynamics, ...
            Q, R)
        
        t = Handler_Simulation.CurrentTime;
        
        desired = Handler_ControlInput_StateSpace.get_x_dx(t);
        desired_x =  desired(:, 1);
        
        A = Handler_dynamics_Linearized_Model.get_A();
        B = Handler_dynamics_Linearized_Model.get_B();
        
        K = lqr(A, B, Q, R);
        
        e  = reshape( ( Handler_State_StateSpace.x - desired_x), [], 1);
        
        u_FB = -K*e;
        
        u_FF = Handler_InverseDynamics.u;
        
        Handler_LQR.u = u_FB + u_FF;
    end

end