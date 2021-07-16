%This class is cenetered around manipulation with precomputed simbolic
%expressions related to robot dynamics. The idea is to have basic dynamics 
%algorithms in the SRDMechanicalEquations, and all the additional stuff,
%as well as the data structure & management here.
%
%last update 10.12.17
classdef SRDMechanicalEquationsManager < SRDMechanicalEquations
    properties
        
    end
    methods
        
        % class constructor
        % initializes properties s and calls the superclass constructor.
        function obj = SRDMechanicalEquationsManager(LinkArray, Casadi)
            obj = obj@SRDMechanicalEquations(LinkArray, Casadi);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        % Basic methods (generating dynamics eq.)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
        % This function updates GeometryArray property calling
        % PrepareGeometry method
        function UpdateGeometryArray(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = true;
            end
            
            timerVal = tic;
            obj.GeometryArray = obj.PrepareGeometryArray(ToSimplify);
            toc(timerVal);
        end
        
        % This function updates AngularVelocityArray property calling
        % PrepareAngularVelocityArray method
        % if ToSimplify == true, method PrepareAngularVelocityArray
        % will be simplifying everything, which makes the calculations
        % take much longer.
        function UpdateAngularVelocityArray(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false;   %the default value of ToSimplify
            end
            
            timerVal = tic;
            obj.AngularVelocityArray = obj.PrepareAngularVelocityArray(ToSimplify);
            toc(timerVal);
        end
        
        % This function updates GeometryArray property calling
        % PrepareKinematicsArray method
        % if ToSimplify == true, method PrepareKinematicsArray
        % will be simplifying everything, which makes the calculations
        % take much longer.
        function UpdateKinematicsArray(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false;   %the default value of ToSimplify
            end
            
            timerVal = tic;
            obj.KinematicsArray = obj.PrepareKinematicsArray(ToSimplify);
            toc(timerVal);
        end
       
        %This function returns a vector - the right hand side of the
        %manipulator equations: %H*ddq + C - G - F = Q;
        %where ddq = d^2(q)/dt^2, %Q - gen forces;
        %RHS =  Q - C + G + F;
        function RHS = GetRightHandSide_ofManipulatorEq(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            c = obj.GetForcesForComputedTorqueController();
            RHS = obj.ForwardDynamicsStructure.ExternalAndMotorForces - c;
%             RHS = obj.ForwardDynamicsStructure.ExternalAndMotorForces - ...
%                 obj.ForwardDynamicsStructure.GenInertialForces + ...
%                 obj.ForwardDynamicsStructure.GenGravitationalForces + ...
%                 obj.ForwardDynamicsStructure.GenDisspativeForces;
    
            if ToSimplify
                disp('Started simplifying right hand side of the manipulator equations');
                RHS = obj.Math.simplify(RHS, 'right hand side of the manipulator equations');
            end
        end
        
        %We have H*ddq + c = Q;
        %H - joint space inertia matrix
        %c - skew vector (see Featherstone for notation), here referred to
        % as ForcesForComputedTorqueController
        %Q - gen. forces         
        function c = GetForcesForComputedTorqueController(obj)
            c = obj.ForwardDynamicsStructure.GenInertialForces - ...
                obj.ForwardDynamicsStructure.GenGravitationalForces - ...
                obj.ForwardDynamicsStructure.GenDisspativeForces;
        end  
        
        % this function updates ForwardDynamicsStructure that holds
        % expressions for forward dynamics
        function UpdateForwardDynamicsStructure(obj, ToSimplify)
            timerVal = tic;
            
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end            
            
            obj.ForwardDynamicsStructure.JSIM = obj.GetGenInertiaMatrix(ToSimplify);                             
            disp('* Finished calculating JSIM'); disp(' ');
            
            [C, dJSIM] = obj.GetGenInertialForces(ToSimplify);
            obj.ForwardDynamicsStructure.GenInertialForces = C;
            obj.ForwardDynamicsStructure.dJSIM = dJSIM;
            disp('* Finished calculating generalised inertial forces'); disp(' ');
            
            obj.ForwardDynamicsStructure.GenGravitationalForces = obj.GetGenGravitationalForces(ToSimplify);     
            disp('* Finished calculating generalised gravitational forces'); disp(' ');
            
            obj.ForwardDynamicsStructure.GenDisspativeForces = obj.GetSimpleGenDisspativeForces();               
            disp('* Finished calculating generalised dissipative forces'); disp(' ');
            
            obj.ForwardDynamicsStructure.ExternalAndMotorForces = obj.GetGenExternalAndMotorForces(ToSimplify); 
            disp('* Finished calculating generalised external and motor forces'); disp(' ');
            
            obj.ForwardDynamicsStructure.ControlActionsToGenMotorTorquesMap = obj.GetControlActionsToGenMotorTorquesMap(ToSimplify);
            disp('* Finished calculating linear map that maps control actions to generalized forses due to motor torques'); disp(' ');
            
            obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq = obj.GetRightHandSide_ofManipulatorEq(ToSimplify);  
            disp('* Finished calculating the right hand side of the manipulator equations'); disp(' ');
            
            obj.ForwardDynamicsStructure.ForcesForComputedTorqueController = obj.GetForcesForComputedTorqueController;
            disp('* Finished calculating skew vector (forces for the computed torque controller)'); disp(' ');
            
            toc(timerVal);
        end        
        
        %This function does all the steps for building the dynamics of the
        %system
        function BuildDynamicsEquations(obj, ToSimplify, ToUpdateKinematics)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false;   %the default value of ToSimplify
            end
            if nargin < 3
                ToUpdateKinematics = false;   %the default value of ToUpdateKinematics
            end
            
            obj.UpdateGeometryArray(ToSimplify);
            if ToUpdateKinematics
                obj.UpdateKinematicsArray(ToSimplify);
            end
            obj.UpdateAngularVelocityArray(ToSimplify);
            if ~obj.MotorActionStructure_user_defined
                obj.UpdateMotorActionStructure();
            end
            obj.UpdateExternalAndMotorForcesArray(ToSimplify);
            obj.UpdateForwardDynamicsStructure(ToSimplify);
        end
        
        
        %This function generates a few M functions, one
        %g_dynamics_JSIM.m for evaluating JSIM -
        %the joint space inertia matrix of the mechanism,
        %second one is g_dynamics_RHS.m for evaluating the right
        %hand side of the manipulator equations,
        %third one is g_dynamics_ControlMap.m for evaluating
        %linear map that maps control actions to generalized forses due to
        %torques
        function GenerateForwardDynamicsFunctions(obj)
            timerVal = tic;
                
            %generate functions
            disp('Started generating JSIM function');
            matlabFunction(obj.ForwardDynamicsStructure.JSIM, 'File', ...
                'g_dynamics_JSIM', 'Vars', {obj.q}, 'Optimize', obj.ToOptimizeFunctions);
               
            disp('Started generating Right Hand Side of Manipulator Eq function');
            matlabFunction(obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq, 'File', ...
                'g_dynamics_RHS', 'Vars', {obj.q, obj.v, obj.u}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('Started generating Control Actions To Gen Motor Torques Map function');
            matlabFunction(obj.ForwardDynamicsStructure.ControlActionsToGenMotorTorquesMap, 'File', ...
                'g_dynamics_ControlMap', 'Vars', {obj.q}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('Started generating function for forces for the computed torque controller');
            matlabFunction(obj.ForwardDynamicsStructure.ForcesForComputedTorqueController, 'File', ...
                'g_control_ForcesForComputedTorqueController', 'Vars', {obj.q, obj.v}, 'Optimize', obj.ToOptimizeFunctions); 
            
            toc(timerVal);
        end
        
        
        function GenerateForwardDynamicsFunctions_Casadi(obj)
            timerVal = tic;
            import casadi.*
            
            %generate functions
            disp('Started generating JSIM function');
            g_dynamics_JSIM = Function('g_dynamics_JSIM', {obj.q}, {obj.ForwardDynamicsStructure.JSIM}, {'q'}, {'H'});
               
            disp('Started generating Right Hand Side of Manipulator Eq function');
            g_dynamics_RHS = Function('g_dynamics_RHS', {obj.q, obj.v, obj.u}, {obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq}, ...
                {'q', 'v', 'u'}, {'RHS'});
            
            disp('Started generating Control Actions To Gen Motor Torques Map function');
            g_dynamics_ControlMap = Function('g_dynamics_ControlMap', ...
                {obj.q}, {obj.ForwardDynamicsStructure.ControlActionsToGenMotorTorquesMap}, {'q'}, {'B'});
            
            disp('Started generating function for forces for the computed torque controller');
            g_control_ForcesForComputedTorqueController = Function('g_control_ForcesForComputedTorqueController', ...
                {obj.q, obj.v}, {obj.ForwardDynamicsStructure.ForcesForComputedTorqueController}, ...
                {'q', 'v'}, {'c'});
            
            
            CG = CodeGenerator('g_dynamics.c');
            CG.add(g_dynamics_JSIM);
            CG.add(g_dynamics_RHS);
            CG.add(g_dynamics_ControlMap);
            CG.add(g_control_ForcesForComputedTorqueController);
            CG.generate();
            
            !gcc -fPIC -shared g_dynamics.c -o g_dynamics.so
            
            toc(timerVal);
        end
              
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        % System uncertainty-related methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
        
        %We have H*ddq + c = Q;
        %H - joint space inertia matric
        %c - skew vector (see Featherstone for notation)
        %Q - gen. forces
        %
        %Y*theta := H*ddq + c, hence
        %Y = d(H*ddq + c)/d(theta); Y is called regressor
        %
        %theta - Ortega-Spong model parameters (masses, lengths and so on)
        %that will be preserved in the quations;
        %see  R Ortega, M. Spong "Adaptive motion control of rigid robots: 
        %a tutorial" for details
        function [Y, Regressor_constant] = GetOrtegaSpongRegressor(obj, theta)
            
            JSIM = obj.ForwardDynamicsStructure.JSIM;
            c = obj.ForwardDynamicsStructure.ForcesForComputedTorqueController;
            
            LHS = JSIM*obj.a + c;
            
            Y = jacobian(LHS, theta);
            Y = obj.Math.simplify(Y, 'regressor');
            
            Regressor_constant = LHS - Y*theta;
            Regressor_constant = obj.Math.simplify(Regressor_constant, 'regressor constant');
        end
        
        function GenerateOrtegaSpongRegressor(obj, theta)
            disp('Started deriving Ortega Spong regressor function');
            [Regressor, Regressor_constant] = obj.GetOrtegaSpongRegressor(theta);
            obj.ForwardDynamicsStructure.Regressor = Regressor;
            obj.ForwardDynamicsStructure.Regressor_constant = Regressor_constant;
            
            disp('Started generating Ortega Spong regressor function');
            matlabFunction(Regressor, 'File', ...
                'g_OrtegaSpong_Regressor', 'Vars', {obj.q, obj.v, obj.a}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('Started generating Ortega Spong regressor constant function');
            matlabFunction(Regressor_constant, 'File', ...
                'g_OrtegaSpong_Regressor_constant', 'Vars', {obj.q, obj.v, obj.a}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('Ortega Spong regressor function - generated');
        end
        
        %theta - Ortega-Spong model parameters (masses, lengths and so on)
        %that will be preserved in the quations;
        %see  R Ortega, M. Spong "Adaptive motion control of rigid robots: 
        %a tutorial" for details
        function GenerateForwardDynamicsFunctions_ExplicitParameters(obj, theta)
            timerVal = tic;
                
            %generate functions
            disp('Started generating JSIM function');
            matlabFunction(obj.ForwardDynamicsStructure.JSIM, 'File', ...
                'g_dynamics_JSIM', 'Vars', {obj.q, theta}, 'Optimize', obj.ToOptimizeFunctions);
               
            disp('Started generating Right Hand Side of Manipulator Eq function');
            matlabFunction(obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq, 'File', ...
                'g_dynamics_RHS', 'Vars', {obj.q, obj.v, obj.u, theta}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('Started generating Control Actions To Gen Motor Torques Map function');
            matlabFunction(obj.ForwardDynamicsStructure.ControlActionsToGenMotorTorquesMap, 'File', ...
                'g_dynamics_ControlMap', 'Vars', {obj.q, theta}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('Started generating function for forces for the computed torque controller');
            matlabFunction(obj.ForwardDynamicsStructure.ForcesForComputedTorqueController, 'File', ...
                'g_control_ForcesForComputedTorqueController', 'Vars', {obj.q, obj.v, theta}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('* Finished generating forward dynamics functions'); disp(' ');
            toc(timerVal);
        end        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        % Additional methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % This function calculates the geometry and basic kinematics of
        % the center of mass
        % if ToSimplify == true, the method will be simplifying
        % everything, which makes the calculations much longer.
        function [rC, vC, aC, J, J1, J2] = GetCoM(obj, ToSimplify)
            
            if nargin < 2
                ToSimplify = false; %the default value of ToSimplify
            end
            
            Mass = zeros(obj.nob, 1);
            
            rC = []; vC = []; aC = []; J = []; J1 = []; J2 = [];
            for i = 1:obj.nob
                GeometryStructure = obj.GeometryArray{i};
                
                Mass(i) = GeometryStructure.Link.Mass;
                
                if isempty(rC)
                    rC = Mass(i)*GeometryStructure.rC;
                    J =  Mass(i)*GeometryStructure.J;
                else
                    rC = rC + Mass(i)*GeometryStructure.rC;
                    J =  J  + Mass(i)*GeometryStructure.J;
                end
                
                if nargout > 2
                    KinematicsStructure = obj.KinematicsArray{i};
                    if isempty(vC)
                        vC = Mass(i)*KinematicsStructure.vC;
                        aC = Mass(i)*KinematicsStructure.aC;
                        J1 = Mass(i)*KinematicsStructure.J1;
                        J2 = Mass(i)*KinematicsStructure.J2;
                    else
                        vC = vC + Mass(i)*KinematicsStructure.vC;
                        aC = aC + Mass(i)*KinematicsStructure.aC;
                        J1 = J1 + Mass(i)*KinematicsStructure.J1;
                        J2 = J2 + Mass(i)*KinematicsStructure.J2;
                    end
                end
                
            end
            rC = (1/sum(Mass))*rC;
            J =  (1/sum(Mass))*J;
            if nargout > 2
                vC = (1/sum(Mass))*vC;
                aC = (1/sum(Mass))*aC;
                J1 = (1/sum(Mass))*J1;
                J2 = (1/sum(Mass))*J2;
            end
            disp('Finished calculating the geometry of the CoM');
            
            if ToSimplify
                rC = obj.Math.simplify(rC, 'rC'); disp('Finished simplifying the position of the CoM');
                J  = obj.Math.simplify(J, 'J');  disp('Finished simplifying the Jacobian J of the CoM');
                
                if nargout > 2
                    vC = obj.Math.simplify(vC, 'vC'); disp('Finished simplifying the velocity of the CoM');
                    aC = obj.Math.simplify(aC, 'aC'); disp('Finished simplifying the acceleration of the CoM');
                    J1 = obj.Math.simplify(J1, 'J1'); disp('Finished simplifying the Jacobian J1 of the CoM');
                    J2 = obj.Math.simplify(J2, 'J2'); disp('Finished simplifying the Jacobian J2 of the CoM');
                end
            end
        end        
        
        
        
    end
end