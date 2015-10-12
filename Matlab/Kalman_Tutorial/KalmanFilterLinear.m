classdef KalmanFilterLinear < handle
    %KalmanFilterLinear 
    %   Detailed explanation goes here
    
    properties
        %Constant values
        A; %State transition matrix
        B; %Control matrix
        H; %Observation matrix
        Q; %Estimated process error covariance
        R; %Estimated measurement error covariance
        %Updated values
        x; %Newest estimate of state matrix
        P; %Newest estimate of covariance (average error for each part of the state)
    end % properties
    
    methods
        % Constructor for the object
        function obj = KalmanFilterLinear(A_,B_,H_,Q_,R_,x0,P0)
            obj.A = A_;
            obj.B = B_;
            obj.H = H_;
            obj.Q = Q_;
            obj.R = R_;
            obj.x = x0;
            obj.P = P0;
        end % function constructor
        
        % Compute one time step of the Kalman filter and update x an P
        function step(obj, u_n, z_n)
            %Prediction step
            x_pred = obj.A*obj.x + obj.B*u_n;
            P_pred = obj.A*obj.P*transpose(obj.A) + obj.Q;
            %Observation step
            y_tilde = z_n - obj.H*x_pred;
            S = obj.H*P_pred*transpose(obj.H)+obj.R;
            %Update step
            K = P_pred*transpose(obj.H)/S;
            obj.x = x_pred + K*y_tilde;
            ksize = size(obj.P,1);
            obj.P = (eye(ksize) - K*obj.H)*P_pred;
        end %function step
        
        % Get the current value of the state
        function x_n = getCurrentState(obj)
            x_n = obj.x;
        end %function getCurrentStep
            
    end % methods
    
end % class

