classdef FSM_AHRS < handle
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Kp = 1;                     % algorithm proportional gain
        Ki = 0;                     % algorithm integral gain
        rampedGain = 10;
        
        qGlobalGyr = NaN;
        AccGlobal = NaN;
        qRotTowardsUp = NaN;
        MagGlobal = NaN;
        qRotTowardsNorth = NaN;
        qGlobalPrevReal  = NaN;
        qGlobalRotatedUp = NaN;

        % these are for storing the angles gamma_a and gamma_m
        gammaA         = NaN;
        gammaM         = NaN;    
        cos_muA_gammaA = NaN;
        sin_muA_gammaA = NaN;
        cos_muM_gammaM = NaN;
        sin_muM_gammaM = NaN;           
    end
    
    %% Public properties
    properties (Access = private)
        eInt = [0 0 0];             % integral error
    end    
 
    %% Public methods
    methods (Access = public)
        function obj = FSM_AHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp'), obj.Kp = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki'), obj.Ki = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        
		% two stage update, pitch roll immune.
        function obj = Updatefan2step(obj, Gyroscope, Accelerometer, Magnetometer, Kpm, Kpa)
            q = obj.Quaternion; % short name local variable for readability

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
 
            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
            
            % Estimated direction of gravity and magnetic field
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
 
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(Accelerometer, v); % + cross(Magnetometer, w); 
            
            % Apply feedback terms
            Gyroscope = Gyroscope + Kpa * e; % + obj.Ki * obj.eInt;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod; 
            q = q / norm(q); % normalise quaternion

        %% Correct for magnetometer - get next rotation around vertical to align measured global xy 
        % Transform magnetometer reading into global frame
		
		% adapter for mike algorithms qMag = quaternProd(q,quaternProd([0,Magnetometer],quaternConj(q)));		
		qGii1 = q(1);
		qGii2 = q(2);
		qGii3 = q(3);
		qGii4 = q(4);
		Mag = Magnetometer;
        Mag1=Mag(1);    Mag2=Mag(2);    Mag3=Mag(3);
        muMag = Kpm;
        muM2 = muMag/2;
        
        if Kpm ~= 0 %~isnan(Mag)
            qm1  = -qGii2*Mag1 - qGii3*Mag2 - qGii4*Mag3;
            qm2  =  qGii1*Mag1 + qGii3*Mag3 - qGii4*Mag2;
            qm3  =  qGii1*Mag2 - qGii2*Mag3 + qGii4*Mag1;
            qm4  =  qGii1*Mag3 + qGii2*Mag2 - qGii3*Mag1;  
            gMag1 = -qm1*qGii2 + qm2*qGii1 - qm3*qGii4 + qm4*qGii3;
            gMag2 = -qm1*qGii3 + qm2*qGii4 + qm3*qGii1 - qm4*qGii2;
            % Get fraction of rotation from mag in global to north (xy components only)
            if muM2<0,    
                muM2 = 0;    warning('Capping mu at 0');
            elseif muM2>0.5,
                muM2 = 0.5;  warning('Capping mu at 0.5');
            end
            qn2 = 0; qn3 = 0;  % these two variable are zeros.
            if gMag2 == 0 % y-component is 0 therefore pointing north
                qn1 = 1;  qn4 = 0;
            else
                gamma_m      = arctan2_approx(abs(gMag2),gMag1); % absolute value unsure if always correct
                obj.gammaM   = gamma_m;
                mu_m_gamma_m = muM2*gamma_m;

                cos_mu_m_gamma_m = 1;
                sin_mu_m_gamma_m = mu_m_gamma_m;
                
                obj.cos_muM_gammaM = cos_mu_m_gamma_m;
                obj.sin_muM_gammaM = sin_mu_m_gamma_m;
                
                qn1 = cos_mu_m_gamma_m;
                qn4 = java.lang.Math.signum(-gMag2)*sin_mu_m_gamma_m;
                qNorm = 1/java.lang.Math.sqrt(qn1*qn1+qn4*qn4);  % can be replaced by the fast inverse square root
                qn1 = qn1*qNorm;
                qn4 = qn4*qNorm; % normalise to unit quaternions
            end
            % Rotate global frame towards 'north'        
            qGiii1 = qn1*qGii1 - qn2*qGii2 - qn3*qGii3 - qn4*qGii4;
            qGiii2 = qn1*qGii2 + qn2*qGii1 + qn3*qGii4 - qn4*qGii3;
            qGiii3 = qn1*qGii3 - qn2*qGii4 + qn3*qGii1 + qn4*qGii2;
            qGiii4 = qn1*qGii4 + qn2*qGii3 - qn3*qGii2 + qn4*qGii1;
        else
            qGiii1 = qGii1; qGiii2 = qGii2; qGiii3 = qGii3; qGiii4 = qGii4;
        end
        q = [qGiii1 qGiii2 qGiii3 qGiii4];  
		
		obj.Quaternion = q;
        end
	end
end

function [atan2x]=arctan2_approx(y,x)
    % only works when y is positive i.e. the numerator in atan2(y,x)
    if x >= 0
        if x>=y
            invsqrtxsq = java.lang.Math.signum(x)*(1/java.lang.Math.sqrt(x*x)); % can be replaced by the fast inverse square root
            %atan2x = arctan_approx(y/x);
            atan2x = arctan_approx(y*invsqrtxsq);
        else % x < y
            invsqrtysq = java.lang.Math.signum(y)*(1/java.lang.Math.sqrt(y*y)); % can be replaced by the fast inverse square root
            %atan2x = pi/2 - arctan_approx(x/y);
            atan2x = pi/2 - arctan_approx(x*invsqrtysq);
        end
    else % x<=0
        if y>abs(x)
            %atan2x = pi/2 + arctan_approx(abs(x)/y);
            invsqrtysq = java.lang.Math.signum(y)*(1/java.lang.Math.sqrt(y*y)); % can be replaced by the fast inverse square root
            atan2x = pi/2 + arctan_approx(abs(x)*invsqrtysq);
        else 
            %atan2x = pi - arctan_approx(y/abs(x));                        
            invsqrtxsq = 1/java.lang.Math.sqrt(x*x); % can be replaced by the fast inverse square root
            atan2x = pi - arctan_approx(y*invsqrtxsq);
        end
    end
end

function [atanx]=arctan_approx(x)
qtr_pi = pi/4;
% multiply (x4)
% addition (x1)
% subtraction (x2)
atanx = (qtr_pi.*x) - x.*(abs(x)-1) .* (0.2447 + 0.0663 .* abs(x));
end


