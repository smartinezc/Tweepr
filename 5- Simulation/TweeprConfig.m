%%                      Tweepr  - State-Space Model
%                               Simulations
%--------------------------------------------------------------------------

%% Model parameters
mb = 0.710;     % Mass of base              [kg]
mp = 0.157;     % Mass of upper body        [kg]
g = 9.81;       % Earth gravity             [m/s^2]
L = 0.145;      % Length of the pendulum    [m]
d1 = 0.01;      % Damping of displacement   []
d2 = 0.01;      % Damping of joint          []

%% Matrix for state-space model
A = [0,         0,                  1,                  0;
     0,         0,                  0,                  1;
     0,     (g*mp)/mb,          -d1/mb,             -d2/(L*mb);
     0, g*(mb+mp)/(L*mb),     -d1/(L*mb),   -(d2*(mb+mp))/(L^(2)*mb*mp)];
 
B = [0;        0;                 1/mb;              1/(L*mb)];
 
% Output matrix
C = [1; 0; 0; 0];       % q1 as unique output
% C = [0; 1; 0; 0];       % q2 as unique output
D = 0;                  % To complete matrix system


%% System representation
sys = ss(A, B, C', D);
pole(sys)
% rlocus(sys);

                                % Initial system condition
x0 = [0; 5*(pi/180); 0; 0];     % Pos:0m - Ang:5°
                                % Vel:0m/s - AngVel:0rad/s
                                
%% Space-State feedback controller
desirePoles = [-3; -3; -3; -3]; % New desire poles of the system
K = acker(A, B, desirePoles);   % Gain vector controller using Ackermann's

%% LQ Regulator Controller
Q = 10*eye(4);                  % States weights
R = 1;                          % Control input weights
K = lqr(A, B, Q, R);            % Optimal gain vector
