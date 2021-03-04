%% Cruise Control: System Modeling
%
% Key MATLAB commands used in this tutorial are:
% <http://www.mathworks.com/help/toolbox/control/ref/ss.html |ss|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/tf.html |tf|>
%
%% Physical setup
%
% Automatic *cruise control* is an excellent example of a feedback control
% system found in many modern vehicles. The purpose of the cruise control
% system is to maintain a constant vehicle speed despite external *disturbances*,
% such as changes in wind or road grade. This is accomplished by
% measuring the vehicle speed, comparing it to the desired or *reference* speed,
% and automatically adjusting the throttle according to a *control law*.
%
% <<Content/CruiseControl/System/Modeling/figures/cruise_control_schematic.png>>
%
% We consider here a simple model of the vehicle dynamics, shown in the
% free-body diagram (FBD) above.  The vehicle, of mass m, is acted on by a
% control force, u. The force u represents the force generated at the
% road/tire interface. For this simplified model we will assume that we can
% control this force directly and will neglect the dynamics of the
% powertrain, tires, etc., that go into generating the force. The resistive
% forces, bv, due to rolling resistance and wind
% drag, are assumed to vary linearly with the vehicle velocity, v, and act
% in the direction opposite the vehicle's motion.
%% System equations
%
% With these assumptions we are left with a
% < ?example=Introduction&section=SystemAnalysis#8 first-order>
% mass-damper system. Summing forces in the x-direction and applying Newton's 2nd law, we
% arrive at the following system equation:
%
% $$
% m \dot{v} + b v = u
% $$
%
% Since we are interested in controlling the speed of the vehicle, the output
% equation is chosen as follows
%
% $$
% y = v
% $$
%
%% System parameters
%
% For this example, let's assume that the parameters of the system are:
%
%  (m)   vehicle mass          1000 kg
%
%  (b)   damping coefficient   50 N.s/m
%
%% State-space model
%
% First-order systems have only has a single energy storage mode, in this case the
% kinetic energy of the car, and therefore only one state variable is
% needed, the velocity.  The state-space representation is therefore:
%
% $$
% \dot{\mathbf{x}}=[\dot{v}]=\left[\frac{-b}{m}\right][v]+\left[\frac{1}{m}\right][u]
% $$
%
% $$
% y=[1][v]
% $$
%
% We enter this state-space model into MATLAB using the following
% commands:
%
%%

m = 1000;
b = 50;

A = -b/m;
B = 1/m;
C = 1;
D = 0;

cruise_ss = ss(A,B,C,D);

%% Transfer function model
%
% Taking the Laplace transform of the governing differential equation and
% assuming zero initial conditions, we find the transfer function of the
% cruise control system to be:
%
% $$
% P(s) = \frac{V(s)}{U(s)} = \frac{1}{ms+b}  \qquad  [ \frac{m/s}{N} ]
% $$
%
% We enter the transfer function model into MATLAB using the following
% commands:
%
%%

s = tf('s');
P_cruise = 1/(m*s+b);
