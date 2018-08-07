%% Cruise Control: System Analysis
%
% Key MATLAB commands used in this tutorial are:
% <http://www.mathworks.com/help/toolbox/control/ref/ss.html |ss|> , 
% <http://www.mathworks.com/help/toolbox/control/ref/step.html |step|>
%
%% System model and parameters
%
% The transfer function model for the cruise control problem is given
% below.  Please see the < ?example=CruiseControl&section=SystemModeling Cruise Control: System Modeling> 
% page for the derivation.
%
% $$
% P(s) = \frac{V(s)}{U(s)} = \frac{1}{ms+b}  \qquad  [ \frac{m/s}{N} ]
% $$
% 
% The parameters used in this example are as follows: 
%
%  (m)   vehicle mass            1000 kg
%
%  (b)   damping coefficient     50 N.s/m
%
%  (u)   nominal control force   500 N
%
%% Performance specifications
%
% The next step is to come up with some *design criteria* that the
% compensated system should achieve. When the engine gives a 500 Newton force, 
% the car will reach a maximum velocity of 10 m/s (22 mph), see open-loop 
% step response section below. An automobile should be able to accelerate 
% up to that speed in less than 5 seconds. In this application, a 10% overshoot 
% and 2% steady-state error on the velocity are sufficient.
%
% Keeping the above in mind, we have proposed the following design criteria
% for this problem:
%
% * Rise time < 5 s
% * Overshoot < 10%
% * Steady-state error < 2%
%
%% Open-loop step response
%
% The *open-loop* response of the system, without any feedback control, to a step input 
% force of 500 Newtons is simulated in MATLAB as follows:
%
%%

m = 1000;
b = 50;
u = 500;

s = tf('s');
P_cruise = 1/(m*s+b);

step(u*P_cruise)

%%
%
% We see that the open-loop system exhibits no overshoot or oscillations 
% (characteristic of first-order systems), and does reach
% the desired steady-state speed of 10 m/s; however, the rise time is much
% too slow, ~60 s.  Therefore we need to design a feedback controller which
% speeds up the response significantly without negatively affecting
% the other dynamic performance metrics.
%
%% Open-loop poles/zeros
%
% The cruise control system has a single pole at s = -b/m which we can see 
% plotted on the s-plane using the following MATLAB commands:

pzmap(P_cruise)
axis([-1 1 -1 1])

%% 
% We observe that the *open-loop* system is stable and does not oscillate since 
% the pole is real and negative.  Furthermore, the speed of response is 
% determined by the magnitude of this pole, |b/m|: the larger the magnitude, the 
% quicker the system approaches the steady-state value. Since we're
% typically not able to change the system parameters to change the
% dynamic response of the system, we must instead design controllers which
% alter the poles and zeros of the *closed-loop* system to meet the desired
% performance specifications.
%
%% Open-loop Bode plot
%
% We are also interested in the open-loop frequency response of the system
% which we find using the following MATLAB command:

bode(P_cruise)

%%
% We see that the Bode plots exhibit the definitive features of 
% < ?example=Introduction&section=SystemAnalysis#8 first-order>
% systems, including a -3 dB magnitude and -45 deg phase at the 
% corner frequency of w = b/m = 0.05 rad/s and -20 dB/dec roll-off at high frequencies.