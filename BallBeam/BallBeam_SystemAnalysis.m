%% Ball & Beam: System Analysis
%
% Key MATLAB commands used in this tutorial are:
% <http://www.mathworks.com/help/toolbox/control/ref/tf.html |tf|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/pzmap.html |pzmap|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/step.html |step|>
%
%% System model
%
% The transfer function from the gear angle ($\theta(s)$) to the ball position
% ($R(s)$), as derived in the
% < ?example=BallBeam&section=SystemModeling Ball & Beam: System Modeling> page.
%
% $$ P(s) = \frac{R(s)}{\Theta(s)} = -\frac{mgd}{L \left(\frac{J}{R^2}+m\right)}
% \frac{1}{s^2} \qquad [ \frac{m}{rad} ]$$
%
% Open a new < ?aux=Extras_Mfile
% m-file> and add the following code to create a transfer function model in
% MATLAB.
%
%%

m = 0.111;
R = 0.015;
g = -9.8;
L = 1.0;
d = 0.03;
J = 9.99e-6;

s = tf('s');
P_ball = -m*g*d/L/(J/R^2+m)/s^2

%% Pole/zero map
%
% The Ball and Beam system is a type II system which has two poles at the
% origin, as seen in the pole/zero map below.  Since the poles are
% not strictly in the left half plane, the open loop system will be unstable as seen
% in the step response below.
%
%%

pzmap(P_ball)

%% Open-loop step response
%
% Now, we would like to observe the ball's response to a step input on the
% motor servo gear angle theta (1-radian step). To do this you will need to add the
% following line to your m-file.
%
%%

step(P_ball)

%%
%
% From this plot it is clear that the system is unstable in open-loop
% causing the ball to roll right off the end of the beam. Therefore, some
% method of controlling the ball's position in this system is required.
% Several examples of controller design are provided in these tutorials to
% address this problem.
