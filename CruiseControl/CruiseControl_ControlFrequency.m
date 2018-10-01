%% Cruise Control: Frequency Domain Methods for Controller Design
%
% Key MATLAB commands used in this tutorial are:
% <http://www.mathworks.com/help/toolbox/control/ref/tf.html |tf|> , 
% <http://www.mathworks.com/help/toolbox/control/ref/feedback.html |feedback|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/step.html |step|>
%
%% System model
%
% The transfer function model for the cruise control problem is given
% below.  Please see the
% < ?example=CruiseControl&section=SystemModeling Cruise Control: System Modeling> 
% page for the derivation.
%
% $$
% P(s) = \frac{V(s)}{U(s)} = \frac{1}{ms+b} \qquad  [ \frac{m/s}{N} ]
% $$
%
%% System parameters
%
% For this example, let's assume that the parameters of the system are
% 
%  (m)   vehicle mass            1000 kg
%
%  (b)   damping coefficient     50 N.s/m
%
%  (r)   reference speed         10 m/s
%
%  (u)   nominal control force   500 N
%
%%
% and the block diagram of an typical unity feedback system is shown below.
% 
% <<Content/CruiseControl/Control/Frequency/figures/feedback_cruise.png>>
%
%% Performance specifications
% 
% * Rise time < 5 sec
% * Overshoot < 10%
% * Steady-state error < 2%
%
%% Bode plot and open-loop response
% The first step in solving this problem using frequency response is to
% determine what open-loop transfer function to use. Just like for the 
% Root-Locus design method, we will only use a proportional controller to 
% solve the problem. The block diagram and the open-loop transfer function 
% are shown below.
%
% <<Content/CruiseControl/Control/Frequency/figures/openloop_cruise.png>>
%
% $$ \frac{Y(s)}{E(s)} = \frac{K_p}{m s + b} \ $$
%
% In order to use a Bode plot, the open-loop response must be stable. Let Kp
% equal 1 for now and see how the open-loop response looks like. Create a 
% new < ?aux=Extras_Mfile m-file> and enter the following commands.

m = 1000;
b = 50;
u = 500;

Kp = 1;
s = tf('s');
P_cruise = 1/(m*s+b);
C = Kp;
step(u*C*P_cruise)

%%
% As you can see, the open-loop system is stable; thus, we can go ahead and
% generate the Bode plot. Change the above m-file by deleting the |step| 
% command and adding in the following command.

bode(C*P_cruise);

%% Proportional controller
% Refer to the 
% < ?example=Introduction&section=ControlFrequency Introduction: Frequency
% Domain Methods for Controller Design> page, 
% and let's see what system characteristics we can determine from the above Bode plot. 
%
% The steady-state error can be found from the following equation:
%
% $$ \mathrm{ss \ error} = \frac{1}{1+M_{\omega \rightarrow 0}} \cdot 100\% \ $$
%
% For this system, the low frequency gain is -34dB = 0.02; therefore, the steady-state error should be 98%. We can confirm this by 
% generating a closed-loop step response as follows.

r = 10;
sys_cl = feedback(C*P_cruise,1);
step(r*sys_cl);

%%
% We need to increase the low frequency gain in order to improve the
% steady-state error. Specifically, the error needs to be < 2%; therefore,
% 1/(1+M_{w=0}) < 0.02 -> M_{w=0} > 49 = 33.8 dB. So to reach the desired
% steady-state error using proportional control only requires a Kp > 67.8
% dB = 2455. Let's look at the Bode diagram of the compensated open-loop system.

Kp = 2500;
C = Kp;

bode(C*P_cruise);

%%
% As you can see from the Bode plot above, the low frequency magnitude is now, 34 dB. 
% Now let's simulate the step response of the closed loop system with this gain.

sys_cl = feedback(C*P_cruise,1);
step(r*sys_cl);

%%
% The steady-state error meets the requirements; however, the rise time is
% much shorter than is needed and is unreasonable in this case since the car
% can not accelerate to 10 m/s in 2 sec. Therefore, we will try using a
% smaller proportional gain to reduce the control action required along
% with a lag compensator to reduce the steady-state error.
%
%% Lag compensator
% If you take a look at the "Lag or Phase-Lag Compensator using Frequency
% Response" section of the < ?aux=Extras_Leadlag Lead and Lag Compensator
% Design> page, the lag compensator  
% adds gain at the low frequencies while keeping the bandwidth frequency at 
% the same place. This is actually what we need: Larger low frequency gain 
% to reduce the steady-state error and keep the same bandwidth frequency to 
% maintain the desired rise time. The transfer function of the lag controller is: 
%
% $$C(s) = \frac{s+z_0}{s+p_0} \ $$
%
% If you read the "Lag or Phase-Lag Compensator using Root-Locus" section
% in < ?aux=Extras_Leadlag Lead and Lag Compensator Design> page, the pole
% and the zero of a lag controller  
% need to be placed close together. Also, it states that the steady-state 
% error will be reduce by a factor of zo/po. For these reasons, let zo equal
% 0.1 and po equal 0.02.  The proportional gain, Kp = 1000
% was chosen by trial-and-error.
%
%%

Kp = 1000;
zo = 0.1;
po = 0.02;

C_lag = (s+zo)/(s+po); 
bode(Kp*C_lag*P_cruise);

%%
% Let's confirm the performance by generating a closed-loop step response.

sys_cl = feedback(Kp*C_lag*P_cruise,1);
t = 0:0.1:20;
step(r*sys_cl,t);

%%
% As you can see, there is a very slight overshoot, the steady state error is close to zero, and the 
% rise time is under 5 seconds. The system has now met all of the design requirements. 
% No more iteration is needed.