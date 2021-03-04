%% Inverted Pendulum: Frequency Domain Methods for Controller Design
%
% Key MATLAB commands used in this tutorial are:
% <http://www.mathworks.com/help/toolbox/control/ref/tf.html |tf|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/zpkdata.html |zpkdata|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/sisotool.html |sisotool|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/feedback.html |feedback|> ,
% <http://www.mathworks.com/help/toolbox/control/ref/impulse.html |impulse|>
%
%%
% In this page we will design a controller for the inverted pendulum
% system using a frequency response design method. In the design process we will assume a
% single-input, single-output plant as described by the following transfer
% function. Otherwise stated, we will attempt to control the pendulum's
% angle without regard for the cart's position.
%
% $$P_{pend}(s) = \frac{\Phi(s)}{U(s)}=\frac{\frac{ml}{q}s}{s^3+\frac{b(I+ml^2)}{q}s^2-\frac{(M+m)mgl}{q}s-\frac{bmgl}{q}} \qquad [ \frac{rad}{N}]$$
%
% where,
%
% $$q = (M+m)(I+ml^2) - (ml)^2$$
%
% The controller we are designing will specifically attempt to maintain the
% pendulum vertically upward when the cart is subjected to a 1-Nsec
% impulse. Under these conditions, the design criteria are:
%
% * Settling time of less than 5 seconds
% * Pendulum should not move more than 0.05 radians away from the vertical
%
%%
% For the original problem setup and the derivation of the above transfer function,
% please consult the < ?example=InvertedPendulum&section=SystemModeling
% Inverted Pendulum: System Modeling> page.
%
% *Note:* Applying a frequency response design approach is relatively
% challenging in the case of this example because
% the open-loop system is unstable. That is, the open-loop transfer
% function has a pole in the right-half complex plane. For this reason, attempting
% this example is not recommended if you are just attempting to learn the basics
% of applying frequency response techniques.  This problem is better suited for
% more advanced students who wish to learn about some nuances of the
% frequency response design approach.
%
%% System structure
%
% The structure of the controller for this problem is a little different
% than the standard control problems you may be used to. Since we are
% attempting to control the pendulum's position, which should return to the
% vertical after the initial disturbance, the reference signal we are
% tracking should be zero. This type of situation is often referred to as a
% regulator problem. The external force applied to the cart can be considered as an
% impulsive disturbance. The schematic for this problem is depicted below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/feedback_pend.png>>
%
%%
% You may find it easier to analyze and design for this system if we first
% rearrange the schematic as follows.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/feedback_pend2.png>>
%
%%
% The resulting transfer function $T(s)$ for the closed-loop system from an input of force
% $F$ to an output of pendulum angle $\phi$ is then determined to be the following.
%
% $$ T(s) = \frac{\Phi(s)}{F(s)} = \frac{P_{pend}(s)}{1 + P_{pend}(s)C(s)} $$
%
%%
% Before we begin designing our controller, we first need to define our
% plant within MATLAB. Create a new
% < ?aux=Extras_Mfile m-file> and type in
% the following commands to create the plant model (refer to the main
% problem for the details of getting these commands).

M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

%%
% As mentioned above, this system is unstable without control. We can
% prove this to ourselves by employing the MATLAB command |zpkdata|.
% In this case, |zpkdata| returns the zeros and poles for the transfer
% function. The added parameter |'v'| returns the outputs in the form of
% vectors instead of cell arrays and can only be employed with
% single-input, single-output models. Entering the following code in the
% MATLAB command window generates the output shown below.

[zeros poles] = zpkdata(P_pend,'v')

%% Closed-loop response without compensation
% We will now examine the response of the closed-loop system
% without compensation before we begin to design our controller. In this
% example we will employ the *SISO Design Tool* for examining the various
% analysis plots rather than employing individual commands such as |bode|,
% |nyquist|, and |impulse|. The *SISO Design Tool* is an interactive tool
% with graphical user interface (GUI) which can be launched by the MATLAB
% command |sisotool| as shown below.
%
% <html>
% <pre class="codeinput">
% sisotool('bode',P_pend)
% </pre>
% </html>
%
% The additional parameter |'bode'| opens the *SISO Design for SISO Design
% Task* window with the bode plot of the system $\mathrm{P_{pend}}(s)$ (which was
% passed to the function) as shown below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure1.png>>
%
% We can then modify the system architecture being
% employed to reflect the fact that our controller $C(s)$ is in the feedback path
% of our system as discussed above. This is accomplished from within the
% *Control and Estimation Tools Manager* window by clicking on the tab
% labeled *Architecture*. Then click on the *Control Architecture* button
% and modify the default configuration to match the form shown below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure11.png>>
%
% Next we will begin to examine some of the analysis plots for our system.
% Recall that we can assess the closed-loop stability of our system based
% on the open-loop frequency response. In the case of this example, the
% open-loop transfer function $L(s)$ of our system is given by the
% following.
%
% $$ L(s) = P_{pend}(s)C(s) $$
%
% Note, this is true even though the controller is in the feedback path of the
% system.
%
% In other examples we have specifically employed a Bode plot
% representation of the open-loop frequency response. For our system
% without compensation, we could employ the MATLAB code |bode(P_Pend)| to
% generate this Bode plot. Instead, we will use the *SISO Design Tool* that
% we are employing currently. The open-loop Bode plot of our system is already
% open, but if it weren't, or if we wished to change the type of plot we
% are employing for design, we could open a new plot from under the *Graphical
% Tuning* tab of the *Control and Estimation Tools Manager* window as shown below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure17.png>>
%
% Examination of the above Bode plot shows that the magnitude is less than
% 0 dB and the phase is greater than -180 degrees for all frequencies. For a
% minimum-phase system this would indicate that the closed-loop system is
% stable with infinite gain margin and infinite phase margin. However,
% since our system has a pole in the right-half complex plane, our system is
% nonminimum phase and the closed-loop system is actually unstable. We will
% prove this to ourselves by examining a couple of other analysis plots.
%
% In general, when dealing with nonminimum-phase systems it is preferrable
% to analyze relative stability using the Nyquist plot of the open-loop
% transfer function. The Nyquist plot is also preferred when analyzing
% higher-order systems. This is because the Bode plot shows frequencies
% that are 360 degrees apart as being different when in fact they are the
% same. Since the Nyquist plot is a polar-type plot, this ambiguity is
% removed.
%
% In order to generate additional plots to better understand the
% closed-loop performance of the system, click on the *Analysis Plots* tab
% in the *Control and Estimation Tools Manager* window. The *SISO Design
% Tool* allows the user to view up to six plots at the same time
% for analysis.  These plots can be viewed for a number of options, such as
% open-loop and closed-loop. We will view the Nyquist plot for the
% open-loop system and the impulse response of the closed-loop system by
% following the steps given below.
%
% 1. Under the *Analysis Plots* tab, select a *Plot Type* of |Nyquist| for
% *Plot 1*. A plot window with a blank Nyquist plot should appear.  Make
% sure that the *Real-Time Update* box is checked in the bottom right corner of
% the plot window.
%
% 2. Select a *Plot Type* of |Impulse| for *Plot 2*.
%
% 3. In the *Contents of Plots* section of the window, check the square for *Open
% Loop L* for plot 1 and the square for *Closed Loop r to y* for plot 2.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure18.png>>
%
% Then click the *Show Analysis Plot* button to generate the figure shown below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure3.png>>
%
% Examination of the above impulse response plot shows that the closed-loop
% system is unstable. This can also be verified from the open-loop
% Nyquist plot by applying the Nyquist stability criterion which is stated
% below.
%
% $$ Z = P + N $$
%
% Where $Z$ is the number of closed-loop poles in the right-half plane, $P$
% is the number of open-loop poles in the right-half plane, and $N$ is the
% number of clockwise encirclements of the point -1 by the open-loop
% Nyquist plot.
%
% From our previous discussion we know that our system has one open-loop
% pole in the right-half plane ($P$ = 1) and from examination of the
% open-loop Nyquist plot above we can see that there are no encirclements
% of the point -1 ($N$ = 0). Therefore, $Z$ = 0 + 1 = 1 and the closed-loop
% system has 1 pole in the right-half plane indicating that it is
% indeed unstable.
%
%% Closed-loop response with compensation
% Since the closed-loop system is unstable without compensation, we need to
% use our controller to stabilize the system and meet the given
% requirements. Our first step will be to add an integrator to cancel the
% zero at the origin. To add an integrator, you may right-click on the Bode plot
% that is already open and choose *Add Pole/Zero > Integrator* from the
% resulting menu. The result is a figure like the one shown below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure6.png>>
%
% Examination of the above shows that at small frequencies the phase plot
% goes to -270 degrees when in fact it should remain at -180 degrees. This
% behavior arises because of numerical errors associated with the pole/zero
% cancellation at the origin. To correct this error, exit out of the *SISO
% Design Tool* and re-open the tool with the integrator already added to
% the plant as shown below.
%
% <html>
% <pre class="codeinput">
% sisotool('bode',P_pend*(1/s))
% </pre>
% </html>
%
% Since the integrator is bundled with the plant which is in the forward
% path, while our controller is actually in the feedback path, we will not
% analyze the closed-loop response of this system from within the *SISO
% Design Tool*. However, the open-loop transfer function is unchanged by
% whether the controller is in the forward or feedback path, therefore, we
% can still use the plots of the open-loop system for analysis and design.
% The resulting bode plot that is generated is shown below and reflects
% the low frequency behavior we would expect.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure11b.png>>
%
% Even with the addition of this integrator, the closed-loop system is
% still unstable. We can attempt to better understand
% the instability (and how to resolve it) by looking more
% closely at the Nyquist plot. We can generate this analysis plot in the
% same manner we did previously. Instead, we can also generate this plot
% by selecting *Open-Loop Nyquist* from the *Analysis* menu accessed from
% the top of the window containing our Bode plot. Following these steps
% generates a figure like the one given below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure12.png>>
%
% Notice that the open-loop Nyquist plot now encircles the -1 point in the
% clockwise direction. This means that the closed-loop system now has two
% poles in the right-half plane ($Z = P + N = 1 + 1 = 2$). Hence, the
% closed-loop system is still unstable. We need to add phase in order to
% get a counterclockwise encirclement. We will do this by adding a zero to
% our controller. For starters, we will place this zero at -1 and view the
% resulting plots. This action can be achieved graphically by
% right-clicking on the Bode plot as we did previously. Instead, we will
% add the zero from the *Compensator Editor* tab of the *Control and
% Estimation Tools Manager* window. Right-click in the *Dynamics* section
% of the window and select *Add Pole/Zero > Real Zero* from the resulting
% menu. By default, the location of the resulting zero is -1. The resulting
% window should appear as shown in the figure below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure7.png>>
%
% This additional zero will change the Bode plot and Nyquist plots that are
% open as long as the *Real-time Update* box is checked. The resulting
% Nyquist plot should appear as shown below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure9.png>>
%
% As you can see, this change did not provide enough phase.  The
% encirclement around -1 is still clockwise.  We will try adding a second
% zero at -1 in the same manner as was described above. The resulting
% Nyquist diagram is shown below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure10.png>>
%
% We still have one clockwise encirclement of the -1 point. However, if we
% add some gain we can increase the magnitude of each point of the Nyquist
% plot, thereby increasing the radius of the counterclockwise circle such
% that it encircles the -1 point. This results in $N$ = -1 where $N$ is negative
% because the encirclement is counterclockwise. To achieve this, you can
% manually enter a new gain value in the *Compensator Editor* tab of the
% *Control and Estimation Tools Manager* window. Alternatively, you can
% modify the gain graphically from the Bode plot. Specifically, go to the
% Bode plot window, click on the magnitude plot, and drag the curve up
% until you have shifted the Nyquist plot far enough to the left that the
% counterclockwise circle encompasses the point -1. Note that this is
% achieved for a gain value of approximately 3.8. We will continue to
% increase the gain to a magnitude of approximately 10 as indicated at the
% bottom of the Bode plot window. The resulting Bode plot should appear as
% in the figure below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure13.png>>
%
% The corresponding Nyquist plot should then appear as in the figure below.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/Figure19.png>>
%
% From our previous discussion we know that $P$ = 1, now that $N$ = -1, we
% have $Z$ = -1 + 1 = 0 closed-loop poles in the right-half plan indicating
% that our closed-loop system is stable. We can verify the stability of our
% system and determine whether or not the other requirements are met by
% examining the system's response to a unit impulse force disturbance.
% Since the integrator of our controller is currently bundled with the
% plant, we will exit from the *SISO Design Tool* and generate the
% closed-loop impulse response form the command line. So far, the
% controller we have designed has the form given below.
%
% $$ C(s) = 10\frac{(s+1)^2}{s} $$
%
% Adding the following code to your m-file will construct the closed-loop
% transfer function from an input of $F(s)$ to an output of $\Phi(s)$.
% Running your m-file at the command line will then generate an impulse
% response plot as shown below.

K = 10;
C = K*(s+1)^2/s;
T = feedback(P_pend,C);
t = 0:0.01:10;
impulse(T,t), grid
title('Response of Pendulum Position to an Impulse Disturbance under Closed-loop Control');

%%
% From examination of the figure above, it is apparent that the response of
% the system is now stable. However, the pendulum position overshoots
% past the required limit of 0.05 radians and the settle time is on the
% verge of being greater than the requirement of 5 seconds. Therefore, we
% will now concentrate on improving the response. We can use the *SISO
% Design Tool* to see how changing the controller gain and the location of
% the zeros affects the system's frequency response plots. In this case,
% increasing the controller gain increases the system's phase margin
% which should help reduce the overshoot of the response. Furthermore,
% trial and error shows that moving one of the zeros farther to the left in
% the complex plane (more negative) makes the response faster. This change
% also increases the overshoot, but this is offset by the increase in gain.
% Experimentation demonstrates that the following controller satisfies the
% given requirements.
%
% $$ C(s) = 35\frac{(s+1)(s+2)}{s} $$
%
% Modify your m-file as shown and re-run at the command line
% to generate the impulse response plot given below.

K = 35;
C = K*(s+1)*(s+2)/s;
T = feedback(P_pend,C);
t = 0:0.01:10;
impulse(T, t), grid
title('Response of Pendulum Position to an Impulse Disturbance under Closed-loop Control');

%%
% Our response has met our design goals.  Feel free to vary the parameters
% further to observe what happens. Note that it also possible to generate
% the system's response to an impulse disturbance from within the *SISO
% Design Tool*. Including the integrator with the controller, rather than
% with the plant, as is proper will result in the numerical abberation
% observed above, but will not significantly affect the impulse response
% generated by MATLAB.
%
%% What happens to the cart's position?
% At the beginning of this page, a block diagram for the inverted pendulum
% system was given. The diagram was not entirely complete. The block
% representing the response of the cart's position $x$ was not included
% because that variable is not being controlled. It is interesting though,
% to see what is happening to the cart's position when the controller for
% the pendulum's angle $\phi$ is in place. To see this we need to consider
% the full system block diagram as shown in the following figure.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/feedback_pend3.png>>
%
% Rearranging, we get the following block diagram.
%
% <<Content/InvertedPendulum/Control/Frequency/figures/feedback_pend4.png>>
%
% In the above, the block $C(s)$ is the controller designed for maintaining
% the pendulum vertical. The closed-loop transfer function $T_2(s)$ from an input force applied
% to the cart to an output of cart position is, therefore, given by the
% following.
%
% $$ T_2(s) = \frac{X(s)}{F(s)} = \frac{P_{cart}(s)}{1 + P_{pend}(s)C(s)}$$
%
%%
% Referring to the < ?example=InvertedPendulum&section=SystemModeling
% Inverted Pendulum: System Modeling> page, the transfer function for
% $\mathrm{P_{cart}}(s)$ is defined as follows.
%
% $$P_{cart}(s) = \frac{X(s)}{U(s)} = \frac{ \frac{ (I+ml^2)s^2 - gml } {q}
% }{s^4+\frac{b(I+ml^2)}{q}s^3-\frac{(M+m)mgl}{q}s^2-\frac{bmgl}{q}s}
% \qquad [ \frac{m}{N}] $$
%
% where,
%
% $$q=[(M+m)(I+ml^2)-(ml)^2]$$
%
%%
% Adding the following commands to your m-file (presuming $\mathrm{P_{pend}}(s)$ and
% $C(s)$ are still defined) will generate the response of the cart's
% position to the same impulsive disturbance we have been considering.

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
T2 = feedback(1,P_pend*C)*P_cart;
T2 = minreal(T2);
t = 0:0.01:10;
impulse(T2, t), grid
title('Response of Cart Position to an Impulse Disturbance under Closed-loop Control');

%%
% The command |minreal| effectively cancels out all common poles and
% zeros in the closed-loop transfer function. This gives the |impulse|
% function better numerical properties.  As you can see, the cart moves in
% the negative direction and stabilizes at about -0.14 meters.  This design
% might work pretty well for the actual controller, assuming that the cart
% had that much room to move.  Keep in mind that this was pure luck.  We
% did not design our controller to stabilize the cart's position, the fact
% that we have is a fortunate side effect.
