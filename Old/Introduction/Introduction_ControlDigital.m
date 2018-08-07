%% Introduction: Digital Controller Design
%
% In this section we will discuss converting continuous time models into
% discrete time (or difference equation) models. We will also introduce
% the z-transform and show how to use it to analyze and design controllers 
% for discrete time systems.
%
% Key MATLAB commands used in this tutorial are:
% <http://www.mathworks.com/help/toolbox/control/ref/c2d.html |c2d|> , 
% <http://www.mathworks.com/help/toolbox/control/ref/pzmap.html |pzmap|> , 
% <http://www.mathworks.com/help/toolbox/control/ref/zgrid.html |zgrid|> , 
% <http://www.mathworks.com/help/toolbox/control/ref/step.html |step|> , 
% <http://www.mathworks.com/help/toolbox/control/ref/rlocus.html |rlocus|>
%
%% Introduction
%
% The figure below shows the typical continuous feedback system that we
% have been considering so far in this tutorial. Almost all of the continuous
% controllers can be built using analog electronics. 
%
% <<Content/Introduction/Control/Digital/figures/block1.png>>
%
% The continuous controller, enclosed in the dashed square, can be replaced
% by a digital controller, shown below, that performs same control task as 
% the continuous controller. The basic difference between these controllers 
% is that the digital system operates on discrete signals (or samples of the 
% sensed signal) rather than on continuous signals. 
% 
% <<Content/Introduction/Control/Digital/figures/block2.png>>
%
% Different types of signals in the above digital schematic can be
% represented by the following plots. 
%
% <<Content/Introduction/Control/Digital/figures/signals.png>>
%
% The purpose of this Digital Control Tutorial is to show you how to use
% MATLAB to work with discrete functions either in transfer function or 
% state-space form to design digital control systems. 
%
%% Zero-Hold Equivalence
%
% In the above schematic of the digital control system, we see that the
% digital control system contains both discrete and the continuous portions. 
% When designing a digital control system, we need to find the discrete 
% equivalent of the continuous portion so that we only need to deal with 
% discrete functions. 
%
% For this technique, we will consider the following portion of the digital
% control system and rearrange as follows. 
%
% <<Content/Introduction/Control/Digital/figures/block3.png>>
%
% <<Content/Introduction/Control/Digital/figures/block4.png>>
%
% The *clock* connected to the *D/A and A/D converters* supplies a pulse every
% T seconds and each D/A and A/D sends a signal only when the pulse arrives. 
% The purpose of having this pulse is to require that Hzoh(z) have only 
% samples u(k) to work on and produce only samples of output y(k); thus, 
% Hzoh(z) can be realized as a discrete function. 
%
% The philosophy of the design is the following. We want to find a discrete
% function Hzoh(z) so that for a piecewise constant input to the continuous 
% system H(s), the sampled output of the continuous system equals the discrete 
% output. Suppose the signal u(k) represents a sample of the input signal. 
% There are techniques for taking this sample u(k) and holding it to produce 
% a continuous signal uhat(t). The sketch below shows that the uhat(t) is 
% held constant at u(k) over the interval kT to (k+1)T. This operation of 
% holding uhat(t) constant over the sampling time is called zero-order
% hold. 
%
% <<Content/Introduction/Control/Digital/figures/zohfig.png>>
%
% The zero-order held signal uhat(t) goes through H2(s) and A/D to produce
% the output y(k) that will be the piecewise same signal as if the discrete 
% signal u(k) goes through Hzoh(z) to produce the discrete output y(k). 
%
% <<Content/Introduction/Control/Digital/figures/block7.png>>
%
% Now we will redraw the schematic, placing Hzoh(z) in place of the
% continuous portion.
%
% <<Content/Introduction/Control/Digital/figures/block6.png>>
%
% By placing Hzoh(z), we can design digital control systems dealing with
% only discrete functions. 
%
% *Note:* There are certain cases where the discrete response does not
% match the continuous response due to a hold circuit implemented in digital 
% control systems. For information, see Lagging effect associated with the hold.
%
%% Conversion Using c2d
%
% There is a MATLAB function called c2d that converts a given continuous
% system (either in transfer function or state-space form) to a discrete 
% system using the zero-order hold operation explained above. The basic 
% command for this in MATLAB is |sys_d = c2d(sys,Ts,'zoh')|
%
% The sampling time (Ts in sec/sample) should be smaller than 1/(30*BW),
% where BW is the closed-loop bandwidth frequency. 
%
%% Example: Mass-Spring-Damper
%
% *Transfer Function*
% 
% Suppose you have the following continuous transfer function 
%
% $$
% \frac{X(s)}{F(s)} = \frac{1}{Ms^2+bs+k}
% $$
%
% Assuming the closed-loop bandwidth frequency is greater than 1 rad/sec,
% we will choose the sampling time (Ts) equal to 1/100 sec. Now, create an 
% new m-file and enter the following commands. 
%
%%

M = 1;
b = 10;
k = 20;

s = tf('s');
sys = 1/(M*s^2+b*s+k);

Ts = 1/100;
sys_d = c2d(sys,Ts,'zoh')

%%
% *State-Space*
%
% The continuous time state-space model is as follows:
%
% $$
% \mathbf{\dot{x}} = \left[ \begin{array}{c} \dot{x} \\ \ddot{x} \end{array} \right] = \left[ \begin{array}{cc} 0 & 1 \\ -\frac{k}{m}  & -\frac{b}{m} \end{array} \right] \left[ \begin{array}{c} x \\ \dot{x} \end{array} \right] + \left[ \begin{array}{c} 0 \\ \frac{1}{m} \end{array} \right] F(t)
% $$
%
% $$
% y = \left[ \begin{array}{cc} 1 & 0 \end{array} \right] \left[
% \begin{array}{c} x \\ \dot{x} \end{array} \right]
% $$
%
% All constants are the same as before.  The following m-file converts the
% above continuous state-space to discrete state-space. 
%
%%

A = [0       1;  
    -k/M   -b/M];
  
B = [  0;
    1/M];   
     
C = [1 0];

D = [0];
     
Ts = 1/100;

sys = ss(A,B,C,D);
sys_d = c2d(sys,Ts,'zoh')

%%
%
% From these matrices, the discrete state-space can be written as 
%
% $$ 
% \left[ \begin{array}{c} x(k) \\ v(k) \end{array} \right] = 
% \left[ \begin{array}{cc} 0.9990 & 0.0095 \\ -0.1903 & 0.9039  \end{array} \right] \left[ \begin{array}{c} x(k-1) \\ v(k-1) \end{array} \right] +
% \left[ \begin{array}{c} 0 \\ 0.0095 \end{array} \right] F(k-1)
% $$
%
% $$
% y(k-1) = \left[ \begin{array}{cc} 1 & 0 \end{array} \right] \left[ \begin{array}{c} x(k-1) \\ v(k-1) \end{array} \right]
% $$
%
% Now you have the discrete time state-space model. 
%
%% Stability and Transient Response
%
% For continuous systems, we know that certain behaviors results from
% different pole locations in the s-plane. For instance, a system is 
% unstable when any pole is located to the right of the imaginary axis. 
% For discrete systems, we can analyze the system behaviors from different 
% pole locations in the z-plane. The characteristics in the z-plane can be 
% related to those in the s-plane by the expression 
%
% $$
% z = e^{sT}
% $$
%
% * T = Sampling time (sec/sample)
% * s = Location in the s-plane
% * z = Location in the z-plane 
%
% The figure below shows the mapping of lines of constant damping ratio
% (zeta) and natural frequency (Wn) from the s-plane to the z-plane using 
% the expression shown above. 
%
% <<Content/Introduction/Control/Digital/figures/zgrid.gif>>
%
% If you noticed in the z-plane, the stability boundary is no longer
% imaginary axis, but is the unit circle |z|=1. The system is stable when 
% all poles are located inside the unit circle and unstable when any pole 
% is located outside. 
%
% For analyzing the transient response from pole locations in the z-plane,
% the following three equations used in continuous system designs are still 
% applicable. 
%
% $$
% \zeta \omega_n \geq \frac{4.6}{Ts}
% $$
%
% $$
% \omega_n \geq \frac{1.8}{Tr}
% $$
%
% $$
% \zeta = \frac{-\ln(\%OS/100)}{\sqrt{\pi^2+\ln(\%OS/100)^2}}
% $$
%
% where,
% 
% * zeta = Damping ratio
% * Wn = Natural frequency (rad/sec)
% * Ts = Settling time
% * Tr = Rise time
% * Mp = Maximum overshoot 
%
% Important: The natural frequency (Wn) in z-plane has the unit of
% rad/sample, but when you use the equations shown above, the Wn must be 
% in the unit of rad/sec. 
%
% Suppose we have the following discrete transfer function 
%
% $$
% \frac{Y(z)}{F(z)} = \frac{1}{z^2-0.3z+0.5}
% $$
%
% Create an new m-file and enter the following commands. Running this
% m-file in the command window gives you the following plot with the lines 
% of constant damping ratio and natural frequency. 
%
%%

numDz = 1;
denDz = [1 -0.3 0.5];
sys = tf(numDz,denDz,-1); % the -1 indicates that the sample time is undetermined

pzmap(sys)
axis([-1 1 -1 1])
zgrid

%%
%
% From this plot, we see poles are located approximately at the natural
% frequency of 9pi/20T (rad/sample) and the damping ratio of 0.25. Assuming 
% that we have a sampling time of 1/20 sec (which leads to Wn = 28.2 rad/sec) 
% and using three equations shown above, we can determine that this system 
% should have the rise time of 0.06 sec, a settling time of 0.65 sec and a 
% maximum overshoot of 45% (0.45 more than the steady-state value). Let's 
% obtain the step response and see if these are correct. Add the following 
% commands to the above m-file and rerun it in the command window. You should 
% get the following step response. 
%
%%

sys = tf(numDz,denDz,1/20);
step(sys,2.5);

%%
%
% As you can see from the plot, the rise time, settling time and overshoot
% came out to be what we expected. This shows how you can use the locations 
% of poles and the above three equations to analyze the transient response 
% of the system. 
%
%% Discrete Root Locus
%
% The root-locus is the locus of points where roots of characteristic
% equation can be found as a single gain is varied from zero to infinity. 
% The characteristic equation of an unity feedback system is 
%
% $$
% 1+KG(z)Hzoh(z) = 0
% $$
%
% where G(z) is the compensator implemented in the digital controller and
% Hzoh(z) is the plant transfer function in z. 
%
% The mechanics of drawing the root-loci are exactly the same in the
% z-plane as in the s-plane. Recall from the continuous Root-Locus Tutorial, 
% we used the MATLAB function called sgrid to find the root-locus region that 
% gives an acceptable gain (K). For the discrete root-locus analysis, we will
% use the function zgrid that has the same characteristics as sgrid. The 
% command zgrid(zeta, Wn) draws lines of constant damping ratio (zeta) and 
% natural frequency (Wn). 
% 
% Suppose we have the following discrete transfer function 
%
% $$
% \frac{Y(z)}{F(z)} = \frac{z-0.3}{z^2-1.6z+0.7}
% $$
%
% and the requirements are a damping ratio greater than 0.6 and a natural
% frequency greater than 0.4 rad/sample (these can be found from design 
% requirements, sampling time (sec/sample) and three equations shown in the
% previous section). The following commands draw the root-locus with the 
% lines of constant damping ratio and natural frequency. Create an new 
% m-file and enter the following commands. Running this m-file should give 
% you the following root-locus plot. 
%
%%

numDz = [1 -0.3];
denDz = [1 -1.6 0.7];
sys = tf(numDz,denDz,-1);

rlocus(sys)
axis([-1 1 -1 1])

zeta = 0.4;
Wn = 0.3;
zgrid(zeta,Wn)

%%
%
% From this plot, you should realize that the system is stable because all
% poles are located inside the unit circle. Also, you see two dotted lines 
% of constant damping ratio and natural frequency. The natural frequency is 
% greater than 0.3 outside the constant-Wn line, and the damping ratio is 
% greater than 0.4 inside the constant-zeta line. In this example, we do 
% have the root-locus drawn in the desired region. Therefore, a gain (K) 
% chosen from one of the loci in the desired region should give you the 
% response that satisfies design requirements.