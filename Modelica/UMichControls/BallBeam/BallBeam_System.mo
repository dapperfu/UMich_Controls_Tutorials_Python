within UMichControls.BallBeam;
model BallBeam_System "Ball and beam position control system"
  "A ball position control system where a ball rolls along a beam, and the beam
   angle is controlled by a servo motor. This system features a double integrator
   plant (marginally stable) and demonstrates control of systems with challenging dynamics."
  
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Mass m = 0.111 "Mass of the ball (kg)";
  parameter SIunits.Length R = 0.015 "Radius of the ball (m)";
  parameter SIunits.MomentOfInertia J = 9.99e-06 "Moment of inertia of the ball (kg.m^2)";
  parameter SIunits.Acceleration g = 9.81 "Gravitational acceleration (m/s^2)";
  parameter Real d = 0.03 "Lever arm offset (m)";
  parameter SIunits.Length L = 0.4 "Length of the beam (m)";
  
  // State variables
  SIunits.Position r(start = 0.0) "Ball position along beam (m)";
  SIunits.Velocity dr(start = 0.0) "Ball velocity (m/s)";
  SIunits.Angle alpha(start = 0.0) "Beam angle (rad)";
  SIunits.Angle theta(start = 0.0) "Servo gear angle (rad)";
  
  // Input
  SIunits.Angle theta_input "Servo gear angle input (rad)";
  
  // Output
  SIunits.Position y = r "System output (ball position)";
  
equation
  // State equations
  der(r) = dr;
  der(alpha) = (d/L)*der(theta);
  
  // Linearized equation of motion: (J/R^2 + m)*ddr = -m*g*alpha
  // Simplified: alpha = (d/L)*theta
  (J/(R*R) + m)*der(dr) = -m*g*alpha;
  
  // Servo relationship
  theta = theta_input;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents a ball and beam position control system. A ball is
    placed on a beam where it is allowed to roll with 1 degree of freedom along
    the length of the beam. A lever arm is attached to the beam at one end and
    a servo gear at the other.</p>
    <p>The system features a double integrator plant (marginally stable) and
    requires careful controller design to achieve stability.</p>
    <p>The linearized equation of motion is:</p>
    <p>(J/R^2 + m)*ddr = -m*g*alpha</p>
    <p>where alpha = (d/L)*theta</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="BallBeam")
    }));
end BallBeam_System;

