within UMichControls.InvertedPendulum;
model InvertedPendulum_System "Inverted pendulum on cart system"
  "A classic control problem involving an inverted pendulum mounted on a motorized cart.
   This unstable, nonlinear system requires active control to maintain the pendulum
   in an upright position. The problem is analogous to attitude control of a booster
   rocket at takeoff."
  
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Mass M = 0.5 "Mass of the cart (kg)";
  parameter SIunits.Mass m = 0.2 "Mass of the pendulum (kg)";
  parameter SIunits.TranslationalDampingConstant b = 0.1 "Coefficient of friction for cart (N/m/sec)";
  parameter SIunits.Length l = 0.3 "Length to pendulum center of mass (m)";
  parameter SIunits.MomentOfInertia I = 0.006 "Mass moment of inertia of the pendulum (kg.m^2)";
  parameter SIunits.Acceleration g = 9.81 "Gravitational acceleration (m/s^2)";
  
  // State variables
  SIunits.Position x(start = 0.0) "Cart position (m)";
  SIunits.Velocity dx(start = 0.0) "Cart velocity (m/s)";
  SIunits.Angle theta(start = 3.14159) "Pendulum angle from vertical (rad)";
  SIunits.AngularVelocity dtheta(start = 0.0) "Pendulum angular velocity (rad/s)";
  
  // Input
  SIunits.Force F "Force applied to the cart (N)";
  
  // Outputs
  SIunits.Angle y1 = theta "Pendulum angle output (rad)";
  SIunits.Position y2 = x "Cart position output (m)";
  
equation
  // State equations (linearized around theta = pi)
  der(x) = dx;
  der(theta) = dtheta;
  
  // Linearized equations of motion (simplified for Modelica)
  // Full nonlinear equations would be more complex
  (M + m)*der(dx) + m*l*cos(theta)*der(dtheta) - m*l*sin(theta)*dtheta^2 = F - b*dx;
  (I + m*l^2)*der(dtheta) + m*l*cos(theta)*der(dx) = m*g*l*sin(theta);
  
  annotation(
    Documentation(info="<html>
    <p>This model represents an inverted pendulum on a cart system. The system
    is unstable and nonlinear, requiring active control to maintain the pendulum
    in an upright position.</p>
    <p>The system has four state variables: cart position, cart velocity,
    pendulum angle, and pendulum angular velocity.</p>
    <p>Note: This is a simplified linearized model. For full nonlinear dynamics,
    the equations would include more complex coupling terms.</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="InvPend")
    }));
end InvertedPendulum_System;

