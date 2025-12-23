within UMichControls.MotorPosition;
model MotorPosition_System "DC motor position control system"
  "A DC motor system for position control. Similar to motor speed but with
   position as output, requiring an integrator.
   
   Governing equations:
   J*ddtheta + b*dtheta = K*i
   L*di/dt + R*i = V - K*dtheta"
  
  import Modelica.SIunits;
  
  // Parameters (same as MotorSpeed)
  parameter SIunits.MomentOfInertia J = 0.01 "Moment of inertia of rotor (kg.m^2)";
  parameter SIunits.RotationalDampingConstant b = 0.1 "Motor viscous friction constant (N.m.s)";
  parameter SIunits.Voltage K = 0.01 "Motor torque constant = back emf constant (N.m/A = V.s/rad)";
  parameter SIunits.Resistance R = 1.0 "Electric resistance (Ohm)";
  parameter SIunits.Inductance L = 0.5 "Electric inductance (H)";
  
  // State variables
  SIunits.Angle theta(start = 0.0) "Angular position (rad)";
  SIunits.AngularVelocity w(start = 0.0) "Angular velocity (rad/s)";
  SIunits.Current i(start = 0.0) "Armature current (A)";
  
  // Input
  SIunits.Voltage V "Input voltage (V)";
  
  // Output
  SIunits.Angle y = theta "System output (angular position)";
  
equation
  // Position-velocity relationship
  der(theta) = w;
  
  // Mechanical equation: J*dw/dt + b*w = K*i
  J*der(w) + b*w = K*i;
  
  // Electrical equation: L*di/dt + R*i = V - K*w
  L*der(i) + R*i = V - K*w;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents a DC motor position control system. Similar to the
    motor speed system, but with position as the output, requiring an integrator
    and demonstrating control of systems with higher-order dynamics.</p>
    <p>The system has three state variables: position, velocity, and current.</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="MotorPos")
    }));
end MotorPosition_System;

