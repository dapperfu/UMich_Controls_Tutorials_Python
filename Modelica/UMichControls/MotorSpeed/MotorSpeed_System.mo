within UMichControls.MotorSpeed;
model MotorSpeed_System "DC motor speed control system"
  "A DC motor system combining electrical and mechanical dynamics.
   The system has two state variables: angular velocity and armature current.
   
   Governing equations:
   J*ddtheta + b*dtheta = K*i
   L*di/dt + R*i = V - K*dtheta"
  
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.MomentOfInertia J = 0.01 "Moment of inertia of rotor (kg.m^2)";
  parameter SIunits.RotationalDampingConstant b = 0.1 "Motor viscous friction constant (N.m.s)";
  parameter SIunits.Voltage K = 0.01 "Motor torque constant = back emf constant (N.m/A = V.s/rad)";
  parameter SIunits.Resistance R = 1.0 "Electric resistance (Ohm)";
  parameter SIunits.Inductance L = 0.5 "Electric inductance (H)";
  
  // State variables
  SIunits.AngularVelocity w(start = 0.0) "Angular velocity (rad/s)";
  SIunits.Current i(start = 0.0) "Armature current (A)";
  
  // Input
  SIunits.Voltage V "Input voltage (V)";
  
  // Output
  SIunits.AngularVelocity y = w "System output (angular velocity)";
  
equation
  // Mechanical equation: J*dw/dt + b*w = K*i
  J*der(w) + b*w = K*i;
  
  // Electrical equation: L*di/dt + R*i = V - K*w
  L*der(i) + R*i = V - K*w;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents a DC motor speed control system. The system combines
    electrical dynamics (armature circuit) with mechanical dynamics (rotor inertia
    and friction) to create a second-order system.</p>
    <p>The governing equations are:</p>
    <ul>
      <li>J*dw/dt + b*w = K*i (mechanical equation)</li>
      <li>L*di/dt + R*i = V - K*w (electrical equation)</li>
    </ul>
    <p>where:</p>
    <ul>
      <li>J is the moment of inertia (0.01 kg.m^2)</li>
      <li>b is the viscous friction constant (0.1 N.m.s)</li>
      <li>K is the motor torque/back emf constant (0.01 N.m/A)</li>
      <li>R is the armature resistance (1.0 Ohm)</li>
      <li>L is the armature inductance (0.5 H)</li>
      <li>w is the angular velocity (rad/s)</li>
      <li>i is the armature current (A)</li>
      <li>V is the input voltage (V)</li>
    </ul>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="MotorSpd")
    }));
end MotorSpeed_System;

