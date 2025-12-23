within UMichControls.AircraftPitch;
model AircraftPitch_System "Aircraft pitch control system"
  "An aircraft autopilot system that controls the pitch angle using elevator deflection.
   Based on linearized longitudinal dynamics from Boeing commercial aircraft data.
   Demonstrates control design for aerospace applications with multiple state variables."
  
  import Modelica.SIunits;
  
  // State variables
  Real alpha(start = 0.0) "Angle of attack (rad)";
  Real q(start = 0.0) "Pitch rate (rad/s)";
  SIunits.Angle theta(start = 0.0) "Pitch angle (rad)";
  
  // Input
  Real delta "Elevator deflection angle (rad)";
  
  // Output
  SIunits.Angle y = theta "System output (pitch angle)";
  
equation
  // Linearized longitudinal equations of motion
  // dalpha/dt = -0.313*alpha + 56.7*q + 0.232*delta
  der(alpha) = -0.313*alpha + 56.7*q + 0.232*delta;
  
  // dq/dt = -0.0139*alpha - 0.426*q + 0.0203*delta
  der(q) = -0.0139*alpha - 0.426*q + 0.0203*delta;
  
  // dtheta/dt = 56.7*q
  der(theta) = 56.7*q;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents an aircraft pitch control system based on linearized
    longitudinal dynamics. The system uses elevator deflection to control pitch angle.</p>
    <p>The state-space representation has three state variables:</p>
    <ul>
      <li>alpha: Angle of attack</li>
      <li>q: Pitch rate</li>
      <li>theta: Pitch angle</li>
    </ul>
    <p>The input is the elevator deflection angle delta, and the output is the pitch angle theta.</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="Aircraft")
    }));
end AircraftPitch_System;

