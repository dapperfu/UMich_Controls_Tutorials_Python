within UMichControls.CruiseControl;
model CruiseControl_System "Cruise control system: vehicle speed control"
  "A first-order system representing vehicle speed control.
   The system equation is: m*dv/dt + b*v = u
   where m is vehicle mass, b is damping coefficient, and u is control force.
   
   State variable: velocity v"
  
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Mass m = 1000 "Vehicle mass (kg)";
  parameter SIunits.TranslationalDampingConstant b = 50 "Damping coefficient (N.s/m)";
  
  // State variable
  SIunits.Velocity v(start = 0.0) "Vehicle velocity (m/s)";
  
  // Input
  SIunits.Force u "Control force input (N)";
  
  // Output
  SIunits.Velocity y = v "System output (velocity)";
  
equation
  // State-space equation: m*dv/dt + b*v = u
  // Rearranged: dv/dt = (u - b*v) / m
  m*der(v) + b*v = u;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents a cruise control system for maintaining constant
    vehicle speed. The system is a first-order mass-damper system where:</p>
    <ul>
      <li>m is the vehicle mass (1000 kg)</li>
      <li>b is the damping coefficient representing rolling resistance and wind drag (50 N.s/m)</li>
      <li>u is the control force at the road/tire interface (N)</li>
      <li>v is the vehicle velocity (m/s)</li>
    </ul>
    <p>The governing equation is:</p>
    <p>m*dv/dt + b*v = u</p>
    <p>In state-space form:</p>
    <p>dv/dt = (-b/m)*v + (1/m)*u</p>
    <p>y = v</p>
    <p>This is a first-order system with a single state variable (velocity).</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="Cruise")
    }));
end CruiseControl_System;

