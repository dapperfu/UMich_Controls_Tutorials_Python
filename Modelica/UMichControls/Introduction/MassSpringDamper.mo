within UMichControls.Introduction;
model MassSpringDamper "Mass-spring-damper system model"
  "A simple mechanical system with mass, spring, and damper.
   The system is governed by the second-order differential equation:
   m*ddx + b*dx + k*x = F(t)
   
   State variables: position x and velocity dx"
  
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Mass m = 1.0 "Mass of the system (kg)";
  parameter SIunits.TranslationalSpringConstant k = 1.0 "Spring constant (N/m)";
  parameter SIunits.TranslationalDampingConstant b = 0.2 "Damping constant (N.s/m)";
  
  // State variables
  SIunits.Position x(start = 0.0) "Position of mass (m)";
  SIunits.Velocity dx(start = 0.0) "Velocity of mass (m/s)";
  
  // Input
  SIunits.Force F "External force input (N)";
  
  // Output
  SIunits.Position y = x "System output (position)";
  
equation
  // State equations
  der(x) = dx;
  der(dx) = (F - b*dx - k*x) / m;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents a mass-spring-damper system, which is a fundamental
    example in control systems. The system consists of:</p>
    <ul>
      <li>A mass m that stores kinetic energy</li>
      <li>A spring with constant k that stores potential energy</li>
      <li>A damper with constant b that dissipates energy</li>
    </ul>
    <p>The governing equation is:</p>
    <p>m*ddx + b*dx + k*x = F(t)</p>
    <p>where F(t) is the external force input.</p>
    <p>The state-space representation uses position x and velocity dx as state variables.</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="MSD")
    }));
end MassSpringDamper;

