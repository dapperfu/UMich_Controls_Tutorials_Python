within UMichControls.Suspension;
model Suspension_System "Quarter-car active suspension system"
  "A multi-mass, multi-spring-damper system for automotive suspension.
   Demonstrates control design for multi-input, multi-output (MIMO) systems.
   
   Governing equations:
   M1*ddX1 = -b1*(dX1-dX2) - K1*(X1-X2) + U
   M2*ddX2 = b1*(dX1-dX2) + K1*(X1-X2) + b2*(dW-dX2) + K2*(W-X2) - U"
  
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Mass M1 = 2500 "1/4 bus body mass (kg)";
  parameter SIunits.Mass M2 = 320 "Suspension mass (kg)";
  parameter SIunits.TranslationalSpringConstant K1 = 80000 "Spring constant of suspension system (N/m)";
  parameter SIunits.TranslationalSpringConstant K2 = 500000 "Spring constant of wheel and tire (N/m)";
  parameter SIunits.TranslationalDampingConstant b1 = 350 "Damping constant of suspension system (N.s/m)";
  parameter SIunits.TranslationalDampingConstant b2 = 15020 "Damping constant of wheel and tire (N.s/m)";
  
  // State variables
  SIunits.Position X1(start = 0.0) "Body position (m)";
  SIunits.Velocity dX1(start = 0.0) "Body velocity (m/s)";
  SIunits.Position X2(start = 0.0) "Suspension position (m)";
  SIunits.Velocity dX2(start = 0.0) "Suspension velocity (m/s)";
  
  // Inputs
  SIunits.Force U "Control force (N)";
  SIunits.Position W "Road disturbance input (m)";
  SIunits.Velocity dW "Road disturbance velocity (m/s)";
  
  // Outputs
  SIunits.Position y1 = X1 - X2 "Body-suspension relative position (m)";
  SIunits.Position y2 = X1 "Body position output (m)";
  
equation
  // State equations
  der(X1) = dX1;
  der(X2) = dX2;
  
  // Body dynamics: M1*ddX1 = -b1*(dX1-dX2) - K1*(X1-X2) + U
  M1*der(dX1) = -b1*(dX1 - dX2) - K1*(X1 - X2) + U;
  
  // Suspension dynamics: M2*ddX2 = b1*(dX1-dX2) + K1*(X1-X2) + b2*(dW-dX2) + K2*(W-X2) - U
  M2*der(dX2) = b1*(dX1 - dX2) + K1*(X1 - X2) + b2*(dW - dX2) + K2*(W - X2) - U;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents a quarter-car active suspension system. The system
    demonstrates control design for multi-input, multi-output (MIMO) systems
    to minimize body motion when encountering road disturbances.</p>
    <p>The system has two masses (body and suspension) connected by springs and dampers.</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="Susp")
    }));
end Suspension_System;

