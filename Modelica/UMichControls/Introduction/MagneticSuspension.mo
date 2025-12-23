within UMichControls.Introduction;
model MagneticSuspension "Magnetically suspended ball system"
  "A magnetic suspension system where a ball is suspended by an electromagnet.
   The system is nonlinear but can be linearized around an equilibrium point.
   
   State variables: height h, velocity dh, and current i"
  
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Mass m = 0.05 "Mass of the ball (kg)";
  parameter Real K = 0.0001 "Magnetic force coefficient";
  parameter SIunits.Inductance L = 0.01 "Inductance (H)";
  parameter SIunits.Resistance R = 1.0 "Resistance (Ohm)";
  parameter SIunits.Acceleration g = 9.81 "Gravitational acceleration (m/s^2)";
  
  // Equilibrium point
  parameter SIunits.Length h_eq = 0.01 "Equilibrium height (m)";
  parameter SIunits.Current i_eq = sqrt(m*g*h_eq/K) "Equilibrium current (A)";
  
  // State variables (deviations from equilibrium)
  SIunits.Length delta_h(start = 0.0) "Deviation of height from equilibrium (m)";
  SIunits.Velocity delta_dh(start = 0.0) "Deviation of velocity (m/s)";
  SIunits.Current delta_i(start = 0.0) "Deviation of current from equilibrium (A)";
  
  // Input (deviation from equilibrium voltage)
  SIunits.Voltage delta_V "Deviation of input voltage from equilibrium (V)";
  
  // Output
  SIunits.Length y = delta_h "System output (height deviation)";
  
  // Linearized system matrices (from linearization around equilibrium)
  // A = [[0, 1, 0], [980, 0, -2.8], [0, 0, -100]]
  // B = [[0], [0], [100]]
  // C = [1, 0, 0]
  
equation
  // Linearized state-space equations around equilibrium point
  der(delta_h) = delta_dh;
  der(delta_dh) = 980*delta_h - 2.8*delta_i;
  der(delta_i) = -100*delta_i + 100*delta_V;
  
  annotation(
    Documentation(info="<html>
    <p>This model represents a magnetic suspension system where a ball is
    suspended in mid-air using an electromagnet. The system is nonlinear,
    but this model uses the linearized equations around an equilibrium point.</p>
    <p>The nonlinear equations are:</p>
    <ul>
      <li>m*d2h/dt2 = mg - K*i^2/h^2 (mechanical equation)</li>
      <li>V = L*di/dt + i*R (electrical equation)</li>
    </ul>
    <p>Linearized around h = 0.01 m, the state-space representation is:</p>
    <p>dx/dt = A*x + B*u</p>
    <p>y = C*x</p>
    <p>where x = [Δh, Δdh, Δi]^T, u = ΔV, and y = Δh</p>
    <p>The system is open-loop unstable (one pole in right-half plane).</p>
    </html>"),
    Icon(graphics={
      Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}, fillColor={255,255,255}),
      Text(extent={{-50,30},{50,-30}}, textString="MagSusp")
    }));
end MagneticSuspension;

