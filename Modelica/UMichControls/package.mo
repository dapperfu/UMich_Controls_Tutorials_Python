within ;
package UMichControls
  "Control system examples converted from Simulink to Modelica
   Based on University of Michigan Controls Tutorials for MATLAB & Simulink
   Converted to Modelica for use with OpenModelica"

  annotation(
    version="1.0.0",
    versionDate="2025-01-01",
    uses(Modelica(version="4.0.0")),
    Documentation(info="<html>
    <p>This package contains Modelica models of control system examples
    originally from the University of Michigan Controls Tutorials for MATLAB & Simulink.</p>
    <p>The examples include:</p>
    <ul>
      <li>Introduction: Mass-spring-damper and magnetic suspension systems</li>
      <li>CruiseControl: Vehicle speed control system</li>
      <li>MotorSpeed: DC motor speed control</li>
      <li>MotorPosition: DC motor position control</li>
      <li>Suspension: Quarter-car active suspension system</li>
      <li>InvertedPendulum: Cart-pendulum system</li>
      <li>AircraftPitch: Aircraft pitch control</li>
      <li>BallBeam: Ball and beam position control</li>
    </ul>
    <p>Each example includes system models and various control implementations
    (PID, Root Locus, Frequency Response, State Space, and Digital control).</p>
    </html>"));

  package Introduction "Introduction examples: mass-spring-damper and magnetic suspension"
    extends Modelica.Icons.ExamplesPackage;
  end Introduction;

  package CruiseControl "Cruise control system: vehicle speed control"
    extends Modelica.Icons.ExamplesPackage;
  end CruiseControl;

  package MotorSpeed "DC motor speed control system"
    extends Modelica.Icons.ExamplesPackage;
  end MotorSpeed;

  package MotorPosition "DC motor position control system"
    extends Modelica.Icons.ExamplesPackage;
  end MotorPosition;

  package Suspension "Quarter-car active suspension system"
    extends Modelica.Icons.ExamplesPackage;
  end Suspension;

  package InvertedPendulum "Inverted pendulum on cart system"
    extends Modelica.Icons.ExamplesPackage;
  end InvertedPendulum;

  package AircraftPitch "Aircraft pitch control system"
    extends Modelica.Icons.ExamplesPackage;
  end AircraftPitch;

  package BallBeam "Ball and beam position control system"
    extends Modelica.Icons.ExamplesPackage;
  end BallBeam;

end UMichControls;

