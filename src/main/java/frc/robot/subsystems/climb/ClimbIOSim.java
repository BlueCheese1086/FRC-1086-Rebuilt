// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ClimbIOSim implements ClimbIO {

  // private static final double kS = 0; // 1.0;
  private static final LoggedNetworkNumber KS = new LoggedNetworkNumber("Climb/KS", 0.0);
  // private static final double kG = 4.0;
  private static final LoggedNetworkNumber KG = new LoggedNetworkNumber("Climb/KG", 4.0);
  // private static final double kV = 1.3; // max battery voltage / max motor rpm
  private static final LoggedNetworkNumber KV = new LoggedNetworkNumber("Climb/KV", 1.3);
  private static final LoggedNetworkNumber div = new LoggedNetworkNumber("Climb/div", 5.655);

  private static final double armVel = 6.7;
  private static final double armAccel = 6.7;

  private static final double yes = 6.7;
  private double kFF = 0.0;

  private static final double kA = 0.0;
  private static final double radius = 2.0;
  public static final double gearing = 1.0 / 10.0;
  private static final double thesamethingasthevalueinthefeedback = 2 * Math.PI * radius * gearing;
  private static final LoggedNetworkNumber KP = new LoggedNetworkNumber("Climb/KP", 0.39551);
  private static final LoggedNetworkNumber KI = new LoggedNetworkNumber("Climb/KI", 0.0);
  private static final LoggedNetworkNumber KD = new LoggedNetworkNumber("Climb/KD", 1.1);
  private static final double kP = 0.43556;
  private static final double kI = 0.0;
  private double position = 0.0;
  private static final double kD = 1.0;
  private ElevatorFeedforward feedforward;

  private MotionMagicVoltage ICastFireball = new MotionMagicVoltage(position);

  private static final DCMotor gearbox = DCMotor.getKrakenX60(1);

  private final ElevatorSim climbSim;
  private boolean closedLoop = false;
  private PIDController controller = new PIDController(KP.get(), KI.get(), KD.get());
  private double appliedVolts = 0.0;

  public ClimbIOSim() {
    climbSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                gearbox,
                Units.lbsToKilograms(5.67),
                Units.inchesToMeters(1),
                ClimbConstants.gearing),
            gearbox,
            Units.inchesToMeters(ClimbConstants.retractedHeight),
            Units.inchesToMeters(ClimbConstants.extendendHeight),
            true,
            0.0);
    this.feedforward = new ElevatorFeedforward(KS.get(), KG.get(), KV.get());
  }

  @Override
  public void setPosition(double position) {
    double pos = position * thesamethingasthevalueinthefeedback;
    if (pos > 10) {
      pos = 10;
    }
    closedLoop = true;
    controller.setSetpoint(pos);
    this.position = pos;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void resetEncoder() {
    controller.reset();
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {

    if (closedLoop) {
      controller.setP(KP.get());
      controller.setI(KI.get());
      controller.setD(KD.get());
      feedforward.setKg(KG.get());
      feedforward.setKv(KV.get());
      feedforward.setKs(KS.get());

      double pidOutput = controller.calculate(climbSim.getPositionMeters());
      double feedforwardOutput = feedforward.calculate(controller.getSetpoint());
      double side = (pidOutput + feedforwardOutput) - this.position;
      appliedVolts = side + 0;
    }

    climbSim.setInputVoltage(MathUtil.clamp((appliedVolts), -12.0, 12.0));
    climbSim.update(0.02);

    inputs.climbPosition = Units.metersToInches(climbSim.getPositionMeters());

    inputs.motorConnected = true;
    inputs.targetPosition = this.position;
    inputs.angle = Rotation2d.kZero.getMeasure();
    inputs.volts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    inputs.temp = yes;
    inputs.velocity =
        (climbSim.getVelocityMetersPerSecond() * 60) / (2 * Math.PI * Units.inchesToMeters(0.5));
    inputs.statorCurrent = Math.abs(climbSim.getCurrentDrawAmps());
    inputs.supplyCurrent = Math.abs(climbSim.getCurrentDrawAmps());
  }
}
