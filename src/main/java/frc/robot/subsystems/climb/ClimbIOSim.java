// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim implements ClimbIO {

  // private static final double kS = 0; // 1.0;
  private static final LoggedTunableNumber KS = new LoggedTunableNumber("Climb/KS", 0.0);
  // private static final double kG = 4.0;
  private static final LoggedTunableNumber KG = new LoggedTunableNumber("Climb/KG", 0.46747);
  // private static final double kV = 1.3; // max battery voltage / max motor rpm
  private static final LoggedTunableNumber KV = new LoggedTunableNumber("Climb/KV", 0.0);

  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Climb/KP", 5.8321);
  private static final LoggedTunableNumber KI = new LoggedTunableNumber("Climb/KI", 0.0);
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Climb/KD", 0.93424);
  private double position = 0.0;
  private ElevatorFeedforward feedforward;

  private static final DCMotor gearbox = DCMotor.getKrakenX60(1);

  private final ElevatorSim climbSim;
  private boolean closedLoop = false;
  private PIDController controller = new PIDController(KP.get(), KI.get(), KD.get());
  private double appliedVolts = 0.0;

  public ClimbIOSim() {
    climbSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                gearbox, Units.lbsToKilograms(1.0), ClimbConstants.radius, ClimbConstants.gearing),
            gearbox,
            ClimbConstants.retractedHeight,
            ClimbConstants.extendedHeight,
            true,
            0.0);
    this.feedforward = new ElevatorFeedforward(KS.get(), KG.get(), KV.get());
  }

  @Override
  public void setPosition(double position) {
    double pos = position;
    if (pos > ClimbConstants.extendedHeight) {
      pos = ClimbConstants.extendedHeight;
    }
    closedLoop = true;
    controller.setSetpoint(pos);
    this.position = pos;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    Logger.recordOutput("Climb/Input Voltage", volts);
    climbSim.setInputVoltage(volts);
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
      double feedforwardOutput = feedforward.calculate(0.0);
      double side = (pidOutput + feedforwardOutput);
      appliedVolts = side + 0;
    }

    climbSim.update(0.02);
    climbSim.setInputVoltage(MathUtil.clamp((appliedVolts), -12.0, 12.0));

    inputs.climbPosition = climbSim.getPositionMeters();

    inputs.motorConnected = true;
    inputs.targetPosition = this.position;
    inputs.volts = appliedVolts;
    inputs.velocity = climbSim.getVelocityMetersPerSecond();
    inputs.statorCurrent = Math.abs(climbSim.getCurrentDrawAmps());
    inputs.supplyCurrent = Math.abs(climbSim.getCurrentDrawAmps());
  }
}
