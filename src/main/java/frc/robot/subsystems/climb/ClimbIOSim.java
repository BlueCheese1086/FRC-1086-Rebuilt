// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

  private static final double radius =
      0.5 / 2; // Pretended like it is a half inch hex shaft which it really is. the nominal
  // diameter of this hex shaft is stupid so I put 0.5 inches as the diameter.
  public static final double gearing = 10.0;
  private static final LoggedNetworkNumber KP =
      new LoggedNetworkNumber("Climb/KP", ClimbConstants.kP);
  private static final LoggedNetworkNumber KI =
      new LoggedNetworkNumber("Climb/KI", ClimbConstants.kD);
  private static final LoggedNetworkNumber KD =
      new LoggedNetworkNumber("Climb/KD", ClimbConstants.kD);
  private double position = 0.0;
  private ElevatorFeedforward feedforward;

  private static final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);

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
                Units.inchesToMeters(radius),
                ClimbConstants.gearing),
            gearbox,
            ClimbConstants.retractedHeight,
            ClimbConstants.extendedHeight,
            true,
            0.0);
    this.feedforward = new ElevatorFeedforward(KS.get(), KG.get(), KV.get());
  }

  @Override
  public void setPosition(double position) {
    closedLoop = true;
    controller.setSetpoint(position);
    this.position = position;
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
      controller.setPID(KP.get(), KI.get(), KD.get());
      feedforward.setKg(KG.get());
      feedforward.setKv(KV.get());
      feedforward.setKs(KS.get());

      double pidOutput = controller.calculate(climbSim.getPositionMeters());
      double feedforwardOutput = feedforward.calculate(controller.getSetpoint());
      double side = (pidOutput + feedforwardOutput);
      appliedVolts = side;
    }

    climbSim.setInputVoltage(MathUtil.clamp((appliedVolts), -12.0, 12.0));
    climbSim.update(0.02);

    inputs.climbPosition = Units.metersToInches(climbSim.getPositionMeters());
    inputs.motorConnected = true;
    inputs.targetPosition = this.position;
    inputs.angle = Radians.of(climbSim.getPositionMeters() / Units.inchesToMeters(0.5));
    inputs.volts = Volts.of(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    inputs.temp = Celsius.of(420);
    inputs.velocity =
        RadiansPerSecond.of(
            (climbSim.getVelocityMetersPerSecond() * 60) / (Units.inchesToMeters(0.5)));
    inputs.statorCurrent = Amps.of(climbSim.getCurrentDrawAmps());
  }
}
