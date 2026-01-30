package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private DCMotorSim feeder;

  public FeederIOSim() {
    feeder =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                ShooterConstants.Mechanical.J.in(KilogramSquareMeters),
                1.0),
            DCMotor.getKrakenX44Foc(1));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    feeder.update(0.02);

    inputs.feedAppliedVoltage = feeder.getInputVoltage();
    inputs.feedStatorCurrent = feeder.getCurrentDrawAmps();
  }

  @Override
  public void setFeedVoltage(double volts) {
    feeder.setInputVoltage(volts);
  }
}
