package frc.robot.subsystems.shooter.FeederIO;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private DCMotorSim feeder;

  public FeederIOSim() {
    feeder =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.01, 1.0),
            DCMotor.getKrakenX44Foc(1));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    feeder.update(0.02);
    inputs.feederVoltage = Volts.of(feeder.getInputVoltage());
  }

  @Override
  public void setFeedVoltage(double volts) {
    feeder.setInputVoltage(volts);
  }
}
