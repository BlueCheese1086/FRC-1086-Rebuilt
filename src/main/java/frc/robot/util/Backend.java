package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

/**
 * @author John Steele, created in Blue Cheese rookie year in order to help next year's rookies.
 */
public class Backend extends TalonFX {
  private TalonFXConfiguration config;
  /**
   * he 'Backend' class is an extension of the TalonFX motors developed by John Steele to help
   * rookies get started. We hope your gradle does not crash and that your paths always work first
   * try. And remember: "Java hates you." -James Farnsworth
   *
   * @param id is the identification number that the motor has, you can find this using the Phoenix
   *     Tuner X tool and connecting to the robot.
   * @param bus is the class used to get information on the available CANBus. It is not reccomended
   *     for rookies to go into this until more expierenced.
   */
  public Backend(int id, CANBus bus) {
    super(id, bus);
    this.config = new TalonFXConfiguration();
  }
  /**
   * The 'Backend' class is an extension of the TalonFX motors developed by John Steele to help
   * rookies get started. We hope your gradle does not crash and that your paths always work first
   * try. And remember: "Java hates you." -James Farnsworth
   *
   * @param id is the identification number that the motor has, you can find this using the Phoenix
   *     Tuner X tool and connecting to the robot.
   */
  public Backend(int id) {
    super(id);
    this.config = new TalonFXConfiguration();
  }

  /**
   * @param inverted is the boolean value that controls the inversion state of the motor from the
   *     front of the motor. If 'inverted' is true then the motor will spin counterclockwise
   *     positive, if not then it will spin clockwise positive.
   */
  public void configureInverted(boolean inverted) {
    if (inverted) {
      this.config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      this.config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
  }

  /**
   * @param neutralMode is the NeutralModeValue that the configurator will be set to. It must be set
   *     to either 'Brake' or 'Coast'. Brake or Coast is the state of the motor controller bridge
   *     when output is neutral or disabled will be set to.
   */
  public void configureNeutral(String neutralMode) {
    if (neutralMode == "Brake" || neutralMode == "brake") {
      this.config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      if (neutralMode == "Coast" || neutralMode == "coast") {
        this.config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      } else {
        System.out.println(
            "Warning! ' "
                + neutralMode
                + " ' is not a valid neutral mode configuration. Defaulting to Coast neutral mode.");
        this.config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      }
    }
  }

  /**
   * @param forwardPeak is the double of maximum (forward) output during duty cycle based control
   *     modes and is clamped between -1.0 and 1.0.
   * @param reversePeak is the double of maximum (reverse) output during duty cycle based control
   *     modes and is clamped between -1.0 and 1.0.
   */
  public void configureForwardDutyCycle(double forwardPeak, double reversePeak) {
    MathUtil.clamp(forwardPeak, -1.0, 1.0);
    MathUtil.clamp(reversePeak, -1.0, 1.0);
    this.config.MotorOutput.PeakForwardDutyCycle = forwardPeak;
    this.config.MotorOutput.PeakReverseDutyCycle = reversePeak;
  }

  /**
   * @param statorLimit is the amount of current allowed in the motor, this can be used to help
   *     restrict torque output of motor.
   * @param supplyLimit is the absolute amount of supply current allowed. This helps to prevent
   *     breaker trips and improve battery longevity and postpone brownouts.
   */
  public void configureCurrentLimits(double statorLimit, double supplyLimit) {
    MathUtil.clamp(statorLimit, 0, 800);
    MathUtil.clamp(supplyLimit, 0, 800);
    this.config.CurrentLimits.StatorCurrentLimit = statorLimit;
    this.config.CurrentLimits.StatorCurrentLimitEnable = true;

    this.config.CurrentLimits.SupplyCurrentLimit = supplyLimit;
    this.config.CurrentLimits.SupplyCurrentLimitEnable = true;
  }
  /**
   * Important: These parameters are being converted to Amp units.
   *
   * @param forwardLimit is the maximum forward torque output of the motor, enter in double but is
   *     converted to amps.
   * @param reverseLimitis the maximum reverse torque output of the motor, enter in double but is
   *     converted to amps.
   */
  public void configureTorqueLimits(double forwardLimit, double reverseLimit) {
    MathUtil.clamp(forwardLimit, -800, 800);
    MathUtil.clamp(reverseLimit, -800, 800);
    this.config.TorqueCurrent.PeakForwardTorqueCurrent = Amps.of(forwardLimit).in(Amps);
    this.config.TorqueCurrent.PeakReverseTorqueCurrent = -Amps.of(reverseLimit).in(Amps);
  }

  /**
   * The 'defaultConfigure' command sets the configuration of the Backend to a series of predefined
   * values. Useful for quick creation in strict circumstances, but uselessly rigid otherwise.
   */
  public void defaultConfigure() {
    this.config.MotorOutput.PeakForwardDutyCycle = 1.0;
    this.config.MotorOutput.PeakReverseDutyCycle = -1.0;

    this.config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    this.config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    this.config.CurrentLimits.StatorCurrentLimit = 60;
    this.config.CurrentLimits.StatorCurrentLimitEnable = true;

    this.config.CurrentLimits.SupplyCurrentLimit = 60;
    this.config.CurrentLimits.SupplyCurrentLimitEnable = true;

    this.config.TorqueCurrent.PeakForwardTorqueCurrent = Amps.of(80).in(Amps);
    this.config.TorqueCurrent.PeakReverseTorqueCurrent = -Amps.of(80).in(Amps);
  }

  /** The 'applyConfiguration' method applys your configuration to the motor for you. */
  public void applyConfiguration() {
    tryUntilOk(5, () -> this.getConfigurator().apply(config, 0.25));
  }

  /**
   * The 'runMaxForward' method runs the motor forward at its max speed. Its forward being its
   * inversion.
   */
  public void runMaxForward() {
    this.set(1.0);
  }

  /**
   * The 'runMaxForward' method runs the motor reverse at its max speed. Its reverse being its
   * opposite inversion.
   */
  public void runMaxReverse() {
    this.set(-1.0);
  }

  /**
   * This command sets the speed of the motor to the percentage of speed.
   *
   * @param speed the percentage speed of the motor is set at. If it is set to '1.0' the motor will
   *     run at 100% speed.
   */
  public void runSpeed(double speed) {
    MathUtil.clamp(speed, -1.0, 1.0);
    this.set(speed);
  }

  /**
   * This command sets the voltage of the motor to the 'volts' parameter Important: The parameter,
   * while a double, is being converted to Volt units.
   *
   * @param volts is the voltage unit that the motor is being set to, clamped between -12 and 12
   *     volts.
   */
  public void runVoltage(double volts) {
    MathUtil.clamp(volts, -12.0, 12.0);
    final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    this.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * The 'runVoltageBreak' sets the motor's voltage to 0. Note: This might not stop the motor
   * immediately, it just stops the supply of voltage to the motor.
   */
  public void runVoltageBreak() {
    final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    this.setControl(voltageRequest.withOutput(0));
  }

  /**
   * The 'runSpeedBrake' sets the motor speed percentage to 0% Note: This might not stop the motor
   * immediately, it just stops the motor from actively moving.
   */
  public void runSpeedBrake() {
    this.set(0);
  }
}
