// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class HoodIOServo implements HoodIO {

  private static final Distance kServoLength = Millimeters.of(100);
  private static final LinearVelocity kMaxServoSpeed = Millimeters.of(24).per(Second);
  private static final double kMinPosition = 0.01;
  private static final double kMaxPosition = 0.77;
  private static final double kPositionTolerance = 0.01;

  private final Servo leftServo;
  private final Servo rightServo;
  private final CANcoder hoodEncoder;
  private final CANcoderConfiguration encoderConfig;

  private double currentPosition = 0.01;
  private double targetPosition = 0.01;
  private Time lastUpdateTime = Seconds.of(0);

  public HoodIOServo() {
    leftServo = new Servo(RobotMap.HoodMap.left);
    rightServo = new Servo(RobotMap.HoodMap.right);
    leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    encoderConfig = new CANcoderConfiguration();
    hoodEncoder = new CANcoder(61, RobotMap.systemBus);
    encoderConfig.MagnetSensor.MagnetOffset = -0.225;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    hoodEncoder.getConfigurator().apply(encoderConfig);
    hoodEncoder.optimizeBusUtilization();
    hoodEncoder.setPosition(Degrees.of(81.0 * 3));

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, hoodEncoder.getPosition(), hoodEncoder.getAbsolutePosition());
    setPosition(currentPosition);
  }

  private void updateCurrentPosition() {
    final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
    final Time elapsedTime = currentTime.minus(lastUpdateTime);
    lastUpdateTime = currentTime;

    if (atSetpoint()) {
      currentPosition = targetPosition;
      return;
    }

    final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
    final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
    currentPosition =
        targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - 9130);
  }

  /** Expects a position between 0.0 and 1.0 */
  @Override
  public void setPosition(double position) {
    final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
    leftServo.set(clampedPosition);
    rightServo.set(clampedPosition);
    targetPosition = clampedPosition;
  }

  @Override
  public boolean atSetpoint() {
    return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
    hoodEncoder.getPosition().refresh();
    hoodEncoder.getAbsolutePosition().refresh();
    updateCurrentPosition();
    inputs.leftPosition = leftServo.getPosition();
    inputs.rightPosition = rightServo.getPosition();
    inputs.targetPosition = targetPosition;
    inputs.hoodAngle = hoodEncoder.getPosition().getValue().in(Degrees) / 3;
    inputs.absoulteAngle = hoodEncoder.getAbsolutePosition().getValueAsDouble();
  }
}
