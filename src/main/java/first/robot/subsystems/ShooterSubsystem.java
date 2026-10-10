// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import static org.wpilib.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ShooterSubsystem extends SubsystemBase {
  // ==================== CHANGE THESE SHOOTER VALUES ====================
  private static final int SHOOTER_CAN_ID = 18;
  private static final double SHOOTER_GEAR_REDUCTION = 1.0;
  private static final boolean SHOOTER_MOTOR_INVERTED = true;

  private static final double SHOOTER_WHEEL_DIAMETER_INCHES = 6.0;
  private static final double SHOOTER_MOMENT_OF_INERTIA_KG_M2 = 2;
  private static final double SHOOTER_SIM_MAX_RPM = 6000.0;

  private static final double SHOOTER_CURRENT_LIMIT_AMPS = 40.0;
  private static final double SHOOTER_OPEN_LOOP_RAMP_SECONDS = 1;
  private static final double SHOOTER_CLOSED_LOOP_RAMP_SECONDS = 1;

  // PID output is voltage; tune these values for the completed shooter.
  private static final double SHOOTER_KP = 0.5;
  private static final double SHOOTER_KI = 0.0;
  private static final double SHOOTER_KD = 0.0;

  // SimpleMotorFeedforward values: kS, kV, and kA.
  private static final double SHOOTER_KS = 0.0;
  private static final double SHOOTER_KV = 0.0;
  private static final double SHOOTER_KA = 0.0;
  // =====================================================================

  private final SmartMotorControllerConfig shooterMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD)
          .withSimClosedLoopController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD)
          .withFeedforward(
              new SimpleMotorFeedforward(SHOOTER_KS, SHOOTER_KV, SHOOTER_KA))
          .withSimFeedforward(
              new SimpleMotorFeedforward(SHOOTER_KS, SHOOTER_KV, SHOOTER_KA))
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          .withGearing(
              new MechanismGearing(GearBox.fromReductionStages(SHOOTER_GEAR_REDUCTION)))
          .withMomentOfInertia(KilogramSquareMeters.of(SHOOTER_MOMENT_OF_INERTIA_KG_M2))
          .withMotorInverted(SHOOTER_MOTOR_INVERTED)
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(SHOOTER_CURRENT_LIMIT_AMPS))
          .withClosedLoopRampRate(Seconds.of(SHOOTER_CLOSED_LOOP_RAMP_SECONDS))
          .withOpenLoopRampRate(Seconds.of(SHOOTER_OPEN_LOOP_RAMP_SECONDS));

  private final SparkFlex spark =
      new SparkFlex(CANPort.CAN_S0, SHOOTER_CAN_ID, MotorType.kBrushless);

  private final SmartMotorController shooterSpark =
      new SparkWrapper(spark, DCMotor.getNeoVortex(1), shooterMotorConfig);

  private final FlyWheelConfig shooterConfig =
      new FlyWheelConfig()
          .withDiameter(Inches.of(SHOOTER_WHEEL_DIAMETER_INCHES))
          .withSpeedometerSimulation(RPM.of(SHOOTER_SIM_MAX_RPM))
          .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig, shooterSpark);

  public ShooterSubsystem() {}

  /**
   * Runs the shooter continuously at the requested angular velocity.
   *
   * @param speed requested shooter speed
   * @return command that runs until interrupted
   */
  public Command run(AngularVelocity speed) {
    // Use the supplier overload so closed-loop control restarts after coasting.
    return shooter.run(() -> speed);
  }

  /**
   * Sets the shooter speed and ends when it is within the requested tolerance.
   *
   * @param speed requested shooter speed
   * @param tolerance allowed speed error
   * @return command that ends once the shooter reaches speed
   */
  public Command runTo(AngularVelocity speed, AngularVelocity tolerance) {
    return shooter.runTo(speed, tolerance);
  }

  /**
   * Sets the closed-loop shooter speed without creating a command.
   *
   * @param speed requested shooter speed
   */
  public void setSpeedSetpoint(AngularVelocity speed) {
    shooter.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Returns a trigger that is true while the shooter is within the speed tolerance.
   *
   * @param speed requested shooter speed
   * @param tolerance allowed speed error
   * @return at-speed trigger
   */
  public Trigger near(AngularVelocity speed, AngularVelocity tolerance) {
    return shooter.near(speed, tolerance);
  }

  /**
   * Gets the current measured shooter speed.
   *
   * @return measured shooter speed
   */
  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  /**
   * Runs the shooter using open-loop duty cycle control.
   *
   * @param dutyCycle value from -1 to 1
   * @return command that runs until interrupted
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
  }

  /**
   * Runs the shooter using open-loop voltage control.
   *
   * @param volts voltage to apply
   * @return command that runs until interrupted
   */
  public Command setVoltage(Voltage volts) {
    return shooter.setVoltage(volts);
  }

  /** Stops applying motor power and allows the shooter wheel to coast naturally. */
  public void coast() {
    shooterSpark.stopClosedLoopController();
    shooterSpark.setDutyCycle(0.0);
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
