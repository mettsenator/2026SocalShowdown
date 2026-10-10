// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import static org.wpilib.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Elevator;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;


public class ElevatorSubsystem extends SubsystemBase{
    private static final Distance MIN_HEIGHT = Inches.of(0);
    private static final Distance MAX_HEIGHT = Inches.of(10);

    private double kP = 2;
    private double kI = 0;
    private double kD = 0.5;


    private SmartMotorControllerConfig ElevatorSMCconfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withDrumRadius(Inches.of(0.25))
        .withSoftLimits(MIN_HEIGHT, MAX_HEIGHT)

        .withClosedLoopController(kP, kI ,kD)
        .withSimClosedLoopController(kP, kI, kD)
        .withTrapezoidalProfile(InchesPerSecond.of(26), InchesPerSecondPerSecond.of(1))

        .withFeedforward(new ElevatorFeedforward(0,0,0))
        .withSimFeedforward(new ElevatorFeedforward(0,0,0))

        .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)

        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4.3)))

        .withMotorInverted(false)
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(10))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withStartingPosition(Inches.of(0))
        ;

    private SparkMax spark = new SparkMax(CANPort.CAN_S0, 19, MotorType.kBrushless);

    private SmartMotorController ElevatorSpark = new SparkWrapper(spark, DCMotor.getNEO(1), ElevatorSMCconfig);

    private ElevatorConfig elevatorConfig = new ElevatorConfig()
        .withHardLimits(MIN_HEIGHT, MAX_HEIGHT)
        .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
        .withCarriageWeight(Pounds.of(2))
        ;

    private Elevator elevator = new Elevator(elevatorConfig, ElevatorSpark);

    /**
   * Runs the elevator to the given height and does not end the command when reached.
   * @param height Distance to go to.
   * @return a Command
   */
  public Command run(Distance height) { return elevator.run(height);}

  /**
   * Runs the elevator to the given height and ends the command when reached, but not the closed loop controller.
   * @param height Distance to go to.
   * @param tolerance Distance tolerance for completion.
   * @return A Command
   */
  public Command runTo(Distance height, Distance tolerance) { return elevator.runTo(height, tolerance);}

  /**
   * Set the elevators closed loop controller setpoint.
   * @param height Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { elevator.setMeasurementPositionSetpoint(height);}

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) { return elevator.set(dutycycle);}

  /**
   * Set the voltage of the elevator, useful for feeding a WPILib {@link org.wpilib.command2.sysid.SysIdRoutine}.
   * @param volts Voltage to apply.
   */
  public Command setVoltage(Voltage volts) { return elevator.setVoltage(volts);}


    public ElevatorSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
