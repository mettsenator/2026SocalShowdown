// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

public class IndexerSubsystem extends SubsystemBase {
  // ==================== CHANGE THESE INDEXER VALUES ====================
  private static final int INDEXER_MOTOR_1_CAN_ID = 17;
  private static final int INDEXER_MOTOR_2_CAN_ID = 20;
  private static final double INDEXER_DUTY_CYCLE = 0.5;
  private static final boolean INDEXER_MOTOR_1_INVERTED = false;
  private static final boolean INDEXER_MOTOR_2_INVERTED = true;
  private static final double INDEXER_CURRENT_LIMIT_AMPS = 20.0;
  private static final double INDEXER_RAMP_SECONDS = 0.1;
  // =====================================================================

  private final SparkMax indexerMotor1 =
      new SparkMax(CANPort.CAN_S0, INDEXER_MOTOR_1_CAN_ID, MotorType.kBrushed);
  private final SparkMax indexerMotor2 =
      new SparkMax(CANPort.CAN_S0, INDEXER_MOTOR_2_CAN_ID, MotorType.kBrushed);

  public IndexerSubsystem() {
    configureMotor(indexerMotor1, INDEXER_MOTOR_1_INVERTED);
    configureMotor(indexerMotor2, INDEXER_MOTOR_2_INVERTED);
  }

  private void configureMotor(SparkMax motor, boolean inverted) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake)
        .secondaryCurrentLimit(INDEXER_CURRENT_LIMIT_AMPS)
        .openLoopRampRate(INDEXER_RAMP_SECONDS);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Runs both indexer motors while scheduled and stops them when interrupted. */
  public Command run() {
    return Commands.startEnd(this::start, this::stop, this);
  }

  private void start() {
    indexerMotor1.setThrottle(INDEXER_DUTY_CYCLE);
    indexerMotor2.setThrottle(INDEXER_DUTY_CYCLE);
  }

  /** Immediately stops both indexer motors. */
  public void stop() {
    indexerMotor1.stopMotor();
    indexerMotor2.stopMotor();
  }
}
