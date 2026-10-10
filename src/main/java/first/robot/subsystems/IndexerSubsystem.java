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
  private static final int INDEXER_CAN_ID = 17; // TODO: Confirm the SPARK MAX CAN ID.
  private static final double INDEXER_DUTY_CYCLE = 0.5;
  private static final boolean INDEXER_MOTOR_INVERTED = false;
  private static final double INDEXER_CURRENT_LIMIT_AMPS = 20.0;
  private static final double INDEXER_RAMP_SECONDS = 0.1;
  // =====================================================================

  private final SparkMax indexerMotor =
      new SparkMax(CANPort.CAN_S0, INDEXER_CAN_ID, MotorType.kBrushed);

  public IndexerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(INDEXER_MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake)
        .secondaryCurrentLimit(INDEXER_CURRENT_LIMIT_AMPS)
        .openLoopRampRate(INDEXER_RAMP_SECONDS);

    indexerMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Runs the indexer while scheduled and stops it when interrupted. */
  public Command run() {
    return Commands.startEnd(
        () -> indexerMotor.setThrottle(INDEXER_DUTY_CYCLE), indexerMotor::stopMotor, this);
  }

  /** Immediately stops the indexer motor. */
  public void stop() {
    indexerMotor.stopMotor();
  }
}
