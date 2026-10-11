// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import com.ctre.phoenix6.Utils;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.networktables.TimestampedDoubleArray;

public class LimelightSubsystem extends SubsystemBase {
  // ==================== CHANGE THESE VISION VALUES ====================
  private static final String LIMELIGHT_NAME = "limelight";

  // MegaTag2 using the WPILib blue-alliance field origin.
  private static final String LIMELIGHT_POSE_TOPIC = "botpose_orb_wpiblue";
  private static final int MINIMUM_TAG_COUNT = 1;
  private static final double MAX_ANGULAR_SPEED_DEGREES_PER_SECOND = 720.0;
  private static final double MAX_MEASUREMENT_LATENCY_MILLISECONDS = 500.0;

  // Larger values make the CTRE pose estimator trust vision less.
  private static final double VISION_X_STANDARD_DEVIATION_METERS = 0.7;
  private static final double VISION_Y_STANDARD_DEVIATION_METERS = 0.7;
  private static final double VISION_HEADING_STANDARD_DEVIATION_RADIANS = 9_999_999.0;

  private static final boolean RESET_POSE_AT_AUTONOMOUS_START = true;
  private static final boolean RESET_POSE_AT_TELEOP_START = true;

  // Drag these Pose2d topics into AdvantageScope's 2D Field "Poses" section.
  private static final String ESTIMATED_POSE_TOPIC = "/AdvantageScope/EstimatedRobotPose";
  private static final String LIMELIGHT_POSE_OUTPUT_TOPIC = "/AdvantageScope/LimelightPose";
  private static final String VISION_ACCEPTED_TOPIC = "/AdvantageScope/VisionMeasurementAccepted";
  // ====================================================================

  private static final int X_INDEX = 0;
  private static final int Y_INDEX = 1;
  private static final int YAW_INDEX = 5;
  private static final int LATENCY_INDEX = 6;
  private static final int TAG_COUNT_INDEX = 7;
  private static final int MINIMUM_POSE_ARRAY_LENGTH = 11;
  private static final double NANOSECONDS_PER_SECOND = 1_000_000_000.0;

  private final CommandSwerveDrivetrain drivetrain;
  private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  private final DoubleArraySubscriber limelightPoseSubscriber;
  private final DoubleArrayPublisher robotOrientationPublisher;
  private final StructPublisher<Pose2d> estimatedPosePublisher;
  private final StructPublisher<Pose2d> limelightPosePublisher;
  private final BooleanPublisher visionAcceptedPublisher;

  private long lastPoseUpdateTimestamp;
  private boolean wasAutonomousEnabled;
  private boolean wasTeleopEnabled;
  private boolean poseResetPending;

  public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    var limelightTable = networkTables.getTable(LIMELIGHT_NAME);
    limelightPoseSubscriber =
        limelightTable.getDoubleArrayTopic(LIMELIGHT_POSE_TOPIC).subscribe(new double[0]);
    robotOrientationPublisher =
        limelightTable.getDoubleArrayTopic("robot_orientation_set").publish();

    estimatedPosePublisher =
        networkTables.getStructTopic(ESTIMATED_POSE_TOPIC, Pose2d.struct).publish();
    limelightPosePublisher =
        networkTables.getStructTopic(LIMELIGHT_POSE_OUTPUT_TOPIC, Pose2d.struct).publish();
    visionAcceptedPublisher =
        networkTables.getBooleanTopic(VISION_ACCEPTED_TOPIC).publish();
  }

  /** Causes the next accepted Limelight pose to replace the drivetrain's estimated pose. */
  public void requestPoseReset() {
    poseResetPending = true;
  }

  @Override
  public void periodic() {
    var drivetrainState = drivetrain.getStateCopy();
    Pose2d currentPose = drivetrainState.Pose;
    double angularSpeedDegreesPerSecond = Math.toDegrees(drivetrainState.Velocity.omega);

    // MegaTag2 needs the robot's current odometry heading; this does not reset robot pose.
    robotOrientationPublisher.set(
        new double[] {
          currentPose.getRotation().getDegrees(),
          angularSpeedDegreesPerSecond,
          0.0,
          0.0,
          0.0,
          0.0
        });
    networkTables.flush();

    boolean autonomousEnabled = RobotState.isAutonomousEnabled();
    boolean teleopEnabled = RobotState.isTeleopEnabled();
    if (RESET_POSE_AT_AUTONOMOUS_START && autonomousEnabled && !wasAutonomousEnabled) {
      requestPoseReset();
    }
    if (RESET_POSE_AT_TELEOP_START && teleopEnabled && !wasTeleopEnabled) {
      requestPoseReset();
    }

    if (!autonomousEnabled && wasAutonomousEnabled && !teleopEnabled) {
      // Do not allow a missing autonomous measurement to cause a surprise reset in teleop.
      poseResetPending = false;
    }
    if (!teleopEnabled && wasTeleopEnabled && !autonomousEnabled) {
      // Do not carry an unfulfilled teleop reset into a later robot mode.
      poseResetPending = false;
    }
    wasAutonomousEnabled = autonomousEnabled;
    wasTeleopEnabled = teleopEnabled;

    boolean accepted = processLatestVisionMeasurement(angularSpeedDegreesPerSecond);
    visionAcceptedPublisher.set(accepted);
    estimatedPosePublisher.set(drivetrain.getStateCopy().Pose);
  }

  private boolean processLatestVisionMeasurement(double angularSpeedDegreesPerSecond) {
    TimestampedDoubleArray timestampedPose = limelightPoseSubscriber.getAtomic();
    if (timestampedPose.timestamp == 0 || timestampedPose.timestamp == lastPoseUpdateTimestamp) {
      return false;
    }
    lastPoseUpdateTimestamp = timestampedPose.timestamp;

    double[] poseData = timestampedPose.value;
    if (poseData.length < MINIMUM_POSE_ARRAY_LENGTH) {
      return false;
    }

    int tagCount = (int) poseData[TAG_COUNT_INDEX];
    double latencyMilliseconds = poseData[LATENCY_INDEX];
    if (tagCount < MINIMUM_TAG_COUNT
        || Math.abs(angularSpeedDegreesPerSecond) > MAX_ANGULAR_SPEED_DEGREES_PER_SECOND
        || !Double.isFinite(latencyMilliseconds)
        || latencyMilliseconds < 0.0
        || latencyMilliseconds > MAX_MEASUREMENT_LATENCY_MILLISECONDS) {
      return false;
    }

    double xMeters = poseData[X_INDEX];
    double yMeters = poseData[Y_INDEX];
    double yawDegrees = poseData[YAW_INDEX];
    if (!Double.isFinite(xMeters)
        || !Double.isFinite(yMeters)
        || !Double.isFinite(yawDegrees)) {
      return false;
    }

    Pose2d visionPose =
        new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(yawDegrees));
    limelightPosePublisher.set(visionPose);

    if (poseResetPending) {
      drivetrain.resetPose(visionPose);
      poseResetPending = false;
      return true;
    }

    double networkTablesAgeSeconds =
        Math.max(0.0, NetworkTablesJNI.now() - timestampedPose.timestamp)
            / NANOSECONDS_PER_SECOND;
    double measurementTimestampSeconds =
        Utils.getCurrentTimeSeconds()
            - networkTablesAgeSeconds
            - latencyMilliseconds / 1000.0;

    drivetrain.addVisionMeasurement(
        visionPose,
        measurementTimestampSeconds,
        VecBuilder.fill(
            VISION_X_STANDARD_DEVIATION_METERS,
            VISION_Y_STANDARD_DEVIATION_METERS,
            VISION_HEADING_STANDARD_DEVIATION_RADIANS));
    return true;
  }
}
