package frc.robot.commands;

import static frc.robot.RobotContainer.telemetrySubsystem;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class TelemetryCommands {

        public static final class Constants {
                public static final Pose2d SpeakerTopStart = new Pose2d(0.76, 6.60, Rotation2d.fromDegrees(60.00));
                public static final Pose2d SpeakerMidStart = new Pose2d(1.30, 5.53, Rotation2d.fromDegrees(0));
                public static final Pose2d SpeakerBotStart = new Pose2d(0.76, 4.49, Rotation2d.fromDegrees(-60));
                public static final Pose2d pbjStart = new Pose2d(1.32, 4.49, Rotation2d.fromDegrees(-39.81));
                public static final Pose2d testingStartPose = new Pose2d(1.24, 5.48, Rotation2d.fromDegrees(0.0));
                public static final Pose2d AmpStart = new Pose2d(1.56, 7.41, Rotation2d.fromDegrees(90));

                public static final double fieldLengthMeters = 16.542;

        }

        public static final Function<Pose2d, Command> createSetStartPoseCommand = (pose) -> {
                Runnable setPose = () -> {
                        if (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red) {
                                Rotation2d robotPoseRotation = pose.getRotation();
                                double degrees = robotPoseRotation.getDegrees();
                                degrees = -(degrees - 90) + 90;
                                robotPoseRotation = Rotation2d.fromDegrees(degrees);
                                Pose2d newPose = new Pose2d(Constants.fieldLengthMeters - pose.getX(), pose.getY(),
                                                robotPoseRotation);
                                RobotContainer.telemetrySubsystem.setPoseEstimator.accept(newPose);
                        } else {
                                RobotContainer.telemetrySubsystem.setPoseEstimator.accept(pose);
                        }
                };

                Command command = Commands.runOnce(setPose,
                                RobotContainer.telemetrySubsystem);
                command.setName("Set Pose");
                return command;
        };

        public static final Supplier<Command> createResetPoseFromFrontCameraCommand = () -> {
                Runnable setPoseFromCamera = () -> {
                        RobotContainer.telemetrySubsystem.enableFieldDetectors.get(0).run();
                        var fieldPose0 = RobotContainer.telemetrySubsystem.fieldDetectorsPositions.get(0).getSecond()
                                        .get();
                        if (fieldPose0.isPresent()) {
                                telemetrySubsystem.setPoseEstimator.accept(fieldPose0.get());
                                SmartDashboard.putNumber("Pose Estimator Reset at TimeIndex",
                                                Timer.getFPGATimestamp());
                        }
                };
                Command command = Commands.runOnce(setPoseFromCamera,
                                RobotContainer.telemetrySubsystem);
                command.setName("Set Pose From Front Camera");
                return command;
        };
        public static final Supplier<Command> createResetPoseFromBackCameraCommand = () -> {
                Runnable setPoseFromCamera = () -> {
                        RobotContainer.telemetrySubsystem.enableFieldDetectors.get(1).run();
                        var fieldPose0 = RobotContainer.telemetrySubsystem.fieldDetectorsPositions.get(1).getSecond()
                                        .get();
                        if (fieldPose0.isPresent()) {
                                telemetrySubsystem.setPoseEstimator.accept(fieldPose0.get());
                        }
                };
                Command command = Commands.runOnce(setPoseFromCamera,
                                RobotContainer.telemetrySubsystem);
                command.setName("Set Pose From Back Camera");
                return command;
        };

        public static final Supplier<Command> createSetFrontCameraToFieldCommand = () -> {
                Runnable setFrontCameraToField = () -> {
                        telemetrySubsystem.enableFieldDetectors.get(0).run();
                };
                DriveCommands.createFieldCentricDriveOriginCommand.get();
                Command command = Commands.runOnce(setFrontCameraToField,
                                RobotContainer.telemetrySubsystem);
                command.setName("Set Front Camera to Field");
                return command;
        };

        public static final Supplier<Command> createSetRearCameraToFieldCommand = () -> {
                Runnable setFrontCameraToField = () -> {
                        telemetrySubsystem.enableFieldDetectors.get(1).run();
                };
                Command command = Commands.runOnce(setFrontCameraToField,
                                RobotContainer.telemetrySubsystem);
                command.setName("Set Rear Camera to Field");
                return command;
        };

        public static final Supplier<Command> createSetRearCameraToFieldCommandDriveSlow = () -> {
                Runnable setFrontCameraToField = () -> {
                        telemetrySubsystem.enableFieldDetectors.get(1).run();
                };
                Command setCameraCommand = Commands.runOnce(setFrontCameraToField,
                                RobotContainer.telemetrySubsystem);
                Command driveCommand = DriveCommands.createFieldCentricDriveOriginSlowCommand.get();
                Command command = Commands.race(setCameraCommand, driveCommand);
                command.setName("Set Rear Camera to Field");
                return command;
        };

        public static final Function<Integer, Command> createSetRearPriorityTargetCommand = (priorityTarget) -> {
                Runnable setPriorityTarget = () -> telemetrySubsystem.setPriorityTargetsFromFieldDetectors.get(1)
                                .accept(priorityTarget);
                Command command = Commands.runOnce(setPriorityTarget, telemetrySubsystem);
                return command;
        };

        public TelemetryCommands() {

        }
}
