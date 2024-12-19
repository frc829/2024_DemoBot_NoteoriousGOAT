package frc.robot.commands;

import java.util.function.Function;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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







        public TelemetryCommands() {

        }
}
