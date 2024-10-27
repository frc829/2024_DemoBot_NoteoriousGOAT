// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.controllers.Controller;
import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Amp;
import frc.robot.commands.BabyBird;
import frc.robot.commands.Barf;
import frc.robot.commands.BasicCommands;
import frc.robot.commands.Climb;
import frc.robot.commands.ControllerCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorShoot;
import frc.robot.commands.Fender;
import frc.robot.commands.Ground;
import frc.robot.commands.Level;
import frc.robot.commands.MovingRangedShot;
import frc.robot.commands.Pass;
import frc.robot.commands.NoteDetect;
import frc.robot.commands.SpinUp;
import frc.robot.commands.StationaryRangedShot;
import frc.robot.commands.TelemetryCommands;
import frc.robot.subsystems.BottomShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.InnerIntakeSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.NotedLoadedSubsystem;
import frc.robot.subsystems.OuterIntakeSubsystem;
import frc.robot.subsystems.ServoRollerSubsystem;
import frc.robot.subsystems.TopShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.triggers.ComplexTriggers;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

public class RobotContainer {

        private static final class Constants {
                private static final class Controller {
                        private static final double deadband = 0.1;
                        private static final int driverPort = 0;
                        private static final int operatorPort = 1;
                        // private static final int testPort = 2;
                }

                private static final ReplanningConfig replanningConfig = new ReplanningConfig(
                                true,
                                true,
                                0.5,
                                0.5);

                private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                                new PIDConstants(2),
                                new PIDConstants(2),
                                DriveSubsystem.Constants.maxLinearVelocity.in(MetersPerSecond),
                                DriveSubsystem.Constants.driveRadius.in(Meters),
                                replanningConfig,
                                0.02);

                private static final BooleanSupplier shouldFlipPath = () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                                return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                };
        }

        public static final Orchestra orchestra = new Orchestra();
        public static final Controller driver = Controller.createFromXBox
                        .apply(Constants.Controller.driverPort)
                        .apply(Constants.Controller.deadband)
                        .apply(true);
        public static final Controller operator = Controller.createFromXBox
                        .apply(Constants.Controller.operatorPort)
                        .apply(Constants.Controller.deadband)
                        .apply(true);
        // public static final Controller test = Controller.createFromXBox
        // .apply(Constants.Controller.testPort)
        // .apply(Constants.Controller.deadband);

        public static final TopShooterSubsystem topShooterSubsystem = TopShooterSubsystem.create.get();
        public static final BottomShooterSubsystem bottomShooterSubsystem = BottomShooterSubsystem.create.get();
        public static final OuterIntakeSubsystem outerIntakeSubsystem = OuterIntakeSubsystem.create.get();
        public static final InnerIntakeSubsystem innerIntakeSubsystem = InnerIntakeSubsystem.create.get();
        public static final TransportSubsystem transportSubsystem = TransportSubsystem.create.get();
        public static final SingulatorSubsystem singulatorSubsystem = SingulatorSubsystem.create.get();
        public static final NotedLoadedSubsystem notedLoadedSubsystem = NotedLoadedSubsystem.create.get();
        public static final ShooterTiltSubsystem shooterTiltSubsystem = ShooterTiltSubsystem.create.get();
        public static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.create.get();
        public static final DriveSubsystem driveSubsystem = DriveSubsystem.create.get();
        public static final TelemetrySubsystem telemetrySubsystem = TelemetrySubsystem.create.get();
        public static final MechanismSubsystem mechanismSubsystem = new MechanismSubsystem();
        public static final ServoRollerSubsystem servoRollerSubsystem = new ServoRollerSubsystem();
        public static final PowerDistribution pdh = new PowerDistribution();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                SmartDashboard.putData("PDH", pdh);
                AutoBuilder.configureHolonomic(
                                telemetrySubsystem.poseEstimate,
                                telemetrySubsystem.setPoseEstimator,
                                driveSubsystem.robotSpeeds,
                                driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d()),
                                Constants.config,
                                Constants.shouldFlipPath,
                                driveSubsystem);
                // SmartDashboard.putData("Outer Intake", outerIntakeSubsystem);
                // SmartDashboard.putData("Inner Intake", innerIntakeSubsystem);
                // SmartDashboard.putData("Transport", transportSubsystem);
                // SmartDashboard.putData("Singulator", singulatorSubsystem);
                SmartDashboard.putData("NoteLoaded", notedLoadedSubsystem);
                // SmartDashboard.putData("Top Shooter", topShooterSubsystem);
                // SmartDashboard.putData("Bottom Shooter", bottomShooterSubsystem);
                SmartDashboard.putData("Shooter Tilt", shooterTiltSubsystem);
                SmartDashboard.putData("Elevator", elevatorSubsystem);
                // SmartDashboard.putData("Drive", driveSubsystem);
                SmartDashboard.putData("Telemetry", telemetrySubsystem);
                // SmartDashboard.putData("Servo Roller", servoRollerSubsystem);

                operator.back.onTrue(Level.command.get());
                operator.leftY.whileTrue(BasicCommands.Elevator.createDriveElevatorCommand.apply(operator.leftYValue));
                operator.rightY.whileTrue(BasicCommands.Tilt.createRotateTiltCommand.apply(operator.rightYValue));
                operator.fullTrigger.whileTrue(BasicCommands.ManualSpinners.spinCommand.get());
                operator.b.whileTrue(Barf.barfCommand.get());
                operator.y.whileTrue(SpinUp.createSpinUp.get());
                operator.x.whileTrue(Climb.createClimbPrep.get());
                operator.x.onFalse(Climb.createClimbEnd.get());
                operator.a.whileTrue(Amp.createAmpPosition.get());
                operator.leftBumper.whileTrue(Ground.groundCommandThenLevel.get());
                operator.leftBumper.onFalse(Level.command.get());
                operator.padUp.whileTrue(BabyBird.babyBirdCommandThenLevel.get());
                operator.padUp.onFalse(Level.command.get());
                operator.rightBumper.whileTrue(Ground.groundCommand.get());
                operator.padDown.whileTrue(ElevatorShoot.create.get());
                operator.padDown.onFalse(Level.command.get());
                driver.leftBumper.whileTrue(Fender.create.get());
                driver.leftBumper.onFalse(Level.command.get());
                driver.rightBumper.whileTrue(StationaryRangedShot.Ranged.create.get());
                driver.rightBumper.onFalse((Level.command.get()));
                driver.back.whileTrue(DriveCommands.createZeroModulesCommand.get());

                driver.start.onTrue(TelemetryCommands.createSetStartPoseCommand
                                .apply(TelemetryCommands.Constants.testingStartPose));
                // driver.start.onTrue(Commands.runOnce(driveSubsystem.resetSteerEncodersFromAbsolutes));
                driver.x.whileTrue(NoteDetect.create.get());
                driver.x.onFalse(TelemetryCommands.createSetFrontCameraToFieldCommand.get()
                                .andThen(Level.command.get()));

                driver.a.whileTrue(Amp.createAmpDrop.get());
                driver.a.onFalse(Level.command.get());
                driver.b.whileTrue(MovingRangedShot.Ranged.create.get());
                driver.b.onFalse(Level.command.get());
                driver.y.whileTrue(Pass.create.get());
                driver.y.onFalse(Level.command.get());
                // driver.y.whileTrue(AlignedPass.create.get());
                // driver.y.onFalse(Level.command.get().andThen(AlignedPass.resetPriorityTargetCommand.get()));
                ComplexTriggers.robotCentricOriginDriveTrigger
                                .whileTrue(DriveCommands.createRobotCentricDriveOriginCommand.get());
                ComplexTriggers.fieldCentricOriginDriveTrigger
                                .whileTrue(DriveCommands.createFieldCentricDriveOriginCommand.get());
                ComplexTriggers.fieldCentricFLDriveTrigger
                                .whileTrue(DriveCommands.createFieldCentricDriveRLCommand.get());
                ComplexTriggers.fieldCentricFLDriveTrigger
                                .onFalse(DriveCommands.createFieldCentricDriveOriginCommand.get());
                ComplexTriggers.fieldCentricFRDriveTrigger
                                .whileTrue(DriveCommands.createFieldCentricDriveRRCommand.get());
                ComplexTriggers.fieldCentricFRDriveTrigger
                                .onFalse(DriveCommands.createFieldCentricDriveOriginCommand.get());

                ComplexTriggers.robotCentricFLDriveTrigger
                                .whileTrue(DriveCommands.createRobotCentricDriveRLCommand.get());
                ComplexTriggers.robotCentricFLDriveTrigger
                                .onFalse(DriveCommands.createRobotCentricDriveOriginCommand.get());
                ComplexTriggers.robotCentricFRDriveTrigger
                                .whileTrue(DriveCommands.createRobotCentricDriveRRCommand.get());
                ComplexTriggers.robotCentricFRDriveTrigger
                                .onFalse(DriveCommands.createRobotCentricDriveOriginCommand.get());
                ComplexTriggers.noteLoadedTrigger.onTrue(ControllerCommands.createOperatorRumble.get());
                ComplexTriggers.noteLoadedTrigger2.onTrue(ControllerCommands.createDriverRumble.get());
                // ComplexTriggers.noteLoadedTrigger.onTrue(Commands.parallel(
                //         ControllerCommands.createOperatorRumble.get(),
                //         ControllerCommands.createDriverRumble.get()
                // ));

                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        telemetrySubsystem.field2d.getObject("path").setPoses(poses);
                });

                PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

                NamedCommands.registerCommand("BackReset",
                                TelemetryCommands.createResetPoseFromBackCameraCommand.get());
                NamedCommands.registerCommand("Fender", Fender.createWithDelay.get());
                NamedCommands.registerCommand("Ranged", StationaryRangedShot.Ranged.createWithDelay.get());
                NamedCommands.registerCommand("Pass", Pass.createWithDelay.get());

                NamedCommands.registerCommand("Pickup", Ground.groundCommand.get());
                NamedCommands.registerCommand("SpinUp", SpinUp.createSpinUp.get());
                NamedCommands.registerCommand("GetLow", Level.command.get());
                NamedCommands.registerCommand("SpeakerTopStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.SpeakerTopStart));
                NamedCommands.registerCommand("SpeakerMidStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.SpeakerMidStart));
                NamedCommands.registerCommand("SpeakerBotStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.SpeakerBotStart));
                NamedCommands.registerCommand("AmpStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.AmpStart));
                NamedCommands.registerCommand("RearToField",
                                TelemetryCommands.createSetRearCameraToFieldCommand.get());

                NamedCommands.registerCommand("GetNote",
                                NoteDetect.createForAuto.get());

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        public Optional<Rotation2d> getRotationTargetOverride() {
                // Some condition that should decide if we want to override rotation
                if (driver.x.getAsBoolean()) {
                        // Return an optional containing the rotation override (this should be a field
                        // relative rotation)
                        var poses = telemetrySubsystem.field2d.getObject("path").getPoses();
                        if (poses.size() != 0) {
                                var lastPose = poses.get(poses.size() - 1);
                                var secondLastPose = poses.get(poses.size() - 2);
                                var vector = lastPose.getTranslation().minus(secondLastPose.getTranslation());
                                return Optional.of(vector.getAngle());
                        } else {
                                return Optional.empty();
                        }

                } else {
                        // return an empty optional when we don't want to override the path's rotation
                        return Optional.empty();
                }
        }

        public void resetEncoders() {
                shooterTiltSubsystem.resetRelEncoderFromAbsolute.run();
                driveSubsystem.resetSteerEncodersFromAbsolutes.run();
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
