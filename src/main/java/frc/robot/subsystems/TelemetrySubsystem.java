package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.FieldDetector;
import com.compLevel0.Gyroscope;
import com.compLevel0.ObjectDetector;
import com.compLevel1.Telemetry;
import com.kauailabs.navx.frc.AHRS;
import com.utility.GoatMath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TelemetrySubsystem extends SubsystemBase {

        public static final class Constants {
                public static final double poseTranslationToleranceMeters = 1;
                public static final double poseRotationToleranceDegrees = 2;
                private static final List<Pair<String, Pose3d>> fieldDetectorNames = List.of(
                                new Pair<>(
                                                "limelight-front",
                                                new Pose3d(Units.inchesToMeters(15.5),
                                                                Units.inchesToMeters(0),
                                                                Units.inchesToMeters(19.25),
                                                                new Rotation3d(
                                                                                0,
                                                                                0,
                                                                                Math.toRadians(0)))),
                                new Pair<>(
                                                "limelight-rear",
                                                new Pose3d(
                                                                -0.2702,
                                                                0.0198,
                                                                0.32,
                                                                new Rotation3d(
                                                                                0,
                                                                                Math.toRadians(15),
                                                                                Math.toRadians(180)))));
                private static final List<Pair<String, Pose3d>> objectDetectorNamesPositions = List.of(
                                new Pair<>(
                                                "limelight-front",
                                                new Pose3d(
                                                                Units.inchesToMeters(15.5),
                                                                Units.inchesToMeters(0),
                                                                Units.inchesToMeters(19.25),
                                                                new Rotation3d(
                                                                                0,
                                                                                0,
                                                                                Math.toRadians(0)))));
        }

        public final Measure<Velocity<Velocity<Distance>>> accelerationX;
        public final Measure<Velocity<Velocity<Distance>>> accelerationY;
        public final Field2d field2d;
        public final Supplier<ChassisSpeeds> fieldSpeeds;
        public final Supplier<Pose2d> poseEstimate;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorsPositions;
        public final List<Pair<String, Supplier<Optional<Measure<Time>>>>> fieldDetectorLatencies;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> objectPositions;
        public final Supplier<Optional<Double>> priorityTargetDistance;
        public final Supplier<Optional<Rotation2d>> priorityTargetRotation;
        public final Consumer<Pose2d> setPoseEstimator;
        public final Runnable resetPoseEstimateFromFieldDetectors;
        public final List<Consumer<Integer>> setPriorityTargetsFromFieldDetectors;
        public final List<Runnable> enableFieldDetectors;
        public final List<Runnable> enableObjectDetectors;
        public final Runnable update;

        private TelemetrySubsystem(
                        Measure<Velocity<Velocity<Distance>>> accelerationX,
                        Measure<Velocity<Velocity<Distance>>> accelerationY,
                        Field2d field2d,
                        Supplier<ChassisSpeeds> fieldSpeeds,
                        Supplier<Pose2d> poseEstimate,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorsPositions,
                        List<Pair<String, Supplier<Optional<Measure<Time>>>>> fieldDetectorLatencies,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> objectPositions,
                        Supplier<Optional<Double>> priorityTargetDistance,
                        Supplier<Optional<Rotation2d>> priorityTargetRotation,
                        Consumer<Pose2d> setPoseEstimator,
                        Runnable resetPoseEstimateFromFieldDetectors,
                        List<Consumer<Integer>> setPriorityTargetsFromFieldDetectors,
                        List<Runnable> enableFieldDetectors,
                        List<Runnable> enableObjectDetectors,
                        Runnable update) {
                this.accelerationX = accelerationX;
                this.accelerationY = accelerationY;
                this.field2d = field2d;
                this.fieldSpeeds = fieldSpeeds;
                this.poseEstimate = poseEstimate;
                this.fieldDetectorsPositions = fieldDetectorsPositions;
                this.fieldDetectorLatencies = fieldDetectorLatencies;
                this.objectPositions = objectPositions;
                this.priorityTargetDistance = priorityTargetDistance;
                this.priorityTargetRotation = priorityTargetRotation;
                this.setPoseEstimator = setPoseEstimator;
                this.resetPoseEstimateFromFieldDetectors = resetPoseEstimateFromFieldDetectors;
                this.setPriorityTargetsFromFieldDetectors = setPriorityTargetsFromFieldDetectors;
                this.enableFieldDetectors = enableFieldDetectors;
                this.enableObjectDetectors = enableObjectDetectors;
                this.update = update;

                enableFieldDetectors.stream().forEachOrdered((detector) -> detector.run());
        }

        @Override
        public void periodic() {
                update.run();
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                super.initSendable(builder);
                // builder.addDoubleProperty(
                // "Linear Acceleration Mag",
                // () -> GoatMath.round(accelerationMag.in(MetersPerSecondPerSecond), 3),
                // null);

                // builder.addDoubleProperty(
                // "Angular Acceleration Mag",
                // () -> GoatMath.round(accelerationMag.in(MetersPerSecondPerSecond)
                // / DriveSubsystem.Constants.driveRadius.in(Meters), 3),
                // null);

                builder.addDoubleProperty(
                                "Field Forward Velocity (mps)",
                                () -> GoatMath.round(fieldSpeeds.get().vxMetersPerSecond, 3),
                                null);

                builder.addDoubleProperty(
                                "Field Strafe Velocity (mps)",
                                () -> GoatMath.round(fieldSpeeds.get().vyMetersPerSecond, 3),
                                null);

                builder.addDoubleProperty(
                                "Field Rotational Velocity (dps)",
                                () -> GoatMath.round(Math.toDegrees(fieldSpeeds.get().omegaRadiansPerSecond), 3),
                                null);

                builder.addDoubleProperty(
                                "Pose Estimate X",
                                () -> GoatMath.round(poseEstimate.get().getX(), 6),
                                null);
                builder.addDoubleProperty(
                                "Pose Estimate Y",
                                () -> GoatMath.round(poseEstimate.get().getY(), 6),
                                null);
                builder.addDoubleProperty(
                                "Pose Estimate Theta",
                                () -> GoatMath.round(poseEstimate.get().getRotation().getDegrees(), 6),
                                null);

                builder.addDoubleProperty(
                                "Priority Target Distance",
                                () -> {
                                        var priorityTarget = priorityTargetDistance.get();
                                        if (priorityTarget.isPresent()) {
                                                return GoatMath.round(priorityTarget.get(), 6);
                                        } else {
                                                return 0.0;
                                        }
                                },
                                null);

                builder.addDoubleProperty(
                                "Priority Target Angle (deg)",
                                () -> {
                                        var priorityTarget = priorityTargetRotation.get();
                                        if (priorityTarget.isPresent()) {
                                                return GoatMath.round(priorityTarget.get().getDegrees(), 6);
                                        } else {
                                                return 0.0;
                                        }
                                },
                                null);

                for (var fieldDetectorPosition : fieldDetectorsPositions) {

                        builder.addDoubleProperty(
                                        "Field X from " + fieldDetectorPosition.getFirst(),
                                        () -> {
                                                Optional<Pose2d> pose = fieldDetectorPosition.getSecond().get();
                                                if (pose.isPresent()) {
                                                        return GoatMath.round(pose.get().getX(), 3);
                                                } else {
                                                        return Double.NaN;
                                                }
                                        },
                                        null);
                        builder.addDoubleProperty(
                                        "Field Y from " + fieldDetectorPosition.getFirst(),
                                        () -> {
                                                Optional<Pose2d> pose = fieldDetectorPosition.getSecond().get();
                                                if (pose.isPresent()) {
                                                        return GoatMath.round(pose.get().getY(), 3);
                                                } else {
                                                        return Double.NaN;
                                                }
                                        }, null);
                        builder.addDoubleProperty(
                                        "Field Theta from " + fieldDetectorPosition.getFirst(),
                                        () -> {
                                                Optional<Pose2d> pose = fieldDetectorPosition.getSecond().get();
                                                if (pose.isPresent()) {
                                                        return GoatMath.round(pose.get().getRotation().getDegrees(), 3);
                                                } else {
                                                        return Double.NaN;
                                                }
                                        }, null);

                }

                for (var fieldDetectorLatency : fieldDetectorLatencies) {

                        builder.addDoubleProperty(
                                        "Field Latency from " + fieldDetectorLatency.getFirst(),
                                        () -> {
                                                Optional<Measure<Time>> latency = fieldDetectorLatency.getSecond()
                                                                .get();
                                                if (latency.isPresent()) {
                                                        return GoatMath.round(latency.get().in(Seconds), 3);
                                                } else {
                                                        return Double.NaN;
                                                }
                                        },
                                        null);
                }

                for (var objectDetectorPosition : objectPositions) {
                        builder.addDoubleProperty(
                                        "Object X from " + objectDetectorPosition.getFirst(),
                                        () -> {
                                                Optional<Pose2d> pose = objectDetectorPosition.getSecond().get();
                                                if (pose.isPresent()) {
                                                        return GoatMath.round(pose.get().getX(), 3);
                                                } else {
                                                        return Double.NaN;
                                                }
                                        }, null);
                        builder.addDoubleProperty(
                                        "Object Y from " + objectDetectorPosition.getFirst(),
                                        () -> {
                                                Optional<Pose2d> pose = objectDetectorPosition.getSecond().get();
                                                if (pose.isPresent()) {
                                                        return GoatMath.round(pose.get().getY(), 3);
                                                } else {
                                                        return Double.NaN;
                                                }
                                        }, null);
                        builder.addDoubleProperty(
                                        "Object Theta from " + objectDetectorPosition.getFirst(),
                                        () -> {
                                                Optional<Pose2d> pose = objectDetectorPosition.getSecond().get();
                                                if (pose.isPresent()) {
                                                        return GoatMath.round(pose.get().getRotation().getDegrees(), 3);
                                                } else {
                                                        return Double.NaN;
                                                }
                                        }, null);

                }

        }

        public static final Supplier<TelemetrySubsystem> create = () -> {

                AHRS navxMXP2 = new AHRS(Port.kMXP);
                Gyroscope gyroscope = Gyroscope.KauaiLabs.createNavxXMP.apply(navxMXP2,
                                RobotContainer.driveSubsystem.robotSpeeds);
                Telemetry telemetry = Telemetry.create
                                .apply(gyroscope)
                                .apply(DriveSubsystem.Constants.kinematics)
                                .apply(RobotContainer.driveSubsystem.swerveDriveWheelPositions);

                Constants.objectDetectorNamesPositions.forEach(
                                (namePosition) -> Telemetry.addObjectDetectorToTelemetry
                                                .apply(ObjectDetector.Limelight.createLimelight
                                                                .apply(namePosition.getFirst())
                                                                .apply(namePosition.getSecond()))
                                                .apply(telemetry));

                Constants.fieldDetectorNames.forEach(
                                (namePosition) -> Telemetry.addFieldDetectorToTelemetry
                                                .apply(FieldDetector.Limelight.createLimelight
                                                                .apply(namePosition.getFirst())
                                                                .apply(namePosition.getSecond())
                                                                .apply(telemetry.poseEstimate))
                                                .apply(telemetry));

                Supplier<ChassisSpeeds> fieldSpeeds = () -> {
                        return ChassisSpeeds.fromRobotRelativeSpeeds(RobotContainer.driveSubsystem.robotSpeeds.get(),
                                        telemetry.poseEstimate.get().getRotation());
                };

                Runnable resetPoseEstimateFromFieldDetectors = () -> {

                };

                Runnable update = () -> {
                        telemetry.update.run();
                        // if (!RobotBase.isSimulation()) {
                        if (true) {
                                for (int i = 0; i < telemetry.fieldDetectorOptPositions.size(); i++) {
                                        var optPose = telemetry.fieldDetectorOptPositions.get(i).getSecond().get();
                                        var optLatency = telemetry.fieldDetectorLatencies.get(i).getSecond().get();
                                        var area = telemetry.averageAreaSuppliers.get(i);
                                        if (optPose.isPresent() && optLatency.isPresent()) {
                                                var poseEstimate = telemetry.poseEstimate.get();
                                                var pose = optPose.get();
                                                var latency = optLatency.get();
                                                var translationGood = pose.getTranslation()
                                                                .minus(poseEstimate.getTranslation())
                                                                .getNorm() <= Constants.poseTranslationToleranceMeters;
                                                var rotationGood = pose.getRotation().minus(poseEstimate.getRotation())
                                                                .getDegrees() <= Constants.poseRotationToleranceDegrees;
                                                if (translationGood && rotationGood && telemetry.fieldDetectorTagCounts
                                                                .get(i).getSecond().get() >= 2 && area.get() != 0.0) {
                                                        telemetry.addDetectedPosesToEstimator.apply(pose)
                                                                        .accept(latency);
                                                        SmartDashboard.putNumber("Pose Added from "
                                                                        + telemetry.fieldDetectorOptPositions.get(i)
                                                                                        .getFirst()
                                                                        + " at Time Index",
                                                                        Timer.getFPGATimestamp());
                                                }
                                        }
                                }
                        }

                };

                return new TelemetrySubsystem(
                                telemetry.accelerationX,
                                telemetry.accelerationY,
                                telemetry.field2d,
                                fieldSpeeds,
                                telemetry.poseEstimate,
                                telemetry.fieldDetectorOptPositions,
                                telemetry.fieldDetectorLatencies,
                                telemetry.objectDetectorOptPositions,
                                telemetry.priorityTargetDistance,
                                telemetry.priorityTargetRotation,
                                telemetry.setPoseEstimate,
                                resetPoseEstimateFromFieldDetectors,
                                telemetry.setPriorityTargetsFromFieldDetectors,
                                telemetry.enableFieldDetectors,
                                telemetry.enableObjectDetectors,
                                update);
        };
}
