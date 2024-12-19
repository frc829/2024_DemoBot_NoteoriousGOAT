package com.compLevel1;

import static edu.wpi.first.units.Units.Degrees;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import java.util.function.Function;

import com.compLevel0.Gyroscope;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

        public final Field2d field2d;
        public final Measure<Angle> gyroYaw;
        public final Measure<Velocity<Velocity<Distance>>> accelerationX;
        public final Measure<Velocity<Velocity<Distance>>> accelerationY;
        public final Supplier<Pose2d> poseEstimate;
        public final Consumer<Pose2d> setPoseEstimate;
        public final Runnable update;
        public final List<Supplier<Double>> averageAreaSuppliers;

        private Telemetry(
                        Field2d field2d,
                        Measure<Angle> gyroYaw,
                        Measure<Velocity<Velocity<Distance>>> accelerationX,
                        Measure<Velocity<Velocity<Distance>>> accelerationY,
                        Supplier<Pose2d> poseEstimate,
                        Consumer<Pose2d> setPoseEstimate,
                        Runnable update, 
                        List<Supplier<Double>> averageAreaSuppliers) {
                this.field2d = field2d;
                this.gyroYaw = gyroYaw;
                this.accelerationX = accelerationX;
                this.accelerationY = accelerationY;
                this.poseEstimate = poseEstimate;
                this.setPoseEstimate = setPoseEstimate;
                this.update = update;
                this.averageAreaSuppliers = averageAreaSuppliers;
                SmartDashboard.putData(field2d);
        }

        public static final Function<Gyroscope, Function<SwerveDriveKinematics, Function<Supplier<SwerveDriveWheelPositions>, Telemetry>>> create = (
                        gyroscope) -> (kinematics) -> (wheelPositions) -> {

                                Field2d field2d = new Field2d();
                                SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                                                kinematics,
                                                Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                wheelPositions.get().positions,
                                                new Pose2d(),
                                                VecBuilder.fill(0.1, 0.1, 0.1),
                                                VecBuilder.fill(0.9, 0.9, 0.9));

                                Supplier<Pose2d> poseEstimate = () -> swerveDrivePoseEstimator.getEstimatedPosition();


                                List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions = new ArrayList<>();

                                Consumer<Pose2d> setPoseEstimate = (resetPose) -> {
                                        swerveDrivePoseEstimator.resetPosition(
                                                        Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                        wheelPositions.get().positions, resetPose);
                                };

                                List<Supplier<Double>> averageAreaSupplier = new ArrayList<>();

                                Runnable update = () -> {
                                        gyroscope.update.run();
                                        swerveDrivePoseEstimator.updateWithTime(
                                                        Timer.getFPGATimestamp(),
                                                        Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                        wheelPositions.get().positions);
                                        field2d.setRobotPose(poseEstimate.get());
                                        for (var objectDetectorOptPosition : objectDetectorOptPositions) {
                                                String name = objectDetectorOptPosition.getFirst();
                                                Optional<Pose2d> position = objectDetectorOptPosition
                                                                .getSecond().get();
                                                if (position.isPresent()) {
                                                        field2d.getObject(name + "-ObjectPose").setPose(position.get());
                                                } else {
                                                        field2d.getObject(name + "-ObjectPose")
                                                                        .setPose(new Pose2d(Double.NaN,
                                                                                        Double.NaN,
                                                                                        Rotation2d.fromDegrees(0)));
                                                }
                                        }

                                };

                                return new Telemetry(field2d,
                                                gyroscope.yaw,
                                                gyroscope.accelerationX,
                                                gyroscope.accelerationY,
                                                poseEstimate,
                                                setPoseEstimate,
                                                update, 
                                                averageAreaSupplier);
                        };
}
