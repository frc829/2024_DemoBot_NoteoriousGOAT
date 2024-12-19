package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.Gyroscope;
import com.compLevel1.Telemetry;
import com.kauailabs.navx.frc.AHRS;
import com.utility.GoatMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TelemetrySubsystem extends SubsystemBase {

        public static final class Constants {
                public static final double poseTranslationToleranceMeters = 1;
                public static final double poseRotationToleranceDegrees = 2;
        }

        public final Measure<Velocity<Velocity<Distance>>> accelerationX;
        public final Measure<Velocity<Velocity<Distance>>> accelerationY;
        public final Field2d field2d;
        public final Supplier<ChassisSpeeds> fieldSpeeds;
        public final Supplier<Pose2d> poseEstimate;
        public final Consumer<Pose2d> setPoseEstimator;
        public final Runnable update;

        private TelemetrySubsystem(
                        Measure<Velocity<Velocity<Distance>>> accelerationX,
                        Measure<Velocity<Velocity<Distance>>> accelerationY,
                        Field2d field2d,
                        Supplier<ChassisSpeeds> fieldSpeeds,
                        Supplier<Pose2d> poseEstimate,
                        Consumer<Pose2d> setPoseEstimator,
                        Runnable update) {
                this.accelerationX = accelerationX;
                this.accelerationY = accelerationY;
                this.field2d = field2d;
                this.fieldSpeeds = fieldSpeeds;
                this.poseEstimate = poseEstimate;
                this.setPoseEstimator = setPoseEstimator;
                this.update = update;
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

        }

        public static final Supplier<TelemetrySubsystem> create = () -> {

                AHRS navxMXP2 = new AHRS(Port.kMXP);
                Gyroscope gyroscope = Gyroscope.KauaiLabs.createNavxXMP.apply(navxMXP2,
                                RobotContainer.driveSubsystem.robotSpeeds);
                Telemetry telemetry = Telemetry.create
                                .apply(gyroscope)
                                .apply(DriveSubsystem.Constants.kinematics)
                                .apply(RobotContainer.driveSubsystem.swerveDriveWheelPositions);


                Supplier<ChassisSpeeds> fieldSpeeds = () -> {
                        return ChassisSpeeds.fromRobotRelativeSpeeds(RobotContainer.driveSubsystem.robotSpeeds.get(),
                                        telemetry.poseEstimate.get().getRotation());
                };


                Runnable update = () -> {
                        telemetry.update.run();
                };

                return new TelemetrySubsystem(
                                telemetry.accelerationX,
                                telemetry.accelerationY,
                                telemetry.field2d,
                                fieldSpeeds,
                                telemetry.poseEstimate,
                                telemetry.setPoseEstimate,
                                update);
        };
}
