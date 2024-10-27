package com.compLevel0;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.RobotBase;

public class ObjectDetector {

    public final String name;
    public final Supplier<Optional<Translation2d>> robotSpaceObjectTranslation;
    public final Supplier<Optional<Measure<Time>>> latency;
    public final Runnable enable;
    public final Runnable update;

    private ObjectDetector(
            String name,
            Supplier<Optional<Translation2d>> robotSpaceObjectTranslation,
            Supplier<Optional<Measure<Time>>> latency,
            Runnable enable,
            Runnable update) {
        this.name = name;
        this.robotSpaceObjectTranslation = robotSpaceObjectTranslation;
        this.latency = latency;
        this.enable = enable;
        this.update = update;
    }

    public static final class Limelight {
        public static Function<String, Function<Pose3d, ObjectDetector>> createLimelight = (
                name) -> (cameraPosition) -> {

                    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
                    NetworkTableEntry validTargetSupplier = table.getEntry("tv");
                    NetworkTableEntry pipelineSupplier = table.getEntry("getpipe");
                    NetworkTableEntry txSupplier = table.getEntry("tx");
                    NetworkTableEntry tySupplier = table.getEntry("ty");
                    NetworkTableEntry tlSupplier = table.getEntry("tl");
                    NetworkTableEntry clSupplier = table.getEntry("cl");
                    NetworkTableEntry pipelineConsumer = table.getEntry("pipeline");

                    Supplier<Double> txDegreesSupplier = () -> {
                        if (RobotBase.isSimulation()) {
                            return 5.0; 
                        } else {
                            return -txSupplier.getDouble(0);
                        }
                    };

                    Supplier<Double> tyDegreesSupplier = () -> {
                        if (RobotBase.isSimulation()) {
                            return 15.0;
                        } else {
                            return tySupplier.getDouble(0);
                        }
                    };

                    MutableMeasure<Time> latencyMeasure = MutableMeasure.zero(Milliseconds);
                    Translation2d cameraTranslationRobotSpace = new Translation2d(cameraPosition.getX(),
                            cameraPosition.getY());
                    Supplier<Optional<Translation2d>> robotSpaceObjectTranslation = () -> {
                        double txDegrees = txDegreesSupplier.get();
                        double tyDegrees = tyDegreesSupplier.get();
                        double tv = RobotBase.isSimulation() ? 1.0 : validTargetSupplier.getDouble(0.0);
                        double pipeline = RobotBase.isSimulation() ? 0.0 : pipelineSupplier.getDouble(0.0);
                        if (tv == 1 && pipeline == 0) {
                            double cameraZ = cameraPosition.getZ();
                            double cameraYaw = cameraPosition.getRotation().getZ();
                            double distance = Math.abs(cameraZ / Math.tan(Math.toRadians(Math.abs(tyDegrees) + 7)));
                            double objectYCamSpace = distance * Math.sin(Math.toRadians(txDegrees));
                            double objectXCamSpace = distance * Math.cos(Math.toRadians(txDegrees));
                            Translation2d objectCamSpace = new Translation2d(objectXCamSpace, objectYCamSpace);
                            Translation2d objectCamSpaceRotatedIntoRobotFrame = objectCamSpace
                                    .rotateBy(Rotation2d.fromRadians(cameraYaw));
                            Translation2d objectTranslationRobot = cameraTranslationRobotSpace
                                    .plus(objectCamSpaceRotatedIntoRobotFrame);
                            return Optional.of(objectTranslationRobot);
                        } else {
                            return Optional.empty();
                        }
                    };

                    Supplier<Optional<Measure<Time>>> latency = () -> {
                        if (RobotBase.isSimulation()) {
                            return Optional.of(Milliseconds.of(30));
                        }
                        if (validTargetSupplier.getInteger(0) == 1 && pipelineSupplier.getInteger(0) == 0) {
                            return Optional.of(Milliseconds.of(latencyMeasure.in(Milliseconds)));
                        } else {
                            return Optional.empty();
                        }
                    };

                    Runnable enable = () -> pipelineConsumer.setNumber(0);

                    Runnable update = () -> {
                        latencyMeasure.mut_setMagnitude(tlSupplier.getDouble(0) + clSupplier.getDouble(0));
                    };

                    return new ObjectDetector(
                            name,
                            robotSpaceObjectTranslation,
                            latency,
                            enable,
                            update);
                };
    }

}
