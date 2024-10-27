package frc.robot.triggers;

import static frc.robot.RobotContainer.notedLoadedSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ComplexTriggers {

        private ComplexTriggers() {
        }

        public static final Trigger fieldCentricOriginDriveTrigger = (new Trigger(RobotContainer.driver.leftX)
                        .or(RobotContainer.driver.leftY)).and(RobotContainer.driver.x.negate())
                        .and(RobotContainer.driver.rightBumper.negate());

        public static final Trigger fieldCentricFLDriveTrigger = fieldCentricOriginDriveTrigger
                        .and(RobotContainer.driver.padLeft);
        public static final Trigger fieldCentricFRDriveTrigger = fieldCentricOriginDriveTrigger
                        .and(RobotContainer.driver.padRight);

        public static final Trigger fieldCentricRotateAlongHeadingDriveTrigger = fieldCentricOriginDriveTrigger
                        .and(RobotContainer.driver.y);

        public static final Trigger robotCentricOriginDriveTrigger = fieldCentricOriginDriveTrigger.negate().and(
                        RobotContainer.driver.rightX.or(RobotContainer.driver.rightY)
                                        .or(RobotContainer.driver.fullTrigger))
                        .and(RobotContainer.driver.x.negate()).and(RobotContainer.driver.rightBumper.negate());

        public static final Trigger robotCentricFLDriveTrigger = robotCentricOriginDriveTrigger
                        .and(RobotContainer.driver.padLeft);
        public static final Trigger robotCentricFRDriveTrigger = robotCentricOriginDriveTrigger
                        .and(RobotContainer.driver.padRight);
        private static final BooleanSupplier hasNote = () -> {
                return notedLoadedSubsystem.hasNoteBB.getAsBoolean() || notedLoadedSubsystem.hasNote.getAsBoolean();
        };
        private static final BooleanSupplier hasNote2 = () -> {
                return notedLoadedSubsystem.hasNote2.getAsBoolean();
        };
        public static final Trigger noteLoadedTrigger = new Trigger(hasNote);
        public static final Trigger noteLoadedTrigger2 = new Trigger(hasNote2);

        // public static final Trigger templateTrigger = new Trigger(null);

}
