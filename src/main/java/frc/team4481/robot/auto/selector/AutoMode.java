package frc.team4481.robot.auto.selector;

import edu.wpi.first.wpilibj.DriverStation;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface AutoMode {
    DriverStation.Alliance[] alliance() default {DriverStation.Alliance.Red, DriverStation.Alliance.Blue};
    String displayName();
}