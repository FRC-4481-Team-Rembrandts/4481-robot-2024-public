package frc.team4481.robot.auto.selector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.robot.auto.modes.DoNothingMode;
import org.reflections.Reflections;

import java.lang.reflect.InvocationTargetException;
import java.util.Set;

/**
 * Class for selecting of autonomous mode via {@code SendableChooser}. AutoModes can be added by annotating them
 * with the {@code @AutoMode} annotation. By supplying a {@code @Disabled} annotation, the AutoMode will not show up.
 */
public class AutoModeSelector {

    private final SendableChooser<AutoModeBase> modeChooser;

    /**
     * Creates an autonomous mode selector which is visible on the driver station.
     */
    public AutoModeSelector() {
        modeChooser = new SendableChooser<>();
        modeChooser.setDefaultOption("[All] Do Nothing", new DoNothingMode());

        // Get all autoModes
        try {
            addAutoModes(fetchAutoModes());
        } catch (Exception e) {
            e.printStackTrace();
        }

        SmartDashboard.putData(modeChooser);
    }

    public AutoModeBase getAutoMode() {return modeChooser.getSelected();}

    /**
     * Gets a set of all annotated AutoModes that do not have the {@code @Disabled} annotation.
     *
     * @return set of all enabled AutoModes
     */
    private Set<Class<? extends AutoModeBase>> fetchAutoModes() {
        Reflections reflections = new Reflections("frc.team4481.robot.auto.modes");
        return reflections.getSubTypesOf(AutoModeBase.class);
    }

    /**
     * Adds a set of annotated AutoModes to the {@code SendableChooser}.
     *
     * @param autoModes set of annotatedAutoModes
     * @throws NoSuchMethodException
     * @throws InvocationTargetException
     * @throws InstantiationException
     * @throws IllegalAccessException
     */
    private void addAutoModes(Set<Class<? extends AutoModeBase>> autoModes)
            throws NoSuchMethodException, InvocationTargetException, InstantiationException, IllegalAccessException
    {

        for (Class<? extends AutoModeBase> currentModeClass : autoModes) {
            if (currentModeClass.isAnnotationPresent(AutoMode.class) &&
                            !currentModeClass.isAnnotationPresent(Disabled.class) ) {
                if ( ( DriverStation.getAlliance().isPresent() &&
                        isAlliancePresent(currentModeClass.getAnnotation(AutoMode.class).alliance(), DriverStation.getAlliance().get()) )
                        || DriverStation.getAlliance().isEmpty() ) {
                    modeChooser.addOption(
                            currentModeClass.getAnnotation(AutoMode.class).displayName(),
                            currentModeClass.getDeclaredConstructor().newInstance()
                    );
                }
            }
        }
    }

    /**
     * Checks if the current alliance is in the provided list of alliances
     *
     * @param allianceList list of alliances
     * @param currentAlliance alliance to check for
     * @return whether the alliance is in the list
     */
    private boolean isAlliancePresent(DriverStation.Alliance[] allianceList, DriverStation.Alliance currentAlliance){
        for (DriverStation.Alliance alliance : allianceList) {
            if (alliance.equals(currentAlliance)) {
                return true;
            }
        }

        return false;
    }
}

