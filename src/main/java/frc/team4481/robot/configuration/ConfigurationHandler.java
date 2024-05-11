package frc.team4481.robot.configuration;

import com.sun.source.doctree.TextTree;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;

/**
 * Singleton class which handles which
 * configuration is currently selected
 */
public class ConfigurationHandler {

    private static ConfigurationHandler instance = null;

    //Declare configuration options
//    private Configuration eventConfig = new EventConfig();
//    private Configuration compConfig = new CompetitionConfig();
    private Configuration testConfig = new TestConfig();

    //Set default configuration
    private Configuration config = testConfig;

    //Define enum for configuration options
    //The top option is the default option
    public enum ConfigOption {
        TEST,
//        COMPETITION,
//        EVENT
    }

    //Define the sendablechooser to select the configuration
    private final SendableChooser<String> chooser;

    private ConfigurationHandler(){
        chooser = new SendableChooser<>();

        int i = 0;
        for (ConfigOption option : ConfigOption.values()){
            if (i == 0){
                chooser.setDefaultOption(option.name().toLowerCase() + " configuration", option.name());
            } else {
                chooser.addOption(option.name().toLowerCase() + " configuration", option.name());
            }
            i ++;
        }
        SmartDashboard.putData(chooser);
    }

    public static ConfigurationHandler getInstance(){
        if(instance == null){
            instance = new ConfigurationHandler();
        }
        return instance;
    }


    /**
     * Update the selected configuration from the SmartDashboard and then
     * send the current configuration
     *
     * @return Configuration that is currently selected
     */
    public Configuration getConfig(){

        ConfigOption option = getSelectedOption();
        if (option != null){
            config = getConfigFromOption(option);
        }
        return config;
    }

    /**
     * Get the selected option in the SmartDashboard
     *
     * @return Selected ConfigOption in the SmartDashboard
     */
    private ConfigOption getSelectedOption(){
        //Fetch the chosen configuration
        String chosenOption = chooser.getSelected();

        for (ConfigOption option : ConfigOption.values()){
            if (chosenOption == option.name()){
                return option;
            }
        }
        return null;
    }


    /**
     * Get the Configuration from the input ConfigOption
     * @param option ConfigOption enum
     * @return Configuration corresponding to the ConfigOption
     */
    private Configuration getConfigFromOption(ConfigOption option){
        switch (option){
//            case EVENT:
//                return eventConfig;
//            case COMPETITION:
//                return compConfig;
            case TEST:
                return testConfig;
            default:
                return null;
        }
    }

    /**
     * Call the function of all the layouts to fetch the subsystemManagers
     */
    public void getLayoutSubsystemManagers(XboxController driver, ControlDevice operator){
        for (ConfigOption option : ConfigOption.values()){
            Configuration config = getConfigFromOption(option);
            HIDLayout layout = config.getHIDLayout(driver, operator);
            layout.getSubsystemManagers();
        }
    }
}
