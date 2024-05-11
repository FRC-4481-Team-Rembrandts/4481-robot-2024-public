/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team4481.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeExecutor;
import frc.team4481.lib.looper.DisabledSuperSubsystemLoop;
import frc.team4481.lib.looper.EnabledSuperSubsystemLoop;
import frc.team4481.lib.looper.Looper;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.robot.subsystems.Outtake;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.auto.selector.AutoModeSelector;
import frc.team4481.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * methods corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.<br>
 * <br>
 * This class, via its super class, provides the following methods which are called by the main loop:<br>
 * <br>
 *   - startCompetition(), at the appropriate times:<br>
 * <br>
 *   - robotInit() -- provide for initialization at robot power-on<br>
 * <br>
 * init methods -- each of the following methods is called once when the appropriate mode is entered:<br>
 *     - disabledInit()   -- called each and every time disabled is entered from another mode<br>
 *     - autonomousInit() -- called each and every time autonomous is entered from another mode<br>
 *     - teleopInit()     -- called each and every time teleop is entered from another mode<br>
 *     - testInit()       -- called each and every time test is entered from another mode<br>
 * <br>
 * periodic methods -- each of these methods is called on an interval:<br>
 *   - robotPeriodic()<br>
 *   - disabledPeriodic()<br>
 *   - autonomousPeriodic()<br>
 *   - teleopPeriodic()<br>
 *   - testPeriodic()<br>
 */
public class Robot extends TimedRobot
{
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
  private final Looper enabledLooper = new Looper();
  private final Looper disabledLooper = new Looper();

  private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
  private final HIDManager hidManager = HIDManager.getInstance();

  private AutoModeExecutor autoModeExecutor;
  private final AutoModeSelector autoModeSelector = new AutoModeSelector();
  private AutoModeBase previousAutoMode;

  @Override
  public void robotInit() {
    subsystemHandler.setSubsystems(
            new Drivetrain(),
            new Intake(),
            new Climber(),
            new Outtake(),
            new Utility()

    );
    hidManager.getSubsystemManagers(subsystemHandler);

    enabledLooper.register(new EnabledSuperSubsystemLoop(subsystemHandler));
    disabledLooper.register(new DisabledSuperSubsystemLoop(subsystemHandler));

//    preloadPaths();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("Log started");
  }

    /**
     * This method is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     * <p>
     * This runs after the mode specific periodic methods, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
      SmartDashboard.putBoolean("Is DriverStation attached", DriverStation.isDSAttached());
      SmartDashboard.putBoolean("Is FMS attached", DriverStation.isFMSAttached());
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

      enabledLooper.outputToSmartDashboard();
    }    

  /**
   * Initialization code for autonomous mode should go here.
   * <p>
   * Users should use this method for initialization code which will be called each time the
   * robot enters autonomous mode.
   */
    @Override
    public void autonomousInit()
    {
      DataLogManager.log("Start Auto init");
      disabledLooper.stop();

      enabledLooper.start();

      if (autoModeExecutor != null && autoModeExecutor.getAutoMode() != null)
        autoModeExecutor.start();

      DataLogManager.log("End Auto init");
    }

    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic()
    {
        
    }

    /**
     * Initialization code for teleop mode should go here.
     * <p>
     * Users should use this method for initialization code which will be called each time the
     * robot enters teleop mode.
     */
    @Override
    public void teleopInit()
    {
      disabledLooper.stop();
      if (autoModeExecutor != null) {
        autoModeExecutor.stop();
      }

      enabledLooper.start();
    }

    /**
     * This method is called periodically during operator control.
     */

    @Override
    public void teleopPeriodic()
    {
      hidManager.update();
    }

  /**
   * Initialization code for test mode should go here.
   * <p>
   * Users should use this method for initialization code which will be called each time the
   * robot enters test mode.
   */
    @Override
    public void testInit()
    {
//      disabledLooper.stop();
//      if (autoModeExecutor != null) {
//        autoModeExecutor.stop();
//      }
//
//      enabledLooper.start();
    }

    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic()
    {
        
    }

  @Override
  public void disabledInit()
  {
    enabledLooper.stop();
    if (autoModeExecutor != null) {
      autoModeExecutor.stop();
    }

    autoModeExecutor = new AutoModeExecutor();

    disabledLooper.start();
  }

  @Override
  public void disabledPeriodic()
  {
    AutoModeBase autoMode = autoModeSelector.getAutoMode();

    if (autoMode != null){
      autoModeExecutor.setAutoMode(autoMode);
      if (previousAutoMode != autoMode){
        autoMode.init();
      }
      previousAutoMode = autoMode;
    }

  }

  /**
   * Preloads Path Planner 2 paths to minimize downtime during auto
   */
//  private void preloadPaths() {
//    TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
//  }

}
