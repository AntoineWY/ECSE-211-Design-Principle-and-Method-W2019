package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Display;

import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.SENSOR_MOTOR;
import static ca.mcgill.ecse211.project.project.TRACK;
import static ca.mcgill.ecse211.project.project.WHEEL_RAD;

import java.util.Stack;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.remote.nxt.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * THIS VERSION IS THE ONE TO BE USED IN THE FINAL DEMO! <P>
 * This is the main class from which the whole project is run. It will perform every step needed to complete the final
 * demo. It serves as a link to the rest of the classes.<p>
 * The procedure is:<br>
 * 1. Define all the global constant which will later be used in other classes. Some of the values like TRACK, WHEEL_RAD and 
 * TILE are crucial for the vehicle to navigate.<br>
 * 2. Allocate ports and get sensors and motors.<br>
 * 3. Construct essential istances from each classes, like localization class and odometer<br>
 * 4. Receive data from the WIFI class and pass the value to the corresponding public fields which will be used in other classes<br>
 * 5. Start odometer thread. Perform tasks sequentially by calling specfic classes. The sequence is: localization, navigation to the search zone,
 * search, navigate back, drop can.
 * 
 * @author TEAM 23
 */

public class project {
  
  /**
   * Size of one tile in cm
   */
  public static final double TILE = 31.48;

  /**
   * A constant factor that can be applied to convert angle units in radians to degrees
   */
  public static final double TO_DEG = 180.0 / Math.PI;

  /**
   * A constant factor that can be applied to convert angular units in degrees to radians
   */
  public static final double TO_RAD = Math.PI / 180.0;

  /**
   * The radius (in cm) of the left/right wheels of the EV3 robot.
   */
  public static final double WHEEL_RAD = 2.15; 

  /**
   * The width (in cm) of the robot measured from the center of the left wheel to the center of the
   * right wheel
   */
  public static final double TRACK = 15.95; 

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  public static final int SMOOTH_ACCELERATION = 500;

  /**
   * Specifies the speed of the left and right EV3 Large motors
   */
  public static final int SPEED = 100;

  /**
   * The heading/Theta value of the robot initially
   */
  public static final int INITIAL_ANGLE = 0;

  /**
   * A revolution of half of a circle in degrees
   */
  public static final int HALF_CIRCLE = 180;

  /**
   * A full revolution of a circle in degrees
   */
  public static final int FULL_CIRCLE = 360;

  /**
   * Measured value for blue color. The RGB values are normalized rgb reading and they serve as the threshold to compare with the 
   * later read can to identify colors.
   */
  public static final double[] BLUE_COLOR = {0.19, 0.70, 0.30}; // value of blue colour
  
  /**
   * Measured value for green color.
   * @see project#BLUE_COLOR
   */
  public static final double[] GREEN_COLOR = {0.35, 0.85, 0.20}; // value of green colour
  
  /**
   * Measured value for yellow color.
   * @see project#BLUE_COLOR
   */
  public static final double[] YELLOW_COLOR = {0.85, 0.52, 0.13}; // value of yellow colour
  
  /**
   * Measured value for red color.
   * @see project#BLUE_COLOR
   */
  public static final double[] RED_COLOR = {0.98, 0.24, 0.07}; // value of red colour

  //params which is need for the game. Will be passed in by WIFI class
  public static int corner;
  public static int team_LL_x;
  public static int team_LL_y;
  public static int team_UR_x;
  public static int team_UR_y;

  public static int tunnel_LL_x;
  public static int tunnel_LL_y;
  public static int tunnel_UR_x;
  public static int tunnel_UR_y;
  
  public static int zone_LL_x;
  public static int zone_LL_y;
  public static int zone_UR_x;
  public static int zone_UR_y;
  
  public static int Island_LL_x; //lower left corner island y
  public static int Island_LL_y; //lower left corner island x
  public static int Island_UR_x; //upper right corner island y
  public static int Island_UR_y; //upper right corner island x
  
  
  /**
   * The instance of the left wheel large EV3 motor. The left motor is connected to port A on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The instance of the right wheel large EV3 motor. The right motor is connected to port D on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The instance of the medium motor that turns the sensor. The motor is connected to port B on the
   * EV3 brick.
   */
  public static final EV3MediumRegulatedMotor SENSOR_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * Port for ultrasonic sensor.
   */
  private static final Port US_PORT = LocalEV3.get().getPort("S4");
   
  /**
   * Port for right color sensor for localization.
   */
  private static final Port RIGHT_PORT = LocalEV3.get().getPort("S2");  
  
  /**
   * Port for left  color sensor for localization.
   */
  private static final Port LEFT_PORT = LocalEV3.get().getPort("S1");

  /**
   * Port for color sensor for can classification.
   */
  private static final Port COLOR_PORT = LocalEV3.get().getPort("S3");

  /**
   * Stack to store important points (where localization are performed) travelled sequentially 
   */
  public static Stack<int[]> keyPoints = new Stack<int[]>();
  
  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  /**
   * The main method. This method is used to start threads and execute the main function of the
   * robot during the final demo. 
   * 
   * @param args - arguments to pass in
   * @throws OdometerExceptions - multiple odometer instances
   */
  public static void main(String[] args) throws OdometerExceptions {

 
    // set up odometer
    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    // US sensor initialization
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];
    
    // light sensor initialization, for can classification
    @SuppressWarnings("resource")
    SensorModes lightSensor = new EV3ColorSensor(COLOR_PORT);
    SampleProvider lightColor = ((EV3ColorSensor) lightSensor).getRGBMode();
    float[] lightData = new float[3];

    // left color sensor for localization initialization
    @SuppressWarnings("resource")
    SensorModes Left_Sensor = new EV3ColorSensor(LEFT_PORT);
    SampleProvider left = Left_Sensor.getMode("Red");
    float[] leftcsData = new float[left.sampleSize()];

    // right color sensor for localization initialization
    @SuppressWarnings("resource")
    SensorModes Right_Sensor = new EV3ColorSensor(RIGHT_PORT);
    SampleProvider right = Right_Sensor.getMode("Red");
    float[] rightcsData = new float[right.sampleSize()];
    
    
      // WIFI ACQUISITION OF DATA
      Wifi.main(args);
      
      // We are team #23
      if (Wifi.RedTeam == 23) {
        corner = Wifi.RedCorner;
        team_LL_x = Wifi.Red_LL_x;
        team_LL_y = Wifi.Red_LL_y;
        team_UR_x = Wifi.Red_UR_x;
        team_UR_y = Wifi.Red_UR_y;
        tunnel_LL_x = Wifi.TNR_LL_x;
        tunnel_LL_y = Wifi.TNR_LL_y;
        tunnel_UR_x = Wifi.TNR_UR_x;
        tunnel_UR_y = Wifi.TNR_UR_y;
        zone_LL_x = Wifi.SZR_LL_x;
        zone_LL_y = Wifi.SZR_LL_y;
        zone_UR_x = Wifi.SZR_UR_x;
        zone_UR_y = Wifi.SZR_UR_y;
        Island_LL_x = Wifi.Island_LL_x;
        Island_LL_y = Wifi.Island_LL_y;
        Island_UR_x = Wifi.Island_UR_x;
        Island_UR_y = Wifi.Island_UR_y;
        
        
      }
      else if (Wifi.GreenTeam == 23) {
        corner = Wifi.GreenCorner;
        team_LL_x = Wifi.Green_LL_x;
        team_LL_y = Wifi.Green_LL_y;
        team_UR_x = Wifi.Green_UR_x;
        team_UR_y = Wifi.Green_UR_y;
        tunnel_LL_x = Wifi.TNG_LL_x;
        tunnel_LL_y = Wifi.TNG_LL_y;
        tunnel_UR_x = Wifi.TNG_UR_x;
        tunnel_UR_y = Wifi.TNG_UR_y;
        zone_LL_x = Wifi.SZG_LL_x;
        zone_LL_y = Wifi.SZG_LL_y;
        zone_UR_x = Wifi.SZG_UR_x;
        zone_UR_y = Wifi.SZG_UR_y;
        Island_LL_x = Wifi.Island_LL_x;
        Island_LL_y = Wifi.Island_LL_y;
        Island_UR_x = Wifi.Island_UR_x;
        Island_UR_y = Wifi.Island_UR_y;
      }
    
      //Start odometer thread
      (new Thread(odometer)).start();
  
      // create instances of important classes
      DoubleLightLocalization dll = new DoubleLightLocalization(odometer,left, right, leftcsData, rightcsData); // light localization instance
      
      NavigationWithCorr navWc = new NavigationWithCorr(odometer,left,right, leftcsData, rightcsData,dll );// navigation class instance
     
      Handling handler = new Handling(); //handling instance
      
      WeightID2 weightid = new WeightID2 ( left, leftcsData, dll, navWc);// weight Identification instance
      
      USLocalizer ul = new USLocalizer(odometer, LEFT_MOTOR, RIGHT_MOTOR, usDistance); // ultrasonic localization instance
      
      SweepSearch sweepsearch = new SweepSearch(odometer, usDistance, usData, lightColor, lightData, 
    		  navWc, handler, weightid); // search instance
                                                                                                           
      //Perform ultrasonic and light localization and issue 3 beeps after finish
      ul.localize();
      dll.DoubleLocalizer();
      beepNTimes(3);
         
      //now that the robot is at the gridline, update odometer based on corner number
      switch (corner) {
        case 0: 
          odometer.setXYT(1 * TILE, 1 * TILE, 0);        
          break;
        case 1:
          odometer.setXYT(14 * TILE, 1 * TILE, 270);
          break;
        case 2:
          odometer.setXYT(14 * TILE, 8 * TILE, 180);
          break;
        case 3:
          odometer.setXYT(1 * TILE, 8 * TILE, 90);        
      } //theta might have to be changed if not the same reference system as in lab 5

     // Judge if the tunnel is vertical
     boolean isTunV = isTunnelVertical();
      
     //Close the gripper on the way
     SENSOR_MOTOR.rotateTo(90);
     
     // Go to the other side of the tunnel on the search island
     int caseFlag = MapPlanner.toIsland(isTunV, navWc, dll, odometer);
     
     // Heading to the search point. Once arrivedm beep 3 times
     int[] searchPoint =MapPlanner.toSearchZone(navWc, odometer);
     beepNTimes(3);
     
     //open the claw and get ready to search
     SENSOR_MOTOR.rotateTo(0);
    
     // Search, identify the can and acquire it
     sweepsearch.search(searchPoint);

     // Driving back to the team zone with the can, through the tunnel
     MapPlanner.backToTeamZone(caseFlag, navWc, odometer);
     
     //Back to the srating block to place the can
     MapPlanner.backToStartingBlock(navWc);
     
     //Drop can and issue 5 beep
     handler.release(true);
     beepNTimes(5);
     
     // Re-localize at the starting point, get ready for next search
     MapPlanner.toNextSearch(navWc);
     
      // exit when esc pressed
      while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      }
      System.exit(0); // exit program after esc pressed
    }
  
  
  // -----------------------------------------------------------------------------
  // Private Method
  // -----------------------------------------------------------------------------

  /**
   * Using the LL corner and UR corner of the tunnel to calculate if the tunnel is vertical
   * @return boolean whether the tunnel is vertical
   */
  private static boolean isTunnelVertical(){
	  if (Math.abs(tunnel_UR_x - tunnel_LL_x) == 2){
		  return false;
	  }else if (Math.abs(tunnel_UR_x - tunnel_LL_x) == 1){
		  return true;
	  }
	  else{
		  Button.waitForAnyPress();
		  return false;
		 
	  }
  }
  
  /**
   * Helper method to let the robot to beep for arbitrary times
   * @param times 
   */
  private static void beepNTimes(int times){
	  for (int i = 0; i < times; i++){
		  Sound.beep();
	  }
  	}

}
