package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Lab5.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.BLUE_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.YELLOW_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.GREEN_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.RED_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.SENSOR_MOTOR;
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Navigator;
import ca.mcgill.ecse211.lab5.Navigation;

/**
 * 
 * @author Yinuo Wang
 * @author Raymond Yang
 * @author Tudor Gurau
 *
 */
public class Search extends Thread {


  private final int SCAN_TIME = 100;
  // {R, G, B} values based on in lab measurements and multiplied by 100
  public static final int LLx = 1; // lower left x coordinate of searching area, modify during demo
  public static final int LLy = 1; // lower left y coordinate of searching area, modify during demo
  public static final int URx = 3; // lower left x coordinate of searching area, modify during demo
  public static final int URy = 3; // lower left y coordinate of searching area, modify during demo

  private static final int RED_INDEX = 4;
  private static final int GREEN_INDEX = 2;
  private static final int BLUE_INDEX = 1;
  private static final int YELLOW_INDEX = 3;

  private static final int TR = 1; // colour of target can: must be changed during demo
  public static final double CAN_EXISTS = 15; // distance to show there is a can at that
                                              // intersection, TODO: tweak in lab

  // private static double[] threshold; // colour threshold to identify correct can
  private Odometer odometer; // odometer
  private float[] usData;
  private SampleProvider usDistance;
  private float[] lightData;
  private SampleProvider lightColor;
  private float[] csData;
  private SampleProvider csColor;
  private int filterSum;
  private double csfilterSum;
  private int std;
  private boolean isCan;
  private Navigation nav;
  private double[] colorData;
  private CanCalibrator calibrator;

  public static double distDisplay;

  /**
   * Constructor of the search class Takes in the sample provider and the data array of 3 sensors
   * (us, light sensor for detecting color and light sensor for localization) and the odometer. An
   * instance of Navigation class using the odometer and Us data is also created so that during
   * search the methods from Navigation classes can be implemented
   * 
   * @param odometer
   * @param usDistance
   * @param usData
   * @param lightColor
   * @param lightData
   * @param csColor
   * @param csData
   */
  public Search(Odometer odometer, SampleProvider usDistance, float[] usData,
      SampleProvider lightColor, float[] lightData, SampleProvider csColor, float[] csData) {
    this.odometer = odometer;
    this.usData = usData;
    this.usDistance = usDistance;
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.csData = csData;
    this.csColor = csColor;
    this.isCan = false;
    this.colorData = new double[4];
    nav = new Navigation(odometer, usDistance, usData);

    // // determine which color from TR given
    // switch (TR) {
    // case 1: // TR = 1 --> blue can
    // threshold = BLUE_COLOR;
    // break;
    // case 2: // TR = 2 --> green can
    // threshold = GREEN_COLOR;
    // break;
    // case 3: // TR = 3 --> yellow can
    // threshold = YELLOW_COLOR;
    // break;
    // case 4: // TR = 4 --> red can
    // threshold = RED_COLOR;
    // }
  }

  /**
   * This method runs the entity of the class. The robot will traverse the entire searching area in
   * a zig-zag manner, reaching every points. On the way it will detect the distance to identify the
   * can. After a can is presented, it will use the sensor motor to rotate the light sensor around
   * the can for 7 readings. Then based on whether the can met is the target, corresponding
   * reactions will be performed. In the end of every even rows the cart will perform a light
   * localization to offset the errors.
   */
  public void run() {
    boolean targetFound = false;

    for (int y = LLy; y <= URy; y++) {
      int x = 0;


      targetFound = false;

      if ((y - LLy) % 2 == 0) {
        for (x = LLx; x <= URx; x++) {
          try {
            Thread.sleep(150);
          } catch (Exception e) {
            // do nothing
          }
          targetFound = false;
          while (targetFound == false) {
            isCan = nav.travelTo(x, y);
            if (isCan) {
              targetFound = canFound();
            } else {
              targetFound = true;
            }
          }
        }
        lineLocalRight(x, y, 0);
      } else {
        if ((y - LLy) % 2 == 1) {
          for (x = URx; x >= LLx; x--) {
            try {
              Thread.sleep(150);
            } catch (Exception e) {
              // do nothing
            }
            targetFound = false;
            while (targetFound == false) {

              isCan = nav.travelTo(x, y);
              if (isCan) {
                targetFound = canFound();
              } else {
                targetFound = true;
              }
            }
          }
          lineLocalRight(x, y, 1);
        }
      }



    }
  }

  /**
   * This class defines the reaction that the robot meets the can. 1. It approach the can by 10cm to
   * offset the "same distance" from the US sensor to the target 2. The color sensor arm reads
   * around the can to identify the color 3. If the read color is same as the target (target found),
   * then the can will be hit to the side the robot then travels to the end point and finishes the
   * task. 4. If the read color is different, the can got pushed away and the cart continues to the
   * next point
   * 
   * @return boolean Indicating whether the target is found (whether the cart needs to still
   *         traverse the map in a loop)
   */
  private boolean canFound() {
    // double[] red_array = new double[100]; // array for reds
    // double[] green_array = new double[100]; // array for greens
    // double[] blue_array = new double[100]; // array for blues

    calibrator = new CanCalibrator(lightColor, lightData);
    // double starting_angle = odometer.getXYT()[2];

    LEFT_MOTOR.rotate(convertDistance(Lab5.WHEEL_RAD, 10), true);
    RIGHT_MOTOR.rotate(convertDistance(Lab5.WHEEL_RAD, 10), false);

    // check if color reading is correct
    int color = rotateSensorDetect();

    LEFT_MOTOR.rotate(-convertDistance(Lab5.WHEEL_RAD, 7), true);
    RIGHT_MOTOR.rotate(-convertDistance(Lab5.WHEEL_RAD, 7), false);

    if (color == colorconvert(TR)) {
      // navigate to the end position
      Sound.beep();
      hitIt(true);
      nav.travelTo(URx, URy);
      System.exit(0);
      return true;
    } else {
      // navigate to the next position
      Sound.twoBeeps();
      hitIt();
      return false;
    }
  }

  /**
   * In our color detection section we used a set of indexes which is different than the requirement
   * Thus this is to convert the requirement index into our index (RGBY = 0123) for color
   * identification
   * 
   * @param tr requirement indexes (RGBY = 4213)
   * @return int The color index we defined (RGBY = 0123)
   */
  private int colorconvert(int tr) {
    switch (tr) {
      case 1:
        return 2;
      case 2:
        return 1;
      case 3:
        return 3;
      case 4:
        return 0;
    }
    return -1;
  }

  /**
   * This method polls the color sensor reading, comparing it with the color RGB theoretical value
   * Every time it completed, the sensor rotate 30 degree ccw to take the next reading The 7
   * readings are stored in an array and then the majority color index is found in the array. This
   * index is returned as the read color
   * 
   * @return int index of the classified
   */
  private int rotateSensorDetect() {
    // create an array of size 7 to store 7 readings
    int[] colorResult = new int[7];
    // Test 7 times
    for (int i = 0; i < 7; i++) {
      // Poll, process RGB, and classify
      colorResult[i] = calibrator.Calibrate();
      // Move Color sensor motor
      Lab5.SENSOR_MOTOR.setAcceleration(300);
      Lab5.SENSOR_MOTOR.setSpeed(300);
      Lab5.SENSOR_MOTOR.rotate(30, false);
    }
    // Color sensor motor return to Original position
    SENSOR_MOTOR.rotateTo(0, false);
    // Finding the majority element of the array (-1 is excluded since it is not a legitimate color)
    Arrays.sort(colorResult);
    int prev = colorResult[0];
    int count = 1;
    for (int i = 1; i < colorResult.length; i++) {
      if (colorResult[i] == prev && colorResult[i] != -1) {
        count++;
        if (count > colorResult.length / 2 - 1) {
          return colorResult[i]; // Find the majority value (being detected after enough times and
                                 // return that index)
        }
      } else {
        count = 1;
        prev = colorResult[i];
      }
    }
    return -1; // No majority or no legit values to return
  }

  /**
   * This method slowly pushes away the can so that the point is cleared up This method will be
   * called when the can detected is not the target
   */
  private void hitIt() {
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);

    // SENSOR_MOTOR.setSpeed(SENSOR_MOTOR.getMaxSpeed());
    // SENSOR_MOTOR.setAcceleration(5000);

    SENSOR_MOTOR.rotate(220, false);
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    SENSOR_MOTOR.rotateTo(0, false);
  }

  /**
   * This method is overloaded. A boolean flag will be taken in as signature, indicating that the
   * target is found Thus there is no need to continue on the searching routine The motor arm will
   * smash the can out of the way (since the rest of the map will not be traversed)
   */
  private void hitIt(boolean flag) {
    SENSOR_MOTOR.setSpeed(SENSOR_MOTOR.getMaxSpeed());
    SENSOR_MOTOR.setAcceleration(5000);

    SENSOR_MOTOR.rotate(220, false);
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    SENSOR_MOTOR.rotateTo(0, false);
  }

  /**
   * This is a static method allows the conversion of a distance to the total rotation of each wheel
   * need to cover that distance.
   * 
   * (Distance / Wheel Circumference) = Number of wheel rotations. Number of rotations * 360.0
   * degrees = Total number of degrees needed to turn.
   * 
   * @param radius - Radius of the wheel
   * @param distance - Distance of path
   * @return an integer indicating the total rotation angle for wheel to cover the distance
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This is a static method that converts the angle needed to turn at a corner to the equivalent
   * total rotation. This method first converts the degrees of rotation, radius of wheels, and width
   * of robot to distance needed to cover by the wheel, then the method calls another static method
   * in process to convert distance to the number of degrees of rotation.
   * 
   * @param radius - the radius of the wheels
   * @param width - the track of the robot
   * @param angle - the angle for the turn
   * @return an int indicating the total rotation sufficient for wheel to cover turn angle
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }


  /**
   * This method let the robot to approach the origin 1. travel in +y direction until meets a line,
   * when encounters a grid line, back off a bit 2. travel in +x direction until meets a line, when
   * encounters a grid line, back off a bit 3. This two moving instruction ensures that even if the
   * heading of the robot is problematic the method will still take the bot to a reasonable location
   * so the bot will be able to scan all 4 lines when doing light localization
   */
  public void approachOrigin(int index) {
    
    if(index == 1) {
      nav.turnTo(-45);
    }
    else {
      nav.turnTo(45);
    }
    
    csfilterSum = fetchFilteredSample();
    while (true) {
      csfilterSum = fetchFilteredSample();
      if (csfilterSum < 0.38) {
        LEFT_MOTOR.stop(true);
        RIGHT_MOTOR.stop();
        Sound.beep();
        break;
      } else {
        LEFT_MOTOR.setSpeed(150);
        RIGHT_MOTOR.setSpeed(150);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();

      }
    }
    
    if(index == 1) {
      nav.turnTo(0);
    }
    LEFT_MOTOR.rotate(-convertDistance(Lab5.WHEEL_RAD, 16 - 3), true);
    RIGHT_MOTOR.rotate(-convertDistance(Lab5.WHEEL_RAD, 16 - 3), false);

    // Move backwards so our origin is close to origin
    // Always back more (-6) than it forwarded so the car center is ways at negative X and Y
    // Consistent with later calculation
  }


  /**
   * This method is to do light localization every 2 lines at the last X point X, y coordinates are
   * taken in as the variable to update the odometer
   * 
   * @param x
   * @param y
   */
  private void lineLocalRight(int x, int y, int left) {
    approachOrigin(left);

    // Scan all four lines and record our angle
    int index = 0;
    double lineData[] = new double[4];

    while (index < 4) {
      LEFT_MOTOR.backward();
      RIGHT_MOTOR.forward();
      csfilterSum = fetchFilteredSample();
      if (csfilterSum < 0.38) {
        lineData[index] = odometer.getXYT()[2]; // Get theta value and store it inside the array by
                                                // rising index
        index++;
        Sound.beep();
      }
    }

    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop();

    double d_X, d_Y, Theta_X, Theta_Y;

    // calculate our location from 0 using the calculated angles
    Theta_Y = lineData[2] - lineData[0];
    Theta_X = lineData[3] - lineData[1];

    // calculate the difference of angle on the odometer and it in reality
    double d_theta = 90 - (lineData[2] - 180) + Theta_Y / 2.0;

    d_X = -Math.abs(13 * Math.cos(Math.toRadians(Theta_Y / 2.0)));
    d_Y = -Math.abs(13 * Math.cos(Math.toRadians(Theta_X / 2.0)));


    // travel to origin to correct position
    odometer.setXYT(d_X, d_Y, odometer.getXYT()[2]);
    nav.travelTo(0.0, 0.0);

    // if we are not facing 0.0 then turn ourselves so that we are
    if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
      Sound.beep();
      LEFT_MOTOR.setSpeed(100);
      RIGHT_MOTOR.setSpeed(100);

      nav.turnTo(odometer.getXYT()[2] + d_theta);
    }
    try {
      Thread.sleep(1000);
    }
    catch(Exception e){
        //do nothing
    }
    
    // After localizing, set the odometer to the current gridline
    if(left == 1) {
      odometer.setX((x + 1)*Lab5.TILE);
    }
    else {
      odometer.setX((x - 1) * Lab5.TILE);  
    }
    odometer.setY(y * Lab5.TILE);
    nav.turnTo(-6.8);
    odometer.setTheta(0);

  }

  
  
  /**
   * This is a mean filter. The filter takes 5 consecutive readings from the light sensor (the one
   * at the back of the cart), amplifies them to increase sensor sensitivity, calculate their mean
   * value and return the mean as the reading The value is the reflect intensity value when the
   * sensor is in RED mode
   * 
   * @return the mean of the five readings of intensities
   */
  private double fetchFilteredSample() {
    csfilterSum = 0;
    // take 5 readings
    for (int i = 0; i < 5; i++) {
      // acquire sample data and read into array with no offset
      csColor.fetchSample(csData, 0);
      // amplify signal for increased sensitivity
      csfilterSum += csData[0];
    }
    // return an amplified average
    return csfilterSum / 5.0;

  }
}
