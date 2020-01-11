package ca.mcgill.ecse211.lab5;

import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.lab5.Lab5.BLUE_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.YELLOW_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.GREEN_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.RED_COLOR;

public class CanCalibrator {

  private SampleProvider lightColor;
  private float[] lightData;
  private double std = 0;
  public static double[] mean = new double[3];
  public static double[] standard_deviation = new double[3];

  // RGB indexes
  private static final int RED_INDEX = 0;
  private static final int GREEN_INDEX = 1;
  private static final int BLUE_INDEX = 2;
  private static final int YELLOW_INDEX = 3;


  /**
   * 
   * This is the constructor
   * 
   * @param lightColor
   * @param lightData
   */
  public CanCalibrator(SampleProvider lightColor, float[] lightData) {
    this.lightColor = lightColor;
    this.lightData = lightData;
  }

  /**
   * This is the classifier.
   * @param target
   * @return 
   */
  public int Calibrate() {

    // initialize array
    double[] red_array = new double[100];
    double[] green_array = new double[100];
    double[] blue_array = new double[100];

    // take initial reading
    // record the computed mean for 100 times for rgb
    for (int i = 0; i < 100; i++) {
      // each array contains 100 values
      red_array[i] = initialReading(RED_INDEX);
      green_array[i] = initialReading(GREEN_INDEX);
      blue_array[i] = initialReading(BLUE_INDEX);
    }

    // compute to find mean of the 100 means
    // mean has only 3 values; r,g,b
    mean[RED_INDEX] = Find_Mean(red_array);
    mean[GREEN_INDEX] = Find_Mean(green_array);
    mean[BLUE_INDEX] = Find_Mean(blue_array);

    // find the standard deviation of each rgb array which contains 100 means
    standard_deviation[RED_INDEX] = Find_Standard_Deviation(red_array, mean[RED_INDEX]);
    standard_deviation[GREEN_INDEX] = Find_Standard_Deviation(green_array, mean[GREEN_INDEX]);
    standard_deviation[BLUE_INDEX] = Find_Standard_Deviation(blue_array, mean[BLUE_INDEX]);

    // normalize the mean to a range of 0 - 1
    mean = Mean_Normalizer(mean[RED_INDEX], mean[GREEN_INDEX], mean[BLUE_INDEX]);

    // determine which color
    return isColor();
  }

  /**
   * 
   * This is the overloaded method. Take readings and determine whether can is the correct one.
   * 
   * @param target
   * @return
   */
  public boolean Calibrate(double[] target) {

    // initialize array
    double[] red_array = new double[100];
    double[] green_array = new double[100];
    double[] blue_array = new double[100];

    // take initial reading
    // record the computed mean for 100 times for rgb
    for (int i = 0; i < 100; i++) {
      // each array contains 100 values
      red_array[i] = initialReading(RED_INDEX);
      green_array[i] = initialReading(GREEN_INDEX);
      blue_array[i] = initialReading(BLUE_INDEX);
    }

    // compute to find mean of the 100 means
    // mean has only 3 values; r,g,b
    mean[RED_INDEX] = Find_Mean(red_array);
    mean[GREEN_INDEX] = Find_Mean(green_array);
    mean[BLUE_INDEX] = Find_Mean(blue_array);

    // find the standard deviation of each rgb array which contains 100 means
    standard_deviation[RED_INDEX] = Find_Standard_Deviation(red_array, mean[RED_INDEX]);
    standard_deviation[GREEN_INDEX] = Find_Standard_Deviation(green_array, mean[GREEN_INDEX]);
    standard_deviation[BLUE_INDEX] = Find_Standard_Deviation(blue_array, mean[BLUE_INDEX]);

    // normalize the mean to a range of 0 - 1
    mean = Mean_Normalizer(mean[RED_INDEX], mean[GREEN_INDEX], mean[BLUE_INDEX]);

    if (Compare_Standard_Deviation(target[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
        && Compare_Standard_Deviation(target[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(target[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])) {
      return false;
    }
    return false;
  }

  /**
   * 
   * Compute the mean
   * 
   * @param color_array
   * @return
   */
  private double Find_Mean(double[] color_array) {
    double mean = 0;
    for (int i = 0; i < color_array.length; i++) {
      mean = mean + color_array[i];
    }
    mean = mean / color_array.length;
    return mean;
  }

  /**
   * 
   * Compute the standard deviation
   * 
   * @param color_array
   * @param mean
   * @return
   */
  private double Find_Standard_Deviation(double[] color_array, double mean) {
    double stdr_dev = 0;
    double[] temp_array = new double[color_array.length];
    for (int i = 0; i < temp_array.length; i++) {
      temp_array[i] = (color_array[i] - mean) * (color_array[i] - mean);
    }
    for (int i = 0; i < temp_array.length; i++) {
      stdr_dev += temp_array[i];
    }
    stdr_dev = Math.sqrt(stdr_dev / (temp_array.length - 1));
    return stdr_dev;
  }

  /**
   * 
   * A method to normalize the values to a range from 0 to 1
   * 
   * @param red
   * @param green
   * @param blue
   * @return
   */
  private double[] Mean_Normalizer(double red, double green, double blue) {
    double[] rgb = new double[3];
    double euclidean = (Math.sqrt(red * red + green * green + blue * blue));
    rgb[0] = red / euclidean;
    rgb[1] = green / euclidean;
    rgb[2] = blue / euclidean;
    return rgb;
  }

  /**
   * 
   * Find out whether the target value is within 2 standard deviations of the measured readings
   * 
   * @param target
   * @param strd_dev
   * @param mean
   * @return
   */
  private boolean Compare_Standard_Deviation(double target, double strd_dev, double mean) {
    if (Math.abs(mean - target) < Math.abs(mean - 2 * strd_dev)) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Determines whether measured color is target color
   * @param color
   * @return int indicating color
   */
  private int isColor() {
    if (Compare_Standard_Deviation(YELLOW_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
    	&& Compare_Standard_Deviation(YELLOW_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(YELLOW_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])) {
       return YELLOW_INDEX;
    } else if(Compare_Standard_Deviation(GREEN_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
        && Compare_Standard_Deviation(GREEN_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(GREEN_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])){
      return GREEN_INDEX;
    } else if(Compare_Standard_Deviation(BLUE_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
        mean[RED_INDEX])
        && Compare_Standard_Deviation(BLUE_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
            mean[GREEN_INDEX])
        && Compare_Standard_Deviation(BLUE_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
            mean[BLUE_INDEX])) {
      return BLUE_INDEX;
    } else if(Compare_Standard_Deviation(RED_COLOR[RED_INDEX], standard_deviation[RED_INDEX],
            mean[RED_INDEX])
            && Compare_Standard_Deviation(RED_COLOR[GREEN_INDEX], standard_deviation[GREEN_INDEX],
                mean[GREEN_INDEX])
            && Compare_Standard_Deviation(RED_COLOR[BLUE_INDEX], standard_deviation[BLUE_INDEX],
                mean[BLUE_INDEX])) {
          return RED_INDEX;
    } else {
      // other color
      return -1;
    }
  }

  /**
   * 
   * Take initial readings
   * 
   * @param index
   * @return
   */
  private double initialReading(int index) {
    for (int i = 0; i < 10; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, 0);
      // place reading into corresponding place
      std += lightData[index];
    }
    // take average of the standard
    return std /= 100.0;
  }
}
