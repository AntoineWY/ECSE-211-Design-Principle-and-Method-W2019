package ca.mcgill.ecse211.lab5;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

import static ca.mcgill.ecse211.lab5.Lab5.SENSOR_MOTOR;

import java.util.Arrays;

import ca.mcgill.ecse211.odometer.*;

/**This class implements the rotation sensor to detect the color of an unknown can.
 * After the can is placed in front of the can for a certain time (over 2 second),
 * the isCan() method will lead the class through to the detection of color phase.
 * 
 * The sensor motor will bring the color sensor around the can for a half revolution.
 * During the process, 7 readings are taken and the index with the highest frequency 
 * will be considered as the read value
 * 
 * After one reading is completed, the information stays on the screen for 6 seconds and 
 * then the program is ready to read another color.
 * 
 * @author Yinuo Antoine Wang
 * @author Yunhao Hu
 *
 */

public class ColorDetection extends Thread {
	 // threshold distance that tells the cart the can is presented
	 private static final double CAN_EXISTS = 10;
	 
	 // Fields to poll the US sensor
	 private float[] usData;
	 private SampleProvider usDistance;
	 
	// Fields to poll the Light sensor
	 private float[] lightData;
	 private SampleProvider lightColor;
	 
	 // 1 for red, 2 for green, 3 for blue, 4 for yellow
	 public int colorIndex;
	 
	 // Field for calibrator. Used to call Calibrate() to identify the color
	 CanCalibrator calibrator;
	 
	// the display LCD screen object
	 private TextLCD lcd;
	
	 /**
	  * Constructor of the class. Sample provider of the color and US sensor and the corrsponding
	  * array to store the value. Also, the instance of LCD textfield created in the mean Lab5 class is 
	  * passed in for the update of color information.
	  * @param usDistance Sample reading buffer for US sensor
	  * @param usData array where distance value is stored
	  * @param lightColor Sample reading buffer for light sensor
	  * @param lightData array where RGB value is stored
	  * @param lcd Screen display instance
	  */
	public ColorDetection (SampleProvider usDistance, float[] usData, SampleProvider lightColor, float[] lightData, TextLCD lcd){
		this.usDistance = usDistance;
		this.usData = usData;
		this.lightColor = lightColor;
        this.lightData = lightData;
        this.lcd = lcd;
        calibrator = new CanCalibrator(lightColor, lightData);
	}
	public void run() {
		
		while(true){
			
			
			if (isCan()){
				lcd.drawString("Object Detected!!!", 0, 0);
				Sound.beep();
				
				colorIndex = rotateSensorDetect();
				if (colorIndex == 0){
					lcd.drawString("Red Can", 0, 1);
				}
				else if (colorIndex == 1){
					lcd.drawString("Green Can", 0, 1);
				}
				else if (colorIndex == 2){
					lcd.drawString("Blue Can", 0, 1);
				}
				else if (colorIndex == 3){
					lcd.drawString("Yellow Can", 0, 1);
				}
				try {
					//Wait 6 second for check reading
					Thread.sleep(6000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				lcd.clear();
				
			}
			
		} 
	}
	
	/**
	 * This method polls the color sensor reading, comparing it with the color RGB theoretical value
	 * Every time it completed, the sensor rotate 30 degree ccw to take the next reading
	 * The 7 readings are stored in an array and then the majority color index is found in the array.
	 * This index is returned as the read color
	 * @return int index of the classified
	 */
	private int rotateSensorDetect(){
		// Create a array of length 7
		int[] colorResult = new int[7];
		
		//Test 7 times
		for (int i = 0; i < 7; i++){
			// Poll, process RGB, and classify
			colorResult[i] = calibrator.Calibrate();
			//Move Color sensor motor
			Lab5.SENSOR_MOTOR.setAcceleration(300);
			Lab5.SENSOR_MOTOR.setSpeed(300);
			Lab5.SENSOR_MOTOR.rotate(30, false);
		}
		// Color sensor motor return to Original position
		SENSOR_MOTOR.rotateTo(0, false);
		
		Arrays.sort(colorResult);
		
		//Below is the process of finding the majority of the legit elements (-1 values are excluded) 
		int prev = colorResult[0];
		int count = 1;
		for(int i = 1; i<colorResult.length; i++){
			if (colorResult[i] == prev && colorResult[i] != -1){
				count ++;
				if(count > colorResult.length / 2 -1 ) {
					return colorResult[i];  // Find the majority value (being detected after enough times and return that index)
					}
			}else{
				count = 1;
				prev = colorResult[i];
			}
		}
		return -1; // No majority or no legit values to return
	}
	/**
	 * This method polls the us sensor reading then uses a median filter to process it. Then, 
	 * if the median is less than the threshold, the thread sleep for 2 seconds.
	 * This is to avoid misreading. After 2s, if the object is still in front of the cart,
	 * the method return true, meaning that a can is present (DISPLAY: Object detected)
	 * @return boolean whether there is an object detected
	 */
	private boolean isCan() {
	    double distance = medianFilter();
	    if (distance < CAN_EXISTS) {
	    	try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    	
	    	 if (distance < CAN_EXISTS){
	    		 return true;
	    	 }
	    }
	    return false;
	  }

	/**
	 * This is a median filter. The filter takes 5 consecutive readings from the ultrasonic sensor,
	 * amplifies them to increase sensor sensitivity, sorts them, and picks the median to minimize the
	 * influence of false negatives and false positives in sensor readings, if any. The sensor is very
	 * likely to report false negatives.
	 * 
	 * @return the median of the five readings, sorted from small to large
	 */
	private double medianFilter() {
		double[] arr = new double[5]; // store readings
		for (int i = 0; i < 5; i++) { // take 5 readingss
			usDistance.fetchSample(usData, 0); // store reading in buffer
			arr[i] = usData[0] * 100.0; // signal amplification
		}
		Arrays.sort(arr); // sort readingss
		return arr[2]; // take median value
	}
	

}
