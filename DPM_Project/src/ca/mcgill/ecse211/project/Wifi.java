package ca.mcgill.ecse211.project;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
 * This class will serve to retrieve the data provided by the WIfi server.
 * 
 * @author Erica De Petrillo
 *
 */
public class Wifi {

  private static final String SERVER_IP = "192.168.2.22"; //on the day of competition, must be "192.168.2.3"
  private static final int TEAM_NUMBER = 23; //our team number
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
  
  //parameters being retrieved from server
  public static int RedTeam; //red team number
  public static int RedCorner; //red team's starting corner
  public static int GreenTeam; //green team number
  public static int GreenCorner; //green team's starting corner
  public static int Red_LL_y; //lower left hand corner of red zone y coord
  public static int Red_LL_x; //lower left hand corner of red zone x coord
  public static int Red_UR_y; //upper right corner red zone y
  public static int Red_UR_x; //upper right corner red zone x
  public static int Green_LL_y; //lower left corner green zone y
  public static int Green_LL_x; //lower left corner green zone x
  public static int Green_UR_y; //upper right corner green zone y
  public static int Green_UR_x; //upper right corner green zone x
  public static int Island_LL_y; //lower left corner island y
  public static int Island_LL_x; //lower left corner island x
  public static int Island_UR_y; //upper right corner island y
  public static int Island_UR_x; //upper right corner island x
  public static int TNR_LL_y; //lower left corner red tunnel footprint y
  public static int TNR_LL_x; //lower left corner red tunnel footprint x
  public static int TNR_UR_y; //upper right corner red tunnel footprint y
  public static int TNR_UR_x; //upper right corner red tunnel footprint x
  public static int TNG_LL_y; //lower left corner green tunnel footprint y
  public static int TNG_LL_x; //lower left corner green tunnel footprint x
  public static int TNG_UR_y; //upper right corner green tunnel footprint y
  public static int TNG_UR_x; //upper right corner green tunnel footpring x
  public static int SZR_LL_y; //lower left corner red search zone y
  public static int SZR_LL_x; //lower left corner red search zone x
  public static int SZR_UR_y; //upper right corner red search zone y
  public static int SZR_UR_x; //upper right corner red search zone x
  public static int SZG_LL_y; //lower left corner green search zone y
  public static int SZG_LL_x; //lower left corner green search zone x
  public static int SZG_UR_y; //upper right corner green search zone y
  public static int SZG_UR_x; //upper right corner green search zone x
  
  @SuppressWarnings("rawtypes")
  public static void main(String[] args) {
    
    System.out.println("Running...");
    
    //initialize wifi connection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
    
 // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData(); 
     
      RedTeam = ((Long) data.get("RedTeam")).intValue();
      RedCorner = ((Long) data.get("RedCorner")).intValue();      
      GreenTeam = ((Long) data.get("GreenTeam")).intValue();
      GreenCorner = ((Long) data.get("GreenCorner")).intValue();
      Red_LL_y = ((Long) data.get("Red_LL_y")).intValue();
      Red_LL_x = ((Long) data.get("Red_LL_x")).intValue();
      Red_UR_y = ((Long) data.get("Red_UR_y")).intValue();
      Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
      Green_LL_y = ((Long) data.get("Green_LL_y")).intValue();
      Green_LL_x = ((Long) data.get("Green_LL_x")).intValue();
      Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();
      Green_UR_x = ((Long) data.get("Green_UR_x")).intValue();
      Island_LL_y = ((Long) data.get("Island_LL_y")).intValue();
      Island_LL_x = ((Long) data.get("Island_LL_x")).intValue();
      Island_UR_y = ((Long) data.get("Island_UR_y")).intValue();
      Island_UR_x = ((Long) data.get("Island_UR_x")).intValue();
      TNR_LL_y = ((Long) data.get("TNR_LL_y")).intValue();
      TNR_LL_x = ((Long) data.get("TNR_LL_x")).intValue();
      TNR_UR_y = ((Long) data.get("TNR_UR_y")).intValue();
      TNR_UR_x = ((Long) data.get("TNR_UR_x")).intValue();
      TNG_LL_y = ((Long) data.get("TNG_LL_y")).intValue();
      TNG_LL_x = ((Long) data.get("TNG_LL_x")).intValue();
      TNG_UR_y = ((Long) data.get("TNG_UR_y")).intValue();
      TNG_UR_x = ((Long) data.get("TNG_UR_x")).intValue();
      SZR_LL_y = ((Long) data.get("SZR_LL_y")).intValue();
      SZR_LL_x = ((Long) data.get("SZR_LL_x")).intValue();
      SZR_UR_y = ((Long) data.get("SZR_UR_y")).intValue();
      SZR_UR_x = ((Long) data.get("SZR_UR_x")).intValue();
      SZG_LL_y = ((Long) data.get("SZG_LL_y")).intValue();
      SZG_LL_x = ((Long) data.get("SZG_LL_x")).intValue();
      SZG_UR_y = ((Long) data.get("SZG_UR_y")).intValue();
      SZG_UR_x = ((Long) data.get("SZG_UR_x")).intValue();

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

  }
}
