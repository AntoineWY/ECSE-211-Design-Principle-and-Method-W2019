package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;

import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.WHEEL_RAD;
import static ca.mcgill.ecse211.project.project.TRACK;
import static ca.mcgill.ecse211.project.project.TILE;

/**
 * The mapPlanner class plans out the route of the robot based on the parameter received from the wifi class.
 * In total there are 24 cases included in the mapPlanner class and for graphical illustrations regarding to each case and the
 * exact route please refer to the software documents.
 * <p> 
 *  This class serves as a collection of methods which partition the entire map traversal into the following segments. <br>
 *  1.From the staring corner, through the tunnel and to the search island, handled by toIsland().<br>
 *  2. From the point near the tunnel, select an appropriate point in the search area and travel to it. Handled by 
 *  toSearchZone() method.<br>
 *  3.From the search point back to the selected point near the tunnel in the home zone, meaning after the acquisition of the can
 *  the vehicle is carrying the can back to the starting point, handled by backToTeamZone().<br>
 *  4.From the point near tunnel to starting corner, handled by backToStartingBlock().<br>
 *  5. After the can has been placed inside of the starting corner, the vehicle needs to re-localize onto correct starting point with 
 *  correct orientation, handled by toNextSearch();<br>
 * @author Antoine Wang
 *
 */
public class MapPlanner {
//------------------------------------------------------------Constants---------------------------------------------------------------------
	/**
	 * This is a modified radian value to engineer a 90 degree turn. The aim is to counter the issue when the vehicle
	 * deviates from the path (doing inaccurate 90 degree turns) after acquiring the can.
	 */
	private static final double DEGREE90_WITHCAN = Math.PI / 2.0;

//------------------------------------------------------------Fields-----------------------------------------------------------------------
	// passing the parameters received and processed in the project class to the mapPlanner class
	public static int corner = project.corner;
	public static int team_LL_x = project.team_LL_x;
	public static int team_LL_y = project.team_LL_y;
	public static int team_UR_x = project.team_UR_x;
	public static int team_UR_y = project.team_UR_y;

	public static int tunnel_LL_x = project.tunnel_LL_x;
	public static int tunnel_LL_y = project.tunnel_LL_y;
	public static int tunnel_UR_x = project.tunnel_UR_x;
	public static int tunnel_UR_y = project.tunnel_UR_y;
	
	public static int zone_LL_x = project.zone_LL_x;
	public static int zone_LL_y = project.zone_LL_y;
	public static int zone_UR_x = project.zone_UR_x;
	public static int zone_UR_y = project.zone_UR_y;

	public static int island_LL_x = project.Island_LL_x;
	public static int island_LL_y = project.Island_LL_y;
	public static int island_UR_x = project.Island_UR_x;
	public static int island_UR_y = project.Island_UR_y;

	
//------------------------------------------------------------public methods-------------------------------------------------------------------
	/**
	 * toIsland() method is a collection of cases. After parameters are received, the corresponding case will
	 * drive the robot to the destination point. Boundary cases are considered, like if the tunnel is by the wall
	 * or the river, the localizing point will be varied thus the entire traversing plan will also have minor changes. 
	 * The detailed graphic description of 24 routes are in the software documents.
	 * <p>
	 * This method will navigates the robot to the selected point at the other side of the tunnel. On the
	 * way constant corrections and re-localizations is performed. First the
	 * robot will arrive near the tunnel, perform localization, driving forward
	 * for half a tile and proceed through the tunnel. Then at the other side of
	 * the tunnel the robot relocalize, then get ready to proceed to the search area.
	 * <p>
	 * A final thing worth noticing is that when the vehicle is localizing near the tunnel before traversing,
	 * the point is pushed in the keyPoint stack. Then after passing the tunnel the localization point on the other side 
	 * is also pushed. Thus on the way back, the points are in popped sequence. This this done through locaNearTunnel(x,y)
	 * from NavigationWithCorr class.
	 * 
	 * @return caseFlag
	 *            corresponds to which route the robot will take, later used in method backToTeamZone() which navigates the 
	 *            robot back using the same route.
	 * @param isTV
	 *            boolean indicates if the tunnel is vertical
	 * @param navWc
	 *            instance of NavigationWithCorr class
	 * @param dll
	 *            instance of DoubleLightLocalization class
	 * @param odo
	 *            instance of odometer class
	 * @see {@link NavigationWithCorr#locaNearTunnel(double, double)}
	 */
	public static int toIsland(boolean isTV, NavigationWithCorr navWc, DoubleLightLocalization dll, Odometer odo) {

		// Those parameters are conditions that the robot should consider to avoid crashing into the wall or falling into the river
		int lowerBound = Math.max(team_LL_y, island_LL_y);
		int upperBound = Math.min(team_UR_y, island_UR_y);
		int leftBound = Math.max(team_LL_x, island_LL_x);
		int rightBound = Math.min(team_UR_x, island_UR_x);
		
		//for debug purpose
		System.out.println("upper: " + upperBound);
		System.out.println("lower: " + lowerBound);
		System.out.println("left: " + leftBound);
		System.out.println("right: " + rightBound);

		// In total 4 big cases for corners
		// From each corner there are 2 tunnel placements
		// Each placement cases have 3 cases to consider
		// in total is 4*2*3 =24 cases and details please refer to the software document
		switch (corner) {
		case 0:
			if (isTV) {
				if (tunnel_LL_x > leftBound && tunnel_UR_x < rightBound) {
					// the tunnel is vertical
					navWc.travelTo(tunnel_LL_x, tunnel_LL_y - 1);
					turnToZero(odo); // make sure near the tunnel the vehicle is
										// heading zero

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);

					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 0 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();
					
					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					navWc.locaNearTunnel(tunnel_UR_x, tunnel_UR_y + 1);

					

					return 1;

				} else if (tunnel_LL_x == leftBound) {
					// the tunnel is vertical
					navWc.travelTo(tunnel_LL_x + 1, tunnel_LL_y - 1);
					turnToZero(odo); // make sure near the tunnel the vehicle is
										// heading zero

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_LL_x + 1, tunnel_LL_y - 1);

					// turn left, forward for half a tile
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 0 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					
					navWc.locaNearTunnel(tunnel_UR_x, tunnel_UR_y + 1);

					
					return 2;

				} else { // tunnel_UR_x == rightBound
							// the tunnel is vertical
					navWc.travelTo(tunnel_LL_x, tunnel_LL_y - 1);
					turnToZero(odo); // make sure near the tunnel the vehicle is
										// heading zero

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);

					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 0 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();
					
					// turn left, forward for half a tile
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);


					navWc.locaNearTunnel(tunnel_UR_x - 1, tunnel_UR_y + 1);

					
					return 3;
				}

			} else {
				if (tunnel_LL_y > lowerBound && tunnel_UR_y < upperBound) {
					// the tunnel is horizontal
					navWc.travelTo(tunnel_LL_x - 1, tunnel_LL_y);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn right, go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, go to key point and localize
					turnToZero(odo);
					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y);

					
					
					return 4;
				} else if (tunnel_LL_y == lowerBound) {
					// the tunnel is horizontal
					navWc.travelTo(tunnel_LL_x - 1, tunnel_LL_y + 1);
					turnToZero(odo);

					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y + 1);

					// backward for half a tile
					RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn right, go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y);

					
					

					return 5;
				} else { // tunnel_UR_y == upperBound
					// the tunnel is horizontal
					navWc.travelTo(tunnel_LL_x - 1, tunnel_LL_y);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn right, go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y - 1);

					
					
					return 6;
				}
			}
		case 1:
			if (isTV) {
				if (tunnel_LL_x > leftBound && tunnel_UR_x < rightBound) {
					// the tunnel is vertical
					navWc.travelTo(tunnel_LL_x, tunnel_LL_y - 1);
					turnToZero(odo); // make sure near the tunnel the vehicle is
										// heading zero

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);
					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 0 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					turnToZero(odo);

					navWc.locaNearTunnel(tunnel_UR_x, tunnel_UR_y + 1);
					
					return 7;
				} else if (tunnel_LL_x == leftBound) { // tunnel adjacent to the
														// river
					// the tunnel is vertical
					navWc.travelTo(tunnel_LL_x + 1, tunnel_LL_y - 1);
					turnToZero(odo); // make sure near the tunnel the vehicle is
										// heading zero
					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_LL_x + 1, tunnel_LL_y - 1);

					// turn left, forward for half a tile
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 0 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					navWc.locaNearTunnel(tunnel_UR_x, tunnel_UR_y + 1);

					
					return 8;
				} else { // tunnel_UR_x = rightbound, ur adjacent to the wall
					// the tunnel is vertical
					navWc.travelTo(tunnel_LL_x, tunnel_LL_y - 1);
					turnToZero(odo); // make sure near the tunnel the vehicle is
										// heading zero

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);

					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 0 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// Move left for half a tile to avoid the wall
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					navWc.locaNearTunnel(tunnel_UR_x - 1, tunnel_UR_y + 1);
					
					return 9;
				}

			} else {
				if (tunnel_LL_y > lowerBound && tunnel_UR_y < upperBound) {
					// the tunnel is horizontal
					navWc.travelTo(tunnel_UR_x + 1, tunnel_UR_y - 1);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y - 1);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn left, go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y + 1);

					
					
					return 10;
				} else if (tunnel_LL_y == lowerBound) {
					// the tunnel is horizontal
					navWc.travelTo(tunnel_UR_x + 1, tunnel_UR_y);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y);

					// backward for half a tile
					RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn left, go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y + 1);

					
					
					return 11;
				} else {// tunnel_UR_y == upperBound
						// the tunnel is horizontal
					navWc.travelTo(tunnel_UR_x + 1, tunnel_UR_y - 1);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y - 1);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn left, go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					// backward for half a tile
					RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y);

					
					
					return 12;
				}
			}

		case 2:
			if (isTV) {
				if (tunnel_LL_x > leftBound && tunnel_UR_x < rightBound) {
					// the tunnel is vertical
					navWc.travelTo(tunnel_UR_x, tunnel_UR_y + 1);
					navWc.turnTo(180);

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_UR_x, tunnel_UR_y + 1);
					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 180 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);
					
					return 13;
				} else if (tunnel_LL_x == leftBound) {
					// the tunnel is vertical
					navWc.travelTo(tunnel_UR_x, tunnel_UR_y + 1);
					navWc.turnTo(180);

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_UR_x, tunnel_UR_y + 1);
					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 180 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// turn left to avoid boundary
					navWc.turnTo(90);

					navWc.locaNearTunnel(tunnel_LL_x + 1, tunnel_LL_y - 1);
					
					return 14;
				} else { // tunnel_UR_x == rightBound
					// the tunnel is vertical
					navWc.travelTo(tunnel_UR_x - 1, tunnel_UR_y + 1);
					navWc.turnTo(180);

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_UR_x - 1, tunnel_UR_y + 1);
					// turn left, forward for half a tile
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 180 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// turn right to avoid boundary
					DoubleLightLocalization.reorientRobot(Math.PI / 2);

					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);
					
					return 15;
				}

			} else {
				if (tunnel_LL_y > lowerBound && tunnel_UR_y < upperBound) {
					// the tunnel is horizontal
					navWc.travelTo(tunnel_UR_x + 1, tunnel_UR_y - 1);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y - 1);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn left, go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y + 1);

					
					
					return 16;
				} else if (tunnel_LL_y == lowerBound) {
					// the tunnel is horizontal
					navWc.travelTo(tunnel_UR_x + 1, tunnel_UR_y);
					navWc.turnTo(180);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn right, go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y + 1);

					
					
					return 17;

				} else { // tunnel_UR_y == upperBound
					// the tunnel is horizontal
					navWc.travelTo(tunnel_UR_x + 1, tunnel_UR_y - 1);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y - 1);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn left, go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					// backward for half a tile to avoid boundary
					RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y);

					
					
					return 18;
				}
			}

		case 3:
			if (isTV) {

				if (tunnel_LL_x > leftBound && tunnel_UR_x < rightBound) {
					// the tunnel is vertical
					navWc.travelTo(tunnel_UR_x - 1, tunnel_UR_y + 1);
					navWc.turnTo(180);

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_UR_x - 1, tunnel_UR_y + 1);
					// turn left, forward for half a tile
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 180 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);
					
					return 19;
				} else if (tunnel_LL_x == leftBound) {
					// the tunnel is vertical
					navWc.travelTo(tunnel_UR_x, tunnel_UR_y + 1);
					navWc.turnTo(180);

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_UR_x, tunnel_UR_y + 1);
					// turn right, forward for half a tile
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 180 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					navWc.throughTunnel();

					// turn right (from the view of spectators) to avoid wall
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);

					navWc.locaNearTunnel(tunnel_LL_x + 1, tunnel_LL_y - 1);
					
					return 20;
				} else {// tunnel_UR_x == rightBound
						// the tunnel is vertical
					navWc.travelTo(tunnel_UR_x - 1, tunnel_UR_y + 1);
					navWc.turnTo(180);

					// localize near the tunnel
					navWc.locaNearTunnel(tunnel_UR_x - 1, tunnel_UR_y + 1);
					// turn left, forward for half a tile
					DoubleLightLocalization.reorientRobot(-Math.PI / 2);
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn to 180 degree heading, then go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// turn left (from the view of spectators) to avoid wall
					DoubleLightLocalization.reorientRobot(Math.PI / 2);

					navWc.locaNearTunnel(tunnel_LL_x, tunnel_LL_y - 1);
					
					return 21;
				}

			} else {

				if (tunnel_LL_y > lowerBound && tunnel_UR_y < upperBound) {
					// the tunnel is horizontal
					navWc.travelTo(tunnel_LL_x - 1, tunnel_LL_y);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn right, go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y);

					
					
					return 22;
				} else if (tunnel_LL_y == lowerBound) {
					navWc.travelTo(tunnel_LL_x - 1, tunnel_LL_y + 1);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y + 1);

					RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn right, go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// after passing through the tunnel, turn to zero degree and
					// localize
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y);

					
					

					return 23;
				} else { // tunnel_UR_y = upperbound, tunnel is adjacent to the
							// upper wall of the table
					navWc.travelTo(tunnel_LL_x - 1, tunnel_LL_y);
					turnToZero(odo);
					navWc.locaNearTunnel(tunnel_LL_x - 1, tunnel_LL_y);

					// forward for half a tile
					RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
					LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

					// turn right, go through the tunnel
					DoubleLightLocalization.reorientRobot(Math.PI / 2);
					navWc.throughTunnel();

					// turn right now the robot heads at 180 degree and move one
					// tile down to avoid the boundary
					DoubleLightLocalization.reorientRobot(Math.PI / 2);

					// localize after the tunnel
					navWc.locaNearTunnel(tunnel_UR_x + 1, tunnel_UR_y - 1);
					
					return 24;
				}

			}
		}
		return -1;
	}
	
	/**
	 * This method takes the robot to the search zone based on the location of the tunnel.<br>
	 * If the exit of the tunnel is at the upper right direction of the search zone, the robot will be lead to the upper 
	 * right point of search area. Similar but to the lower left for other cases.
	 * Upon arriving localize the robot at the point to be ready to perform search routine.<br>
	 * Note that this method also handles the edge cases. If the search zone is adjacent to the river or wall, the robot
	 * dodges those points by modifying its search point.
 	 * @param odo instance of odometer class
	 * @param navWc instance of NavigationWithCorr class
	 * @return point[]<br>
	 *         An integer array holding the search point's coordinate 
	 */
	public static int[] toSearchZone(NavigationWithCorr navWc, Odometer odo){
		switch(corner){
		case 0: case 1: case 3:
			if (zone_LL_x == island_LL_x || zone_LL_y == island_LL_y){ // BOUNDARY CASE: travel to the inner point
				navWc.travelTo(zone_LL_x + 1, zone_LL_y + 1);
				navWc.turnTo(0);
				navWc.localizeOnTheWay(zone_LL_x + 1, zone_LL_y + 1);
				return new int[]{zone_LL_x + 1, zone_LL_y + 1};
				
			}else{ // normal case, the robot proceed to the ll point of the search zone
				navWc.travelTo(zone_LL_x, zone_LL_y );
				navWc.turnTo(0);
				navWc.localizeOnTheWay(zone_LL_x, zone_LL_y );
				return new int[]{zone_LL_x, zone_LL_y};
			}
		case 2:
			if (zone_UR_x == island_UR_x || zone_UR_y == island_UR_y){ // BOUNDARY CASE: travel to the inner point
				navWc.travelTo(zone_UR_x - 1, zone_UR_y - 1);
				navWc.turnTo(180);
				navWc.localizeOnTheWay(zone_UR_x - 1, zone_UR_y - 1);
				return new int[]{zone_UR_x - 1, zone_UR_y - 1};
				
			}else{ //normal case, the robot proceed to the ur point of the search zone
				navWc.travelTo(zone_UR_x , zone_UR_y);
				navWc.turnTo(180);
				navWc.localizeOnTheWay(zone_UR_x , zone_UR_y);
				return new int[]{zone_UR_x , zone_UR_y};
			}
		}
		return null;
		
	}

	/**
	 * This is the method which brings the vehicle back to the home zone. The starting point is after the robot localize on the search
	 * point. The end point is that the robot relocalize on the other side of the tunnel, in the team zone with the can.
	 * <p>
	 * The procedure of traversing the tunnel in a opposite manner is:<br>
	 * 1. The stack storing localizing points near the tunnel pops the two key points, thus the vehicle knows where to localize before and 
	 * after traversing the tunnel <br>
	 * 2. Based on the case returned on the way to the search zone, the robot choose the corresponding route to return. <br>
	 * 3. After the traversal of the tunnel, the robot will be navigate to the same point on which it localized on the way
	 * to the search zone. Then, the robot localize on that point and that is the end point of the method.
	 * 
	 * @param caseFlag the number representation of the route the robot takes
	 * @param navWc instance of the navigation class driving the robot
	 * @param odo instance of the odometer class keeping the robot's current position
	 */
	public static void backToTeamZone(int caseFlag, NavigationWithCorr navWc, Odometer odo) {

		//tunnel end point is the localization point selected near the tunnel on the side of the search island
		int[] tunnelEndPoint = project.keyPoints.pop();
		
		//tunnel start point is the localization point selected near the tunnel on the side of the team zone
		int[] tunnelStartPoint = project.keyPoints.pop();
		
		//debug purpose
		System.out.println("tunnelendPoint: " + tunnelEndPoint[0] + " " + tunnelEndPoint[1]);
		System.out.println("tunnelStartPoint: " + tunnelStartPoint[0] + " " + tunnelStartPoint[1]);
		System.out.println("case: " + caseFlag);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	
		//Go from the search point to the tunnel, finish localization and get ready for tunnel traversing
		navWc.travelTo(tunnelEndPoint[0], tunnelEndPoint[1]);
		turnToZero(odo);
		navWc.localizeOnTheWay(tunnelEndPoint[0], tunnelEndPoint[1]); 

		//Based on the caseFlag, choosing corresponding route to pass the tunnel
		//details please refer to software document
		switch (caseFlag) {
		case 1:
		case 2:
			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);

			navWc.throughTunnel();
			odo.setXYT((tunnel_LL_x + 0.5) * TILE, (tunnel_LL_y - 1) * TILE, 180);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 3:
			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);

			navWc.throughTunnel();
			odo.setXYT((tunnel_LL_x + 0.5) * TILE, (tunnel_LL_y - 1) * TILE, 180);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 4:
		case 5:
			// backward for half a tile
			RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);
			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);

			navWc.throughTunnel();
			odo.setXYT((tunnel_LL_x - 1) * TILE, (tunnel_LL_y + 0.5) * TILE, 270);

			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 6:
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			navWc.throughTunnel();
			odo.setXYT((tunnel_LL_x - 1) * TILE, (tunnel_LL_y + 0.5) * TILE, 270);

			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 7:
		case 8:

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);

			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);

			navWc.throughTunnel();

			odo.setXYT((tunnel_LL_x + 0.5) * TILE, (tunnel_LL_y - 1) * TILE, 180);

			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 9:
			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			navWc.throughTunnel();
			odo.setXYT((tunnel_LL_x + 0.5) * TILE, (tunnel_LL_y - 1) * TILE, 180);

			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 10:
		case 11:

			// backward for half a tile
			RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_UR_x + 1) * TILE, (tunnel_UR_y - 0.5) * TILE, 90);
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;
		case 12:

			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_UR_x + 1) * TILE, (tunnel_UR_y - 0.5) * TILE, 90);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 13:
		case 15:

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			navWc.throughTunnel();
			odo.setXYT((tunnel_UR_x - 0.5) * TILE, (tunnel_UR_y + 1) * TILE, 0);

			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 14:

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_UR_x - 0.5) * TILE, (tunnel_UR_y + 1) * TILE, 0);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 16:
		case 17:

			// backward for half a tile
			RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_UR_x + 1) * TILE, (tunnel_UR_y - 0.5) * TILE, 90);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 18:

			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_UR_x + 1) * TILE, (tunnel_UR_y - 0.5) * TILE, 90);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;

		case 19:
		case 21:

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_UR_x - 0.5) * TILE, (tunnel_UR_y + 1) * TILE, 0);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;
		case 20:

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// right turn
			DoubleLightLocalization.reorientRobot(DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_UR_x - 0.5) * TILE, (tunnel_UR_y + 1) * TILE, 0);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;
		case 22:
		case 23:

			// backward for half a tile
			RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_LL_x - 1) * TILE, (tunnel_LL_y + 0.5) * TILE, 270);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;
		case 24:

			// forward for half a tile
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), true);
			LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, 0.5 * TILE), false);

			// left turn
			DoubleLightLocalization.reorientRobot(-DEGREE90_WITHCAN);
			navWc.throughTunnel();

			odo.setXYT((tunnel_LL_x - 1) * TILE, (tunnel_LL_y + 0.5) * TILE, 270);
			navWc.travelTo(tunnelStartPoint[0], tunnelStartPoint[1]);
			navWc.localizeOnTheWay(tunnelStartPoint[0], tunnelStartPoint[1]);
			break;
		}

	}
	
	/**
	 * This method is to navigating the robot back to the inner position starting block.
	 * Based on the corner, correspoding points are input with minor changes (like travel to (0.7, 0.7) instead of (1,1))
	 * to make sure the robot always goes inside of that block.
	 * Thus the vehicle is guaranteed to drop the can inside of the starting zone
	 * @param navWc
	 *            instance of NavigationWithCorr class
	 */

	public static void backToStartingBlock(NavigationWithCorr navWc) {
		switch (corner) {
		case 0:
			navWc.travelTo(0.7, 0.7);
			break;
		case 1:
			navWc.travelTo(14.3, 0.7);
			break;
		case 2:
			navWc.travelTo(14.3, 8.3);
			break;
		case 3:
			navWc.travelTo(0.7, 8.3);
		}
	}

	/**
	 * This method aims to relocalize the vehicle at the starting corner to restore the vehicle's
	 * original position and prepare for the next search.
	 * <p>
	 * The robot will turn to corresponding starting orientation first and perform double light sensor localization
	 * @param navWc Instance of the NavigationWithCorr class
	 */
	public static void toNextSearch(NavigationWithCorr navWc) {
		switch (corner) {
		case 0:
			navWc.turnTo(0);
			navWc.localizeOnTheWay(1, 1);
			break;
		case 1:
			navWc.turnTo(270);
			navWc.localizeOnTheWay(14, 1);
			break;
		case 2:
			navWc.turnTo(180);
			navWc.localizeOnTheWay(14, 14);
			break;
		case 3:
			navWc.turnTo(90);
			navWc.localizeOnTheWay(1, 14);
			break;
		}
	}

	/**
	 * Re-orient the robot to zero degree heading by turning to clear the value
	 * of odometer theta to zero
	 * 
	 * @param odo Odometer passed in to regulating the turning of the robot
	 */
	public static void turnToZero(Odometer Odo) {
		double theta = Odo.getXYT()[2];
		if (theta > 0 && theta < 180) {
			LEFT_MOTOR.rotate(-NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, theta), true);
			RIGHT_MOTOR.rotate(NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, theta), false);
		} else if (theta >= 180) {
			LEFT_MOTOR.rotate(NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, 360 - theta), true);
			RIGHT_MOTOR.rotate(-NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, 360 - theta), false);
		}

	}
}
