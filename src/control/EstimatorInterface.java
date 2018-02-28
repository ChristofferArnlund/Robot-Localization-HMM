package control;

/*
 * This interface provides the graphics class (RobotLocalizationViewer) with the 
 * necessary access points to the data it wants to display.
 * If you want to use the viewer, have some class implement this interface, which 
 * you could see as a wrapper to your internal model for the transition and sensor 
 * matrices as well as the probability distribution over the possible positions
 * 
 * The viewer assumes a MxN-grid and in each field (m,n) the robot can have four 
 * different headings. Your implementation of this interface needs hence to provide 
 * the requested data in relation to this assumption!
 */
public interface EstimatorInterface {

	/*
	 * return the number of assumed rows, columns and possible headings for the grid
	 * a number of four headings makes obviously most sense... the viewer can handle 
	 * four headings at maximum in a useful way.
	 */
	public int getNumRows();
	public int getNumCols();
	public int getNumHead();
	
	/*
	 * should trigger one step of the estimation, i.e., true position, sensor reading and 
	 * the probability distribution for the position estimate should be updated one step
	 * after the method has been called once.
	 */
	public void update();
	
	/*
	 * returns the currently known true position i.e., after one simulation step
	 * of the robot as (x,y)-pair.
	 */
	public int[] getCurrentTruePosition();
	
	/*
	 * returns the currently available sensor reading obtained for the true position 
	 * after the simulation step 
	 */
	public int[] getCurrentReading();
	
	/*
	 * returns the currently estimated (summed) probability for the robot to be in position
	 * (x,y) in the grid. The different headings are not considered, as it makes the 
	 * view somewhat unclear.  
	 */
	public double getCurrentProb( int x, int y);

	/*
	 * returns the entry of your observation matrix corresponding to the probability 
	 * of getting the sensor reading r expressed as 
	 * position (rX, rY) given the state (x, y, h). 
	 * See page 579, about how the observation matrices O_r are built, 
	 * i.e. O_r(i,i) = P( r | X = i). 
	 * If rX or rY (or both) are -1, the method should return the probability for 
	 * the sensor to return "nothing" given the robot is in pose (or state) (x, y, h).
	 */
	public double getOrXY( int rX, int rY, int x, int y, int h);

	/*
	 * returns the probability entry (Tij) of your transition model T to go from pose 
	 * i = (x, y, h) to pose j = (nX, nY, nH)
	 */	
	public double getTProb( int x, int y, int h, int nX, int nY, int nH);

}
