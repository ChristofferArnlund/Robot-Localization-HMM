package model;

import java.util.ArrayList;
import java.util.Random;
import java.util.Vector;

import Jama.Matrix;
import control.EstimatorInterface;

public class HMMLocalizer implements EstimatorInterface {

	private int rows, cols, head;
	private int[] robotPosition;
	private Random generator;
	private Matrix f,T;
	private ArrayList<ArrayList<Integer>> redField;
	private ArrayList<ArrayList<Integer>> yellowField;
	public HMMLocalizer(int rows, int cols) {
		this.rows = rows;
		this.cols = cols;
	
		
		robotPosition = new int[2];
		// Fixed seed for repeat (seed:1338)
		generator = new Random(1338);
		// generate random starting point for robot
		int randX = generator.nextInt(rows);
		int randY = generator.nextInt(cols);
		// Set the robotposition to a random spot on the board
		
		//robotPosition[0] = randX;
		//robotPosition[1] = randY;

		robotPosition[0] = 0;
		robotPosition[1] = 3;

		
		// Generate random Heading
		int randHead = generator.nextInt(4);
		head = randHead;
		f = initateForward();
		T = initiateTransitionMatrix();
		// double[][] vals = {{1.,2.,3},{4.,5.,6.},{7.,8.,10.}};
		// Matrix A = new Matrix(vals);
		// https://math.nist.gov/javanumerics/jama/doc/
		/*
		 * T-matrix for 2x2 X00h0 X01h0 X02h0 X03h0 X00h01 X01h1 X01h3 X01h4 etc Y00h1
		 * -- Y00h2 Y00h3 Y00h4 Y01h1 Y01h2 Y01h3 Y01h4
		 * 
		 */

	}

	@Override
	public int getNumRows() {

		return rows;
	}

	@Override
	public int getNumCols() {

		return cols;
	}

	@Override
	public int getNumHead() {

		return 4;
	}

	@Override
	public void update() {
		moveRobot();
		Matrix observedMatrix = sensorData(robotPosition[0],robotPosition[1]);
		forward(observedMatrix,T);

	}

	private void forward(Matrix observedMatrix,Matrix T) {
		//observedMatrix.print(2, 3);
		
		Matrix temp =observedMatrix.times(T.transpose());
		//temp.print(2, 3);
		Matrix res = temp.times(f);
		//res.print(2, 3);
		double[] f_sub = res.getMatrix(0, rows*cols*4-1,0,0).getRowPackedCopy();
		//double tot=0;
		//for(int i=0;i<f_sub.length;i++) {
		//	tot += f_sub[i]*f_sub[i];
		//}
		//tot = Math.sqrt(tot);
		//f_sub.print(2, 3);
		double norm = res.getMatrix(0, rows*cols*4-1,0,0).norm1();
		//double norm = res.normF();
		//f.print(2, 3);
		f.print(2, 3);
		f= res.times(1/norm);
		f.print(2, 3);
	}
	private Matrix initateForward() {
		int size = (rows * cols * 4);
		double prob = 1.0 / size;
		Matrix f = new Matrix(size,size,prob);
		return f;
	}
	public void moveRobot() {
		double prob = generator.nextDouble();
		ArrayList<Integer> headings = availableNewHeadings(head, robotPosition[0], robotPosition[1]);

		// No wall hit in the head direction
		if (prob <= 0.3) {
			// Gets the index of the head and removes it
			if (headings.indexOf(head) != -1) {
				headings.remove(headings.indexOf(head));
			}
			int tempHead = generator.nextInt(headings.size());
			head = headings.get(tempHead);
			// if the new headings dont contains old head
		} else if (!headings.contains(head)) {
			int tempHead = generator.nextInt(headings.size());
			head = headings.get(tempHead);
		}

		step(head);

	}

	private void step(int newHead) {
		switch (newHead) {
		case 0:
			robotPosition[0] -= 1;
			break;
		case 1:
			robotPosition[1] += 1;
			break;
		case 2:
			robotPosition[0] += 1;
			break;
		case 3:
			robotPosition[1] -= 1;
			break;

		}

	}

	private ArrayList<Integer> availableNewHeadings(int currentHeading, int x, int y) {
		ArrayList<Integer> headings = new ArrayList<Integer>();
		headings.add(0);
		headings.add(1);
		headings.add(2);
		headings.add(3);

		if (x == 0) {
			headings.remove(headings.indexOf(0));
		}
		if (y == 0) {
			headings.remove(headings.indexOf(3));
		}
		if (x == rows - 1) {
			headings.remove(headings.indexOf(2));
		}
		if (y == cols - 1) {
			headings.remove(headings.indexOf(1));
		}

		return headings;

	}

	// Returns the observationMaxtrix O
	public Matrix sensorData(int x, int y) {
	
		int size = rows * cols * 4;
		// double[][] sensorObservations = new double[size][size];
		Matrix O = new Matrix(size, size);
		Matrix ONothing = new Matrix(size, size);
		// Get the MAXIMAL & MINIMAL range we can get from the x,y points but not out of
		// bounds
		int[] xRange = { Math.max(x - 2, 0), Math.min(x + 2, rows - 1) };
		int[] yRange = { Math.max(y - 2, 0), Math.min(y + 2, cols - 1) };

		// Red Field is the Field with sensor prob of 0.05 (euclidean distance < 2)
		// Yellow Field is the Field with sensor prob of 0.025 (euc. distance >= 2)
		redField = new ArrayList<ArrayList<Integer>>();
		yellowField = new ArrayList<ArrayList<Integer>>();
		ArrayList<ArrayList<Integer>> theNothingField = new ArrayList<ArrayList<Integer>>();

		for (int i = xRange[0]; i <= xRange[1]; i++) {
			for (int j = yRange[0]; j <= yRange[1]; j++) {
				double prob = 0;
				// Initiate Point that we are investigating and putting a possibility on:

				ArrayList<Integer> possiblePoint = new ArrayList<Integer>();
				possiblePoint.add(i);
				possiblePoint.add(j);
				// Get the euclidian distance between the points
				double dist = getEuclideanDistance(x, y, i, j);
				// Add to redField if distance is less than 2 else it has less probability
				if (dist <= 0.01) {
					// ugly but does the job
					prob = 0.1;
					int Oindex = 4 * (cols * x + y);
					for (int f = Oindex; f < Oindex + 4; f++) {
						O.set(f, f, prob);
					}
				} else if (dist < 2 && dist > 0.01) {

					redField.add(possiblePoint);
				} else if (dist >= 2 && dist <= 3) {

					yellowField.add(possiblePoint);
				} else {
					theNothingField.add(possiblePoint);

				}

			}
		}
		// Get the startIndex for the Transition matrix
		// I.e first the state (0,0) heading 0,1,2,3 are the first four diagonals
		int rs = redField.size();
		int ys = yellowField.size();
		// Add the redfield to the diagonal Observation Matrix
		for (int k = 0; k < redField.size(); k++) {
			ArrayList<Integer> theRedDot = redField.get(k);
			int startIndex = 4 * (cols * theRedDot.get(0) + theRedDot.get(1));
			for (int f = startIndex; f < startIndex + 4; f++) {
				O.set(f, f, 0.05);
			}
		}
		// Add the yellow to the diagonal Observation Matrix
		for (int k = 0; k < yellowField.size(); k++) {
			ArrayList<Integer> theYellowDot = yellowField.get(k);
			int startIndex = 4 * (cols * theYellowDot.get(0) + theYellowDot.get(1));
			for (int f = startIndex; f < startIndex + 4; f++) {
				O.set(f, f, 0.025);
			}
		}
		// Add the Nothingfield to the diagonal Observation Matrix
		double noProb = 1 - 0.1 - rs * 0.05 - ys * 0.025;
		for (int k = 0; k < theNothingField.size(); k++) {
			ArrayList<Integer> theNoDot = theNothingField.get(k);
			int startIndex = 4 * (cols * theNoDot.get(0) + theNoDot.get(1));
			for (int f = startIndex; f < startIndex + 4; f++) {
				ONothing.set(f, f, noProb);
			}
		}

	/*for(int q = 0;q < rows*cols*4;q++) {
		System.out.println(O.get(q, q));
	}*/
		return O;

	}

	private double getEuclideanDistance(int x, int y, int a, int b) {
		return (Math.sqrt(Math.pow(Math.abs(x - a), 2) + Math.pow(Math.abs(y - b), 2)));
	}

	@Override
	public int[] getCurrentTruePosition() {

		return robotPosition;
	}

	public Matrix initiateTransitionMatrix() {
		int size = rows * cols * 4;
		Matrix T = new Matrix(size, size);

		for (int i = 0; i < size; i++) {

		

				ArrayList<Integer> stateTranslations = stateTranslation(i, 0);
				int oldX = stateTranslations.get(0);
				int oldY = stateTranslations.get(1);
				int oldHead = stateTranslations.get(4);
				

				ArrayList<Integer> availableHeadings = availableNewHeadings(oldHead, oldX, oldY);
		
				//Check if the position + oldHead exists
				if (availableHeadings.contains(oldHead)){
					int nextX = oldX,nextY = oldY;
					switch (oldHead) {
					case 0:
						nextX -= 1;
						break;
					case 1:
						nextY += 1;
						break;
					case 2:
						nextX += 1;
						break;
					case 3:
						nextY -= 1;
						break;

					

					}
					int  newJ = getMatrixPosition(nextX,nextY,oldHead);
					
						T.set(i,newJ,0.7);
						availableHeadings.remove(availableHeadings.indexOf(oldHead));
				}
				if(!availableHeadings.isEmpty()) {
					for(int theNewHead : availableHeadings) {
						int nextX = oldX,nextY = oldY;
						switch (theNewHead) {
						case 0:
							nextX -= 1;
							break;
						case 1:
							nextY += 1;
							break;
						case 2:
							nextX += 1;
							break;
						case 3:
							nextY -= 1;
							break;
						}
						int  newJ = getMatrixPosition(nextX,nextY,theNewHead);
						
						T.set(i,newJ,0.3/(availableHeadings.size()));
					}
				}
				
			}
		
		Matrix tNorm = normalizeByRow(T);
		
	
		//tNorm.print(1, 2);
		
		return tNorm;

	}

	private int getMatrixPosition(int oldX, int oldY, int oldHead) {
		
		int j = 4*(cols*oldX+oldY)+oldHead;
		
		return j;

		
	}

	private Matrix normalizeByRow(Matrix t) {
		double norm = t.normF();
		
		for(int mi =0; mi<rows*cols*4;mi++) {
			Matrix Tsub = t.getMatrix(mi,mi,0,rows*cols*4-1);
			norm = Tsub.normInf();
			if(norm!=0) {
				Tsub = Tsub.times(1/norm);
				t.setMatrix(mi,mi, 0,rows*cols*4-1, Tsub);	
			}
			 		
		}
		return t;
	}

	private ArrayList<Integer> stateTranslation(int i, int j) {
		ArrayList<Integer> res = new ArrayList<Integer>();
		int newHead = j % 4;
		int oldHead = i % 4;

		int oldX = (int) Math.floor(cols * i / (rows * cols * 4));
		int oldY = (int) Math.floor((i / 4) % rows);

		int newX = (int) Math.floor(cols * j / (rows * cols * 4));
		int newY = (int) Math.floor((j / 4) % rows);

		res.add(oldX);
		res.add(oldY);
		res.add(newX);
		res.add(newY);
		res.add(oldHead);
		res.add(newHead);
		return res;

	}

	@Override
	/*
	 * returns the currently available sensor reading obtained for the true position 
	 * after the simulation step 
	 */
	// Return the output of the sensor, so 0.1 on the actual robot location
	// then 0.05 the 0.025 then "nothing"
	public int[] getCurrentReading() {
		// Check if probability overcomes the nothing "matrix"
		double sensorProb = generator.nextDouble();
		
		if(sensorProb <= 0.1) {
			return robotPosition;
		}else if (sensorProb <= 0.1  + 0.05 * redField.size()) {
			int rand = generator.nextInt(redField.size());
			int[] fakePos = {redField.get(rand).get(0),redField.get(rand).get(1)};
			return fakePos;
			
		}else if(sensorProb <= 0.1  +  0.05 * redField.size() + 0.025 * yellowField.size()) {
			int rand = generator.nextInt(yellowField.size());
			int[] fakePos = {yellowField.get(rand).get(0),yellowField.get(rand).get(1)};
			return fakePos;
		}
		
		return null;
		
	}
	/*
	 * returns the currently estimated (summed) probability for the robot to be in position
	 * (x,y) in the grid. The different headings are not considered, as it makes the 
	 * view somewhat unclear.  
	 */
	@Override
	public double getCurrentProb(int x, int y) {
		double totalProb=0;
		int j = getMatrixPosition(x, y, 0);
		 Matrix f_sub = f.getMatrix(j,j+3,0,0);
		 
		for(int k = 0;k < 4; k++) {
			totalProb += f_sub.get(k,0);
		}
		return totalProb;
	}
	/*
	 * returns the entry of your observation matrix corresponding to the probability 
	 * of getting the sensor reading r expressed as 
	 * position (rX, rY) given the state (x, y, h). 
	 * See page 579, about how the observation matrices O_r are built, 
	 * i.e. O_r(i,i) = P( r | X = i). 
	 * If rX or rY (or both) are -1, the method should return the probability for 
	 * the sensor to return "nothing" given the robot is in pose (or state) (x, y, h).
	 */
	@Override
	public double getOrXY(int rX, int rY, int x, int y, int h) {
		Matrix O = sensorData(x,y);
		if( rX==-1 || rY ==-1) {
			return 1 - 0.1 - redField.size()*0.05 - yellowField.size() * 0.025;
		}
		int j = getMatrixPosition(rX, rY, h);
		return O.get(j, j);
	}
	/*
	 * returns the probability entry (Tij) of your transition model T to go from pose 
	 * i = (x, y, h) to pose j = (nX, nY, nH)
	 */	
	// Obtain the Transition probability for the robot
	// That is the probability for the new heading
	public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
		int i = getMatrixPosition(x, y, h);
		int j = getMatrixPosition(nX, nY, nH);
		double prob = T.get(i, j);
		return prob;
	}

}
