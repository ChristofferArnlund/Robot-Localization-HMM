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
	private Matrix f, T;
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

		robotPosition[0] = randX;
		robotPosition[1] = randY;

		
		// Generate random Heading
		int randHead = generator.nextInt(4);
		head = randHead;
		f = initateForward();
		T = initiateTransitionMatrix();


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
		int[] sensorPosition = getCurrentReading();
		Matrix observedMatrix;
		if(sensorPosition != null) {
			
			 observedMatrix = sensorData(sensorPosition[0], sensorPosition[1]);
		}else {
			 observedMatrix = sensorData(-1,-1);
		}
		
		forward(observedMatrix, T);

	}
	//Implements forward algorithm, Hidden Markov Model 
	//simplified model with a single discrete State Variable
	//f_1:t+1=alpha (O_t+1*T^T*f_1:t)
	private void forward(Matrix observedMatrix, Matrix T) {
		
		Matrix temp = observedMatrix.times(T.transpose());
		
		Matrix res = temp.times(f);
		
		double norm = res.norm1();
		
		//Normalise column matrix
		f = res.times(1 / norm);
		
	}
	/*
	 * Initiates the forwards column Matrix with 
	 * equal probabilities.
	 */
	private Matrix initateForward() {
		int size = (rows * cols * 4);
		double prob = 1.0 / size;
		Matrix f = new Matrix(size, 1, prob);
		return f;
	}
	
	/*
	 * Makes the Robot move in a specified heading on the board
	 */
	public void moveRobot() {
		double prob = generator.nextDouble();
		ArrayList<Integer> headings = availableNewHeadings(head, robotPosition[0], robotPosition[1]);

		//With a small probability 
		if (prob <= 0.3) {
			
			// If the old heading exists in the new available headings, remove it. 
			if (headings.indexOf(head) != -1) {
				headings.remove(headings.indexOf(head));
			}
			//Get a new random heading
			int tempHead = generator.nextInt(headings.size());
			head = headings.get(tempHead);
			
			// if the new headings don't contains old heading direction
		} else if (!headings.contains(head)) {
			//Get a new random heading
			int tempHead = generator.nextInt(headings.size());
			head = headings.get(tempHead);
		}
		
		//Move robot 
		step(head);

	}
	/*
	 * changes the x,y coordinates of the robot in the right direction given a heading
	 */
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
	/*
	 * Returns headings the robot can go given a current heading and a x,y coordinate. 
	 * Does not remove the current heading.
	 */
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


	/*
	 * Sets the red field (all positions around the robot with a distance of <2 but not itself)
	 * and the yellowfield (all positions around the robot with a distance of >= 2 but <3 )
	 * given a point. 
	 */
	public void createFields(int x, int y) {

		// Get the max and min range we can get from the x,y points but not out of
		// bounds
		int[] xRange = { Math.max(x - 2, 0), Math.min(x + 2, rows - 1) };
		int[] yRange = { Math.max(y - 2, 0), Math.min(y + 2, cols - 1) };

		// Red Field is the Field with sensor prob of 0.05 (euclidean distance < 2)
		// Yellow Field is the Field with sensor prob of 0.025 (euc. distance >= 2)
		redField = new ArrayList<ArrayList<Integer>>();
		yellowField = new ArrayList<ArrayList<Integer>>();

		for (int i = xRange[0]; i <= xRange[1]; i++) {
			for (int j = yRange[0]; j <= yRange[1]; j++) {
				
				
				// Initiate Point that we are investigating and putting a possibility on:
				ArrayList<Integer> possiblePoint = new ArrayList<Integer>();
				possiblePoint.add(i);
				possiblePoint.add(j);
				
				double dist = getEuclideanDistance(x, y, i, j);
				
				// Add to redField if distance is less than 2 and some rounding errors.
				if (dist < 2 && dist > 0.01) {

					redField.add(possiblePoint);
				} else if (dist >= 2 && dist < 3) {

					yellowField.add(possiblePoint);
				}

			}
		}
		
		
	}
	/*
	 * Returns the Observation Matrix given a sensoryReading x,y
	 * If sensor reading was flawed i.e -1,-1 then return the "nothing Matrix"
	 */
	public Matrix sensorData(int x, int y) {
		
		//Generate the Fields for the current sensory Reading.
		createFields(x, y);
		
		int size = rows * cols * 4;
		Matrix O = new Matrix(size, size);
		
		//flawed reading generates nothing Matrix
		if(x == -1 || y == -1) {
			for(int i = 0; i < rows ; i++) {
				for(int j = 0 ; j < cols ; j++) {
					//Generate the Fields for the current point we are looking at  
					createFields(i,j);
					
					//Get the O-Matrix index
					int Oindex = 4 * (cols * i + j);
					
					//The O matrix has the same value for all headings, and since it is diagonal and
					//the headings are consecutive, add this point 4 times. 
					for (int f = Oindex; f < Oindex + 4; f++) {
						O.set(f,f, 1 - 0.1 - redField.size() * 0.05 - yellowField.size() * 0.025);
					}
					
				}
			}
			
			return O;
		}
		
					
	
		//Obtain O-index for the current point.
		//Set the currents point probabilty to 0.1
		int Oindex = 4 * (cols * x + y);
		
		for (int f = Oindex; f < Oindex + 4; f++) {
			O.set(f, f, 0.1);
		}
		
		// Get the startIndex for the Transition matrix
		// I.e first the state (0,0) heading 0,1,2,3 are the first four diagonals
		int rs = redField.size();
		int ys = yellowField.size();
		// Add the redfield to the diagonal Observation Matrix
		for (int k = 0; k < redField.size(); k++) {
			//Get the points around our reading that should have a probability of 0.05
			ArrayList<Integer> theRedDot = redField.get(k);
			int startIndex = 4 * (cols * theRedDot.get(0) + theRedDot.get(1));
			for (int f = startIndex; f < startIndex + 4; f++) {
				O.set(f, f, 0.05);
			}
		}
		// Add the yellowField to the diagonal Observation Matrix
		for (int k = 0; k < yellowField.size(); k++) {
			//Get the points around our reading that should have a probability of 0.025
			ArrayList<Integer> theYellowDot = yellowField.get(k);
			int startIndex = 4 * (cols * theYellowDot.get(0) + theYellowDot.get(1));
			for (int f = startIndex; f < startIndex + 4; f++) {
				O.set(f, f, 0.025);
			}
		}

		return O;

	}
	/*
	 * Returns the Euclidean distance for two points.
	 */
	private double getEuclideanDistance(int x, int y, int a, int b) {
		return (Math.sqrt(Math.pow(Math.abs(x - a), 2) + Math.pow(Math.abs(y - b), 2)));
	}
	/*
	 * @see control.EstimatorInterface#getCurrentTruePosition()
	 */
	@Override
	public int[] getCurrentTruePosition() {

		return robotPosition;
	}
	/*
	 * Return the Transition Matrix 
	 */
	public Matrix initiateTransitionMatrix() {
		int size = rows * cols * 4;
		Matrix T = new Matrix(size, size);
		
		for (int i = 0; i < size; i++) {
			//Translate the the Matrix index to real board coordinates X,Y,heading aka "StateTranslation"
			ArrayList<Integer> stateTranslations = stateTranslation(i, 0);
			int oldX = stateTranslations.get(0);
			int oldY = stateTranslations.get(1);
			int oldHead = stateTranslations.get(4);
			
			//Check what are the available Headings for current Point
			ArrayList<Integer> availableHeadings = availableNewHeadings(oldHead, oldX, oldY);

			// Check if the position + oldHead exists
			//Check available positions we can take with this point,
			//Sets the probability to 0.7 if the next point is in the same heading, else 
			//iterate through the rest of the points and set the probability to 0.3
			if (availableHeadings.contains(oldHead)) {
				int nextX = oldX, nextY = oldY;
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
				int newJ = getMatrixPosition(nextX, nextY, oldHead);

				T.set(i, newJ, 0.7);
				availableHeadings.remove(availableHeadings.indexOf(oldHead));
			}
			if (!availableHeadings.isEmpty()) {
				for (int theNewHead : availableHeadings) {
					int nextX = oldX, nextY = oldY;
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
					int newJ = getMatrixPosition(nextX, nextY, theNewHead);

					T.set(i, newJ, 0.3 / (availableHeadings.size()));
				}
			}

		}
		
		//normalize the vector since the sum of the row should be 1. 
		Matrix tNorm = normalizeByRow(T);

		return tNorm;

	}
	/*
	 * Return the j Position in the T matrix given X,Y and heading
	 */
	private int getMatrixPosition(int oldX, int oldY, int oldHead) {

		int j = 4 * (cols * oldX + oldY) + oldHead;

		return j;

	}
	/*
	 * Normalizes matrix row by row
	 */
	private Matrix normalizeByRow(Matrix t) {
		double norm = t.normF();

		for (int mi = 0; mi < rows * cols * 4; mi++) {
			Matrix Tsub = t.getMatrix(mi, mi, 0, rows * cols * 4 - 1);
			norm = Tsub.normInf();
			if (norm != 0) {
				Tsub = Tsub.times(1 / norm);
				t.setMatrix(mi, mi, 0, rows * cols * 4 - 1, Tsub);
			}

		}
		return t;
	}
	/*
	 * Translates i, j coordinates in T matrix to X,Y,Heading coordinates
	 */
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
	/*
	 * 
	 * @see control.EstimatorInterface#getCurrentReading()
	 */
	@Override
	public int[] getCurrentReading() {
		// Check if probability overcomes the nothing "matrix"
		double sensorProb = generator.nextDouble();
		createFields(robotPosition[0], robotPosition[1]);
		if (sensorProb <= 0.1) {
			return robotPosition;
		} else if (sensorProb <= 0.1 + 0.05 * redField.size()) {
			int rand = generator.nextInt(redField.size());
			int[] fakePos = { redField.get(rand).get(0), redField.get(rand).get(1) };
			return fakePos;

		} else if (sensorProb <= 0.1 + 0.05 * redField.size() + 0.025 * yellowField.size()) {
			int rand = generator.nextInt(yellowField.size());
			int[] fakePos = { yellowField.get(rand).get(0), yellowField.get(rand).get(1) };
			return fakePos;
		}

		return null;

	}

	/*
	 * 
	 * @see control.EstimatorInterface#getCurrentProb(int, int)
	 */
	@Override
	public double getCurrentProb(int x, int y) {
		double totalProb = 0;
		int j = getMatrixPosition(x, y, 0);
		Matrix f_sub = f.getMatrix(j, j + 3, 0, 0);
		f_sub.print(2, 4);
		for (int k = 0; k < 4; k++) {
			totalProb += f_sub.get(k, 0);
		}
		return totalProb;
	}

	/*
	 * 
	 * @see control.EstimatorInterface#getOrXY(int, int, int, int, int)
	 */
	@Override
	public double getOrXY(int rX, int rY, int x, int y, int h) {
		
		Matrix O = sensorData(rX, rY);
		int j = getMatrixPosition(x, y, h);
		return O.get(j, j);
	}

	/*
	 * 
	 * @see control.EstimatorInterface#getTProb(int, int, int, int, int, int)
	 */
	public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
		int i = getMatrixPosition(x, y, h);
		int j = getMatrixPosition(nX, nY, nH);
		double prob = T.get(i, j);
		return prob;
	}

}
