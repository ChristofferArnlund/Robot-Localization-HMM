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

	public HMMLocalizer(int rows, int cols) {
		this.rows = rows;
		this.cols = cols;

		robotPosition = new int[2];
		// Fixed seed for repeat (seed:1337)
		generator = new Random(1337);
		// generate random starting point for robot
		int randX = generator.nextInt(rows);
		int randY = generator.nextInt(cols);
		// Set the robotposition to a random spot on the board
		robotPosition[0] = randX;
		robotPosition[1] = randY;

		// Generate random Heading
		int randHead = generator.nextInt(4);
		head = randHead;

		// double[][] vals = {{1.,2.,3},{4.,5.,6.},{7.,8.,10.}};
		// Matrix A = new Matrix(vals);
		// https://math.nist.gov/javanumerics/jama/doc/
		/*
		 * T-matrix for 2x2 X00h0 X01h0 X02h0 X03h0 X00h01 X01h1 X01h3 X01h4 etc Y00h1 --
		 * Y00h2 Y00h3 Y00h4 Y01h1 Y01h2 Y01h3 Y01h4
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

		return head;
	}

	@Override
	public void update() {
		moveRobot();

	}

	void moveRobot() {
		double prob = generator.nextDouble();
		ArrayList<Integer> headings = availableNewHeadings(head,robotPosition[0],robotPosition[1]);
		
		//No wall hit in the head direction
		if (prob <= 0.3) {
			// Gets the index of the head and removes it
			if(headings.indexOf(head)!=-1) {
				headings.remove(headings.indexOf(head));
			}
			int tempHead = generator.nextInt(headings.size() - 1);
			head = headings.get(tempHead);
				
		}else if(!headings.contains(head)) {
			int tempHead = generator.nextInt(headings.size() - 1);
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

	private ArrayList<Integer> availableNewHeadings(int currentHeading,int x, int y) {
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
	
	//Returns the observationMaxtrix O 
	public Matrix sensorData(){
		int x = robotPosition[0];
		int y = robotPosition[1];
		int size = rows*cols*4;
		//double[][] sensorObservations = new double[size][size];
		Matrix O = new Matrix(size,size);
		Matrix ONothing = new Matrix(size,size);
		//Get the MAXIMAL & MINIMAL range we can get from the x,y points but not out of bounds
        int[] xRange = {Math.max(x - 2, 0), Math.min(x + 2, rows - 1)};
        int[] yRange = {Math.max(y - 2, 0), Math.min(y + 2, cols - 1)};
		
        //Red Field is the Field with sensor prob of 0.05 (euclidean distance < 2)
        // Yellow Field is the Field with sensor prob of 0.025 (euc. distance >= 2)
        ArrayList<ArrayList<Integer>> redField = new ArrayList<ArrayList<Integer>>();
        ArrayList<ArrayList<Integer>> yellowField = new ArrayList<ArrayList<Integer>>();
        ArrayList<ArrayList<Integer>> theNothingField = new ArrayList<ArrayList<Integer>>();
     
        
        

        
        for (int i = xRange[0]; i <= xRange[1]; i++) {
            for (int j = yRange[0]; j <= yRange[1]; j++) {
            	double prob=0;
            	//Initiate Point that we are investigating and putting a possibility on:
                
            	ArrayList<Integer> possiblePoint = new ArrayList<Integer>();
            	possiblePoint.add(i);
            	possiblePoint.add(j);
            	//Get the euclidian distance between the points
            	double dist = getEuclideanDistance(x,y,i,j);
            	//Add to redField if distance is less than 2 else it has less probability
            	if(dist <= 0.01) {
            		//ugly but does the job
            		prob= 0.1;
            		int Oindex = 4*(4*x+y);
            		for(int f=Oindex;f<Oindex+4;f++) {
                		O.set(f, f, prob);
                	}
            	}
            	else if( dist < 2 && dist >0.01) {
            		
            		redField.add(possiblePoint);
            	}else if(dist >= 2 && dist <= 3){
            	
            		yellowField.add(possiblePoint);
            	}else {
            		theNothingField.add(possiblePoint);
            			
            	}
            		
            	
            }
        }
    	//Get the startIndex for the Transition matrix
    	//I.e first the state (0,0) heading 0,1,2,3 are the first four diagonals
        int rs = redField.size();
        int ys = yellowField.size();
        //Add the redfield to the diagonal Observation Matrix 
        for(int k = 0; k < redField.size();k++) {
        	ArrayList<Integer> theRedDot = redField.get(k);
        	int startIndex = 4*(4*theRedDot.get(0)+theRedDot.get(1));
        	for(int f=startIndex;f<startIndex+4;f++) {
        		O.set(f, f, 0.05);
        	}
        }
        //Add the yellow to the diagonal Observation Matrix 
        for(int k = 0; k < yellowField.size();k++) {
        	ArrayList<Integer> theYellowDot = yellowField.get(k);
        	int startIndex = 4*(4*theYellowDot.get(0)+theYellowDot.get(1));
        	for(int f=startIndex;f<startIndex+4;f++) {
        		O.set(f, f, 0.025);
        	}
        }
        //Add the Nothingfield to the diagonal Observation Matrix 
        double noProb = 1- 0.1 -rs*0.05-ys*0.025; 
        for(int k = 0; k < theNothingField.size();k++) {
        	ArrayList<Integer> theNoDot = theNothingField.get(k);
        	int startIndex = 4*(4*theNoDot.get(0)+theNoDot.get(1));
        	for(int f=startIndex;f<startIndex+4;f++) {
        		ONothing.set(f, f, noProb);
        	}
        }
    	
		//O.print(5, 3);
		return O;
		
	}

	 private double getEuclideanDistance(int x, int y, int a, int b) {
	        return Math.round(Math.sqrt(Math.pow(Math.abs(x - a), 2) + Math.pow(Math.abs(y - b), 2)));
	    }
	@Override
	public int[] getCurrentTruePosition() {

		return robotPosition;
	}
	public Matrix initiateTransitionMatrix() {
		int size= rows*cols*4;
		Matrix T = new Matrix(size,size);
		
		for(int i = 0;i<size;i++) {
	
			
			for(int j =0;j<size;j++) {
				
			ArrayList<Integer> stateTranslations =  stateTranslation(i,j);
			int oldX=stateTranslations.get(0);
			int oldY = stateTranslations.get(1);
			int newX=stateTranslations.get(2);
			int newY = stateTranslations.get(3);
			
			int oldHead = stateTranslations.get(5);
			int newHead = stateTranslations.get(4);
			ArrayList<Integer> availableHeadings = availableNewHeadings(oldHead, oldX, oldY);
			int nbrOfAvailableHeadings= availableHeadings.size();
			double dist=getEuclideanDistance(oldX, oldY, newX, newY);
			
			//Not colliding/
			if(availableHeadings.contains(oldHead) && (dist >0.99 && dist < 1.01)){
				if(oldHead==newHead) {
					T.set(i, j, 0.7);	
				}else {
					
					T.set(i, j, 0.3/(nbrOfAvailableHeadings-1));
				}
				 
				
			}else {
				T.set(i, j, 0); 
			}
			}
		}
		
		T.print(2, 2);
		
		return T;
		
	}
	private ArrayList<Integer> stateTranslation(int i, int j) {
		ArrayList<Integer> res = new ArrayList<Integer>();
		int newHead = j  % 4;
		int oldHead = i % 4;
		
		int oldX =(int) Math.floor(4*i/(rows*cols*4));
		int oldY = (int) Math.floor((i/4)%4);
		
		
		int newX =(int) Math.floor(4*j/(rows*cols*4));
		int newY = (int) Math.floor((j/4)%4);
		
		res.add(oldX);
		res.add(oldY);
		res.add(newY);
		res.add(newY);
		res.add(newHead);
		res.add(oldHead);
		return res;
		
	}
 
	@Override
	// Return the output of the sensor, so 0.1 on the actual robot location
	// then 0.05 the 0.025 then "nothing"
	public int[] getCurrentReading() {
		// Check if probability overcomes the nothing "matrix"
		return null;
	}

	@Override
	public double getCurrentProb(int x, int y) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getOrXY(int rX, int rY, int x, int y, int h) {
		// TODO Auto-generated method stub
		return 0;
	}

	// Obtain the Transition probability for the robot
	// That is the probability for the new heading
	public double getTProb(int x, int y, int h, int nX, int nY, int nH) {

		return 0;
	}

}
