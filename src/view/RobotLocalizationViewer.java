package view;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.LineBorder;

import control.*;


public class RobotLocalizationViewer {
	
	private JFrame viewer;
	private JTextField[][][] states;
	private JPanel[][] positions;
	private int rows, cols, head;
	private EstimatorInterface loc;
	private int sXCount, sYCount, tXCount, tYCount, tHCount;
	private boolean runFlag, initFlag;
	
	public RobotLocalizationViewer( EstimatorInterface l) {
		loc = l;
		this.rows = loc.getNumRows();
		this.cols = loc.getNumCols();
		this.head = loc.getNumHead();
		
		runFlag = initFlag = false;
		
		sXCount = sYCount = 0;
		tXCount = tYCount = tHCount = 0;
		
		viewer = new JFrame( "RobotLocalizer");	
		viewer.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE);
		viewer.setResizable( false);
		JPanel fieldPanel = new JPanel();
		Dimension dState = new Dimension( 40, 40);
		Dimension dPos = new Dimension( 120, 120);
		fieldPanel.setLayout(new GridLayout( rows, cols));
		
		positions = new JPanel[rows][cols];
		states = new JTextField[rows][cols][5]; // fixed number of display fields, fewer should work
		
		for( int i=0; i<rows; i++) {
			for( int j=0; j<cols; j++) {
				positions[i][j] = new JPanel();
				positions[i][j].setPreferredSize( dPos);
				positions[i][j].setLayout( new BorderLayout());
				
				for( int h=0; h<5; h++) {
					states[i][j][h] = new JTextField();
					states[i][j][h].setPreferredSize( dState);
					states[i][j][h].setLayout(null);
					states[i][j][h].setBackground( Color.white);
					states[i][j][h].setText( "");
					states[i][j][h].setHorizontalAlignment( JTextField.CENTER);
					states[i][j][h].setBorder(javax.swing.BorderFactory.createEmptyBorder());
				}
				positions[i][j].setBorder( new LineBorder( Color.black));
				positions[i][j].add( states[i][j][0], BorderLayout.NORTH);
				positions[i][j].add( states[i][j][1], BorderLayout.EAST);
				positions[i][j].add( states[i][j][2], BorderLayout.SOUTH);
				positions[i][j].add( states[i][j][3], BorderLayout.WEST);
				positions[i][j].add( states[i][j][4], BorderLayout.CENTER);
				fieldPanel.add( positions[i][j]);
				
			}
		}
		
		JPanel buttonPanel = new JPanel();
		JButton initButton = new JButton( "Init filter");
		initButton.addActionListener( new ActionListener() {
			public void actionPerformed( ActionEvent e) {
				initLocViewer();
			}
		});
	
		JButton stepButton = new JButton( "One step");
		stepButton.addActionListener( new ActionListener() {
			public void actionPerformed( ActionEvent e) {
				updateOneStep();
			}
		});
		
		JButton runButton = new JButton( "Go");
		runButton.addActionListener( new ActionListener() {
			public void actionPerformed( ActionEvent e) {
				setRunFlag( true);
			}
		});
		JButton stopButton = new JButton( "Stop");
		stopButton.addActionListener( new ActionListener() {
			public void actionPerformed( ActionEvent e) {
				setRunFlag( false);
			}
		});

		
		JPanel modelButtonPanel = new JPanel();
		JButton transButton = new JButton( "Show transitions");
		transButton.addActionListener( new ActionListener() {
			public void actionPerformed( ActionEvent e) {
				updateTransitionView();

			}
		});
		
		JButton sensorButton = new JButton( "Show sensor");
		sensorButton.addActionListener( new ActionListener() {
			public void actionPerformed( ActionEvent e) {
				updateSensorView();
			}
		});
		
		
		buttonPanel.add( initButton);
		
		buttonPanel.add( stepButton);
		buttonPanel.add( runButton);
		buttonPanel.add( stopButton);
		
		modelButtonPanel.setLayout( new GridLayout( 2, 1));
		modelButtonPanel.add( transButton);
		modelButtonPanel.add( sensorButton);
		
		
		viewer.add( fieldPanel, BorderLayout.CENTER);
		viewer.add( buttonPanel, BorderLayout.SOUTH);
		viewer.add( modelButtonPanel, BorderLayout.WEST);
		
		viewer.pack();
		viewer.setVisible( true);
		
	}
	

	public synchronized void setRunFlag( boolean run) {
		runFlag = run;
		notifyAll();
	}
	
	public synchronized void initLocViewer( ){		

		int[] start = loc.getCurrentTruePosition();
		
		updateViewer( start[0], start[1], -1, -1);
		initFlag = true;
	}

	public synchronized void updateOneStep( ){		
		if( initFlag) {
			loc.update();
			int[] tXY = loc.getCurrentTruePosition();
			int[] sXY = loc.getCurrentReading();
			if( sXY != null)
				updateViewer(  tXY[0], tXY[1], sXY[0], sXY[1]);	
			else
				updateViewer(  tXY[0], tXY[1], -1, -1);	
					
		}
	}
	
	public synchronized void updateContinuously() throws InterruptedException {
		while( !runFlag) wait();
		
		updateOneStep();
	}

	
	public void updateViewer( int tX, int tY, int sX, int sY) {
		int x, y, h, maxX, maxY;
		double posProb, posProbMax;
		
		posProb = 0.0;
		posProbMax = 0.0;
		maxX = maxY = -1;
		for( x=0; x<rows; x++) {
			for( y=0; y<cols; y++) {

				posProb = 0.0;
				posProb = loc.getCurrentProb( x, y);
				states[x][y][0].setText( String.format("%.4f", posProb));	

				for( h=1; h<5; h++) {
					states[x][y][h].setText( "");				
				}


				if( posProb == 0.0) {
					for( h=0; h<5; h++) {
						states[x][y][h].setBackground(Color.white);
					}
				} else if( posProb <= 0.1) {
					for( h=0; h<5; h++) {
						states[x][y][h].setBackground(Color.yellow);
					}
					
				} else if( posProb <= 0.3) {
					for( h=0; h<5; h++) {
						states[x][y][h].setBackground(Color.orange);
					}
				} else {
					for( h=0; h<5; h++) {
						states[x][y][h].setBackground(Color.red);
					}
				}
				if( posProb > posProbMax) {
					posProbMax = posProb;
					maxX = x;
					maxY = y;
				}
	
			}
		}
		if( maxX != -1) {
			states[maxX][maxY][0].setBackground(Color.lightGray);
			states[maxX][maxY][1].setBackground(Color.lightGray);
			states[maxX][maxY][2].setBackground(Color.lightGray);
			states[maxX][maxY][3].setBackground(Color.lightGray);
		}
			
		states[tX][tY][4].setBackground(Color.black);
		if( sX != -1)
			states[sX][sY][4].setBackground(Color.cyan);
		
		viewer.repaint();				
	}

	public void updateTransitionView() {

		int x, y, h;
		
		for( x=0; x<rows; x++) {
			for( y=0; y<cols; y++) {
				for( h=0; h<head; h++) {
					states[x][y][h].setText( String.format("%.2f", loc.getTProb( tXCount, tYCount, tHCount, x, y, h)));
					states[x][y][h].setBackground(Color.white);
						
				}
				states[x][y][4].setBackground(Color.white);
				states[x][y][4].setText( "");

			}
		}
		
		states[tXCount][tYCount][tHCount].setBackground(Color.cyan);
		//states[tX][tY][4].setText( Integer.toString( tH));
		if( ++tHCount == head) { 
			tHCount = 0;
			if( ++tYCount == cols) {
				tYCount = 0;
				if( ++tXCount == rows) {
					tXCount = 0;
				}
			}
		}

		viewer.repaint();

	}
	
	public void updateSensorView() {
		int x, y, h;
		String s = "";
	
		if( sYCount == cols) {
			if( sXCount == rows) {				
				sYCount = -1;
				sXCount = -1;
			} else {
				sYCount = 0;
			}
		}

		
		for( x=0; x<rows; x++) {
			for( y=0; y<cols; y++) {
				s = "";
				states[x][y][0].setText( s);
				states[x][y][4].setBackground(Color.white);

				for( h=0; h<4; h++) {
					states[x][y][h].setBackground(Color.white);		
					s = String.format("%.3f", loc.getOrXY( sXCount, sYCount, x, y, h));
					states[x][y][h].setText( s);
				}
				
					
				System.out.print(s + ", ");
				
			}
			System.out.println(";");

		}	
		
		if( sXCount >=0 && sYCount>=0)
			states[sXCount][sYCount][4].setBackground(Color.cyan);
		
		viewer.repaint();

		if( ++sYCount == cols || sXCount == -1)
			sXCount++;
				
	}
	

}