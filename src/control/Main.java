package control;


import model.HMMLocalizer;
import view.RobotLocalizationViewer;

public class Main {

	
	public static void main( String[] args) {
		
		/*
		 * generate you own localiser / estimator wrapper here to plug it into the 
		 * graphics class.
		 */
		HMMLocalizer l = new HMMLocalizer( 8,8);
		/*for(int i =0;i<100;i++) {
			l.update();	
		}
		System.out.println(l.getCurrentTruePosition()[0]);
		System.out.println(l.getCurrentTruePosition()[1]);
		*/
//		l.sensorData(3,0);
		//l.initiateTransitionMatrix();
		//l.update();
		//l.update();
		RobotLocalizationViewer viewer = new RobotLocalizationViewer( l);

		/*
		 * this thread controls the continuous update. If it is not started, 
		 * you can only click through your localisation stepwise
		 */
		new LocalizationDriver( 500, viewer).start();
	}
}	