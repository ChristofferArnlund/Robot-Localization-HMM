package control;

import view.*;

public class LocalizationDriver extends Thread {
	
	private RobotLocalizationViewer l;
	long timer;
	
	public LocalizationDriver( long stepTime, RobotLocalizationViewer v) {
		this.l = v;
		this.timer = stepTime;
	}
	
	public void run() {
		while( !isInterrupted()) {
			
			
			try{
				l.updateContinuously();
				sleep( timer);
			} catch( InterruptedException e) {
				System.out.println( "oops");
			}

		}
	}
	
}