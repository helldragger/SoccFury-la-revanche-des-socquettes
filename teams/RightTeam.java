/*
 * RightTeam.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;

/**
 * Example of a simple strategy for a robot
 * soccer team.
 * It illustrates how to use many of the sensor and
 * all of the motor methods of a SocSmall robot.
 * (c)1997 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */


public class RightTeam extends ControlSystemSS
	{
	/**
	Configure the control system.  This method is
	called once at initialization time.  You can use it
	to do whatever you like.
	*/
	public void Configure()
		{
		// not used in this example.
		}
		
	
	/**
	Called every timestep to allow the control system to
	run.
	*/
	public int TakeStep()
		{
            // tell the parent we're OK
            return(CSSTAT_OK);
		}
	}
