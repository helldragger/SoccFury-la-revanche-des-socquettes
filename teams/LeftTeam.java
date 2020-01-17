/*
 * LeftTeam.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import  java.lang.Math.*;
import java.util.Random;
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


public class LeftTeam extends ControlSystemSS
	{
 
    Vec2 LastKnownBallPos;
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
     * Move the player toward the ball if he doesn't have it already.
     * 
     */
    private void fetchBall() {
		abstract_robot.setDisplayString("Fetching ball");
        // get vector to the ball		
        long curr_time = abstract_robot.getTime();

        Vec2 ball = abstract_robot.getBall(curr_time);
        
		// set the heading
		abstract_robot.setSteerHeading(curr_time, ball.t);
		// set speed at maximum
		abstract_robot.setSpeed(curr_time, 1.0);
    }

    /**
     * Move the player randomly on the board.
     */
    private void moveRandomly(){		
		abstract_robot.setDisplayString("Moving randomly");

        long curr_time = abstract_robot.getTime();
        Vec2 result = new Vec2(Math.floor(Math.random()*100-50),Math.floor(Math.random()*100-50));
		// set the heading
		abstract_robot.setSteerHeading(curr_time, result.t);
		// set speed at maximum
		abstract_robot.setSpeed(curr_time, Math.random());
    }


    /**
     * Pass the ball to an ally
    // find the ball
    // if the ball is already taken (player withing kicking distance)
    //   we wait
    // if the ball is not and is moving 
    //   we wait
    // else:
    //  if a player has not fetched it, the closest one fetch it\
    //  else he determine check for a random available target (no obstacles between the ball and him, and enough space behind the ball, his team)
    //      if none is found, he targets the barycenter of his team.
    //      else he targets him. 
    //  He shoots.
     */
    private void sendToAlly(){
		abstract_robot.setDisplayString("Sending to Ally");
        int NOONE = -1;
        int PLAYER= -2;

        long curr_time = abstract_robot.getTime();
        // find the ball
		Vec2 ball = abstract_robot.getBall(curr_time);
        
        // if the ball is already taken by a teammate (player withing kicking distance)
        Vec2[] teammates = abstract_robot.getTeammates(curr_time);

        double DIST_THRESHOLD = 0.3*2;
        int takenBy = PLAYER;
        int closestTeammate = PLAYER; // by default, -2 the player has taken the ball
        double closestDist = ball.r;
        if (!(closestDist < DIST_THRESHOLD)){
            closestTeammate = NOONE; // the player doesn have the ball, -1 noone has it
		    for (int i=0; i< teammates.length; i++)
            {
                Vec2 vdist = abstract_robot.getBall(curr_time);
                vdist.sub(teammates[i]);
                double dist  = Math.abs(vdist.x)+Math.abs(vdist.y); 
                if (dist < DIST_THRESHOLD) takenBy = i;
                if (dist < closestDist){
                    closestDist = dist;
                    closestTeammate = i;
                }
            }        
        }
        // if the ball is already taken by a teammate (player withing kicking distance)
        if (takenBy != NOONE && takenBy != PLAYER){
            moveRandomly();
            return;
        }
        //   we wait

        boolean isMoving = (Math.abs(LastKnownBallPos.x - ball.x) + Math.abs(LastKnownBallPos.y - ball.y)) > 0.01;
        //if (isMoving){
            // if the ball is not and is moving 
        //    moveRandomly();
        //    return;
        //}
        //   we wait


        // else:
        //  if a player has not fetched it, the closest one fetch it
        if (takenBy == NOONE){
            if (closestTeammate != PLAYER) return;
            fetchBall();
            return;
        }

        //  else he determine check for a random available target (no obstacles between the ball and him, and enough space behind the ball, his team) ?
        // for now it s a random target
        if(takenBy == PLAYER){
            teammates = abstract_robot.getTeammates(curr_time);

            int target_id = randInt(0, teammates.length); //random between 0 and teammates lenght-1
            if (target_id > teammates.length-1) return;
            Vec2 target = teammates[target_id];
            // compute a point one robot radius
            // behind the ball.
            Vec2 kickspot = new Vec2(ball.x, ball.y);
            kickspot.sub(target);
            kickspot.setr(abstract_robot.RADIUS);
            kickspot.add(ball);

            /*--- Send commands to actuators ---*/
            // set the heading
            abstract_robot.setSteerHeading(curr_time, kickspot.t);

            // set speed at maximum
            abstract_robot.setSpeed(curr_time, 1.0);

            // kick it if we can
            if (abstract_robot.canKick(curr_time))
                abstract_robot.kick(curr_time);
        }else{
            
                moveRandomly();
                return;
        
        }
        //  else he determine check for a random available target (no obstacles between the ball and him, and enough space behind the ball, his team) ?
        //      if none is found, he targets the barycenter of his team.
        //      else he targets him. 
        //  He shoots.
        //TODO
    }

	
	/**
	Called every timestep to allow the control system to
	run.
    */
	public int TakeStep()
		{
            if (LastKnownBallPos == null) LastKnownBallPos = abstract_robot.getBall(abstract_robot.getTime());
            if(true){
                sendToAlly();
                return (CSSTAT_OK);    
            }
		// the eventual movement command is placed here
		Vec2	result = new Vec2(0,0);

		// get the current time for timestamps
		long	curr_time = abstract_robot.getTime();


		/*--- Get some sensor data ---*/
		// get vector to the ball
		Vec2 ball = abstract_robot.getBall(curr_time);

		// get vector to our and their goal
		Vec2 ourgoal = abstract_robot.getOurGoal(curr_time);
		Vec2 theirgoal = abstract_robot.getOpponentsGoal(curr_time);

		// get a list of the positions of our teammates
		Vec2[] teammates = abstract_robot.getTeammates(curr_time);

		/*--- check get opponents routine ---*/
		//Vec2[] opponents = abstract_robot.getOpponents(curr_time);
		//for (int i=0;i<teammates.length;i++)
			//{
			//System.out.println(i+" team "+teammates[i].x+" "+
				//teammates[i].y+" ");
			//System.out.println(i+" opp  "+opponents[i].x+" "+
				//opponents[i].y);
			//}

		// find the closest teammate
		Vec2 closestteammate = new Vec2(99999,0);
		for (int i=0; i< teammates.length; i++)
			{
			if (teammates[i].r < closestteammate.r)
				closestteammate = teammates[i];
			}


		/*--- now compute some strategic places to go ---*/
		// compute a point one robot radius
		// behind the ball.
		Vec2 kickspot = new Vec2(ball.x, ball.y);
		kickspot.sub(theirgoal);
		kickspot.setr(abstract_robot.RADIUS);
		kickspot.add(ball);

		// compute a point three robot radii
		// behind the ball.
		Vec2 backspot = new Vec2(ball.x, ball.y);
		backspot.sub(theirgoal);
		backspot.setr(abstract_robot.RADIUS*5);
		backspot.add(ball);

		// compute a north and south spot
		Vec2 northspot = new Vec2(backspot.x,backspot.y+0.7);
		Vec2 southspot = new Vec2(backspot.x,backspot.y-0.7);

		// compute a position between the ball and defended goal
		Vec2 goaliepos = new Vec2(ourgoal.x + ball.x,
				ourgoal.y + ball.y);
		goaliepos.setr(goaliepos.r*0.5);

		// a direction away from the closest teammate.
		Vec2 awayfromclosest = new Vec2(closestteammate.x,
				closestteammate.y);
		awayfromclosest.sett(awayfromclosest.t + Math.PI);


		/*--- go to one of the places depending on player num ---*/
		int mynum = abstract_robot.getPlayerNumber(curr_time);

		/*--- Goalie ---*/
		if (mynum == 0)
			{
			// go to the goalie position if far from the ball
			if (ball.r > 0.5) 
				result = goaliepos;
			// otherwise go to kick it
			else if (ball.r > 0.1) 
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		/*--- midback ---*/
		else if (mynum == 1)
			{
			// go to a midback position if far from the ball
			if (ball.r > 0.5) 
				result = backspot;
			// otherwise go to kick it
			else if (ball.r > 0.30) 
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		else if (mynum == 2)
			{
			// go to a the northspot position if far from the ball
			if (ball.r > 0.5) 
				result = northspot;
			// otherwise go to kick it
			else if (ball.r > 0.30) 
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		else if (mynum == 4)
			{
			// go to a the northspot position if far from the ball
			if (ball.r > 0.5) 
				result = southspot;
			// otherwise go to kick it
			else if (ball.r > 0.3 )
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		/*---Lead Forward ---*/
		else if (mynum == 3)
			{
			// if we are more than 4cm away from the ball
			if (ball.r > .3)
				// go to a good kicking position
				result = kickspot;
			else
				// go to the ball
				result = ball;
			}


		/*--- Send commands to actuators ---*/
		// set the heading
		abstract_robot.setSteerHeading(curr_time, result.t);

		// set speed at maximum
		abstract_robot.setSpeed(curr_time, 1.0);

		// kick it if we can
		if (abstract_robot.canKick(curr_time))
			abstract_robot.kick(curr_time);

		// tell the parent we're OK
		return(CSSTAT_OK);
        }
        /**
         * Returns a pseudo-random number between min and max, inclusive.
         * The difference between min and max can be at most
         * <code>Integer.MAX_VALUE - 1</code>.
         *
         * @param min Minimum value
         * @param max Maximum value.  Must be greater than min.
         * @return Integer between min and max, inclusive.
         * @see java.util.Random#nextInt(int)
         */
        public static int randInt(int min, int max) {

            // NOTE: This will (intentionally) not run as written so that folks
            // copy-pasting have to think about how to initialize their
            // Random instance.  Initialization of the Random instance is outside
            // the main scope of the question, but some decent options are to have
            // a field that is initialized once and then re-used as needed or to
            // use ThreadLocalRandom (if using at least Java 1.7).
            // 
            // In particular, do NOT do 'Random rand = new Random()' here or you
            // will get not very good / not very random results.
            Random rand = new Random();
            // nextInt is normally exclusive of the top value,
            // so add 1 to make it inclusive
            int randomNum = rand.nextInt((max - min) + 1) + min;

            return randomNum;
        }
	}
