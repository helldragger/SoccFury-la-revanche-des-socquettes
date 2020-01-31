/*
 * LeftTeam.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import fields.FieldSensor;
import swarm.DirSensor;
import swarm.Swarm;
import	EDU.gatech.cc.is.abstractrobot.*;
import java.lang.Math.*;
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


public class LeftTeam extends ControlSystemSS implements DirSensor, Swarm, FieldSensor
	{
 
	Vec2 lastPos ;
	Vec2[] lastTeammatesPos;
	
	@Override
	public Vec2 getDirection() {
		Vec2 res = new Vec2(abstract_robot.getPosition(abstract_robot.getTime()));
		res.sub(this.lastPos);
		return res;
	}

	@Override
	public Vec2 getPos() {
		return this.abstract_robot.getPosition(abstract_robot.getTime());
	}

	@Override
	public Vec2 getTeammatesMeanDirection() {
		Vec2[] teammates = abstract_robot.getTeammates(abstract_robot.getTime());
		Vec2 mean = new Vec2(0,0);
		int n = teammates.length;
		for (int i = 0; i < teammates.length; i++) {
			Vec2 teammate = new Vec2(teammates[i]);
			if (closeEnough(this.abstract_robot.getTime(), teammate)){
				teammate.sub(lastTeammatesPos[i]);
				mean.add(teammate);
				n++;				
			}
		}
		if (n==0) return new Vec2(0,0);
		return new Vec2(mean.x/n,mean.y/n);
	}
	
	
	/**
	Configure the control system.  This method is
	called once at initialization time.  You can use it
	to do whatever you like.
	*/
	public void Configure()
    {
	
		this.lastPos = this.abstract_robot.getPosition(this.abstract_robot.getTime());
		this.lastTeammatesPos = this.abstract_robot.getTeammates(this.abstract_robot.getTime());

    }
    
	/**
	Called every timestep to allow the control system to
	run.
    */
	public int TakeStep(){
		
		long timestamp = abstract_robot.getTime();
		if (lastTeammatesPos.length == 0) lastTeammatesPos = abstract_robot.getTeammates(timestamp);
	
		Vec2 dir = this.applySwarmBehaviour(timestamp);
		Vec2 ball = abstract_robot.getBall(timestamp);
		Vec2 pos = abstract_robot.getPosition(timestamp);
		pos.add(dir);
		//dir.normalize(1);
		double SWARM_FACTOR = 1;
		dir.normalize(dir.r*SWARM_FACTOR);
		double BALL_FIELD_FACTOR = 0.1;
		double OPPONENT_FIELD_FACTOR = 0;
		double ADVERSE_GOAL_FIELD_FACTOR = 0;
		double KICKSPOT_FIELD_FACTOR = 1;
		
		Vec2 bf = getBallField(timestamp); 
		bf.normalize(bf.r*BALL_FIELD_FACTOR);
		dir.add(bf);
		
		Vec2 of = getOpponentField(timestamp); 
		of.normalize(of.r*OPPONENT_FIELD_FACTOR);
		dir.add(of);
		
		Vec2 agf = getAdverseGoalField(timestamp); 
		agf.normalize(agf.r*ADVERSE_GOAL_FIELD_FACTOR);
		dir.add(agf);

		Vec2 ksf = getKickSpotField(timestamp); 
		ksf.normalize(ksf.r*KICKSPOT_FIELD_FACTOR);
		dir.add(ksf);
		
		//dir.add(kickspot);
		dir.normalize(1);
		this.abstract_robot.setSteerHeading(timestamp, dir.t);
		this.abstract_robot.setSpeed(timestamp, dir.r);

		if(this.abstract_robot.canKick(timestamp)) this.abstract_robot.kick(timestamp);

		// update the last positions only if the values are not identical to keep track of the last angle
		if (lastPos != abstract_robot.getPosition(timestamp)) lastPos = abstract_robot.getPosition(timestamp);
		Vec2[] teammates = abstract_robot.getTeammates(timestamp);
		for (int i = 0; i < teammates.length; i++) {
			if (teammates[i] != lastTeammatesPos[i]) {
				lastTeammatesPos[i] = teammates[i];
			}
		}

		return (CSSTAT_OK);    
		
	}

	@Override
	public Vec2[] getTeammates(long timestamp) {
		return abstract_robot.getTeammates(timestamp);
	}

	@Override
	public Vec2[] getOpponents(long timestamp) {
		return abstract_robot.getOpponents(timestamp);
	}

	@Override
	public int getPlayerNumber(long timestamp) {
		return abstract_robot.getPlayerNumber(timestamp);
	}

	@Override
	public void setKinMaxRange(double r) {
		abstract_robot.setKinMaxRange(r);
	}

	@Override
	public Vec2 getBallField(long timestamp) {
		return abstract_robot.getBall(timestamp);
	}

	@Override
	public Vec2 getAdverseGoalField(long timestamp) {
		return abstract_robot.getOpponentsGoal(timestamp);
	}

	@Override
	public Vec2 getKickSpotField(long timestamp) {
		Vec2 ball = getBallField(timestamp);
		Vec2 adverse = getAdverseGoalField(timestamp);
		// Here we get the ball to goal vector
		adverse.sub(ball);
		Vec2 kickspot = new Vec2(0,0);
		// we invert its direction
		kickspot.sub(adverse);
		// we normalize it the actual kicking distance 
		kickspot.normalize(abstract_robot.RADIUS);
		// we calculate the actual kicking spot relative to the ball
		ball.add(kickspot);
		// we have our field.
		return ball;
	}

	@Override
	public Vec2 getOpponentField(long timestamp) {
		Vec2 field = new Vec2(0,0);
		for (Vec2 opponent : abstract_robot.getOpponents(timestamp)){
			field.sub(new Vec2(opponent.x / opponent.r, opponent.y / opponent.r));
		}
		return field;
	}
}
