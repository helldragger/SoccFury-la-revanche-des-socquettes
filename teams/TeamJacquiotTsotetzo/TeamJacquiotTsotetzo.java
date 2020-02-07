package TeamJacquiotTsotetzo;

/*
 * LeftTeam.java
 */

import EDU.gatech.cc.is.util.Vec2;
import TeamJacquiotTsotetzo.fields.FieldSensor;
import TeamJacquiotTsotetzo.mirror.Mirror;
import TeamJacquiotTsotetzo.swarm.DirSensor;
import TeamJacquiotTsotetzo.swarm.Swarm;
import EDU.gatech.cc.is.abstractrobot.*;
import TeamJacquiotTsotetzo.communication.IndexationMessage;

import java.util.Enumeration;
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


public class TeamJacquiotTsotetzo extends ControlSystemSS implements DirSensor, Swarm, FieldSensor, Mirror
	{
	int focusedIndex = -1;
	Vec2 lastFocusedOpponent;
	Vec2 lastPos ;
	Vec2[] lastTeammatesPos;
	
	@Override
	public Vec2 getDirection() {
		final Vec2 res = new Vec2(abstract_robot.getPosition(abstract_robot.getTime()));
		res.sub(this.lastPos);
		return res;
	}

	@Override
	public Vec2 getPos() {
		return this.abstract_robot.getPosition(abstract_robot.getTime());
	}

	@Override
	public Vec2 getTeammatesMeanDirection() {
		final Vec2[] teammates = abstract_robot.getTeammates(abstract_robot.getTime());
		final Vec2 mean = new Vec2(0, 0);
		int n = teammates.length;
		for (int i = 0; i < teammates.length; i++) {
			final Vec2 teammate = new Vec2(teammates[i]);
			if (closeEnough(this.abstract_robot.getTime(), teammate)) {
				teammate.sub(lastTeammatesPos[i]);
				mean.add(teammate);
				n++;
			}
		}
		if (n == 0)
			return new Vec2(0, 0);
		return new Vec2(mean.x / n, mean.y / n);
	}

	/**
	 * Configure the control system. This method is called once at initialization
	 * time. You can use it to do whatever you like.
	 */
	public void Configure() {

		this.lastPos = this.abstract_robot.getPosition(this.abstract_robot.getTime());
		this.lastTeammatesPos = this.abstract_robot.getTeammates(this.abstract_robot.getTime());
		// this.lastFocusedOpponent =
		// abstract_robot.getOpponents(this.abstract_robot.getTime())[this.abstract_robot.getID()];
	}

	/**
	 * Called every timestep to allow the control system to run.
	 */
	public int TakeStep() {
		final long timestamp = abstract_robot.getTime();
		if (lastTeammatesPos.length == 0)
			lastTeammatesPos = abstract_robot.getTeammates(timestamp);
		final boolean MIRROR = true;
		if (MIRROR) {
			Vec2 move = getMirrorPosition(timestamp);
			this.abstract_robot.setSteerHeading(timestamp, move.t);
			this.abstract_robot.setSpeed(timestamp, move.r);
			if (abstract_robot.canKick(timestamp) && abstract_robot.getBall(timestamp).x>0.1 ) abstract_robot.kick(timestamp);
			return (CSSTAT_OK);
		}
		final Vec2 dir = this.applySwarmBehaviour(timestamp);
		final Vec2 ball = abstract_robot.getBall(timestamp);
		final Vec2 pos = abstract_robot.getPosition(timestamp);
		pos.add(dir);
		// dir.normalize(1);
		final double SWARM_FACTOR = 1;
		dir.normalize(dir.r * SWARM_FACTOR);
		final double BALL_FIELD_FACTOR = 0.1;
		final double OPPONENT_FIELD_FACTOR = 0;
		final double ADVERSE_GOAL_FIELD_FACTOR = 0;
		final double KICKSPOT_FIELD_FACTOR = 1;

		final Vec2 bf = getBallField(timestamp);
		bf.normalize(bf.r * BALL_FIELD_FACTOR);
		dir.add(bf);

		final Vec2 of = getOpponentField(timestamp);
		of.normalize(of.r * OPPONENT_FIELD_FACTOR);
		dir.add(of);

		final Vec2 agf = getAdverseGoalField(timestamp);
		agf.normalize(agf.r * ADVERSE_GOAL_FIELD_FACTOR);
		dir.add(agf);

		final Vec2 ksf = getKickSpotField(timestamp);
		ksf.normalize(ksf.r * KICKSPOT_FIELD_FACTOR);
		dir.add(ksf);

		// dir.add(kickspot);
		dir.normalize(1);
		this.abstract_robot.setSteerHeading(timestamp, dir.t);
		this.abstract_robot.setSpeed(timestamp, dir.r);

		if (this.abstract_robot.canKick(timestamp))
			this.abstract_robot.kick(timestamp);

		// update the last positions only if the values are not identical to keep track
		// of the last angle
		if (lastPos != abstract_robot.getPosition(timestamp))
			lastPos = abstract_robot.getPosition(timestamp);
		final Vec2[] teammates = abstract_robot.getTeammates(timestamp);
		for (int i = 0; i < teammates.length; i++) {
			if (teammates[i] != lastTeammatesPos[i]) {
				lastTeammatesPos[i] = teammates[i];
			}
		}

		return (CSSTAT_OK);

	}

	@Override
	public Vec2[] getTeammates(final long timestamp) {
		return abstract_robot.getTeammates(timestamp);
	}

	@Override
	public Vec2[] getOpponents(final long timestamp) {
		return abstract_robot.getOpponents(timestamp);
	}

	@Override
	public int getPlayerNumber(final long timestamp) {
		return abstract_robot.getPlayerNumber(timestamp);
	}

	@Override
	public void setKinMaxRange(final double r) {
		abstract_robot.setKinMaxRange(r);
	}

	@Override
	public Vec2 getBallField(final long timestamp) {
		return abstract_robot.getBall(timestamp);
	}

	@Override
	public Vec2 getAdverseGoalField(final long timestamp) {
		return abstract_robot.getOpponentsGoal(timestamp);
	}

	@Override
	public Vec2 getKickSpotField(final long timestamp) {
		final Vec2 ball = getBallField(timestamp);
		final Vec2 adverse = getAdverseGoalField(timestamp);
		// Here we get the ball to goal vector
		adverse.sub(ball);
		final Vec2 kickspot = new Vec2(0, 0);
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
	public Vec2 getOpponentField(final long timestamp) {
		final Vec2 field = new Vec2(0, 0);
		for (final Vec2 opponent : abstract_robot.getOpponents(timestamp)) {
			field.sub(new Vec2(opponent.x / opponent.r, opponent.y / opponent.r));
		}
		return field;
	}

	@Override
	public Vec2 getMirrorPosition(final long timestamp) {
		if (abstract_robot.getOpponents(timestamp).length == 0) {return new Vec2(0,0);}
		if (focusedIndex == -1){
			if( abstract_robot.getPlayerNumber(timestamp) == 0){// broadcaster
				abstract_robot.broadcast(new IndexationMessage(abstract_robot.getOpponents(timestamp), this.abstract_robot.getPosition(timestamp)));
				focusedIndex = 0;
			}
			else{
				final Enumeration receiver = abstract_robot.getReceiveChannel();
				if(!receiver.hasMoreElements()){
					return new Vec2(0,0);
				}
				IndexationMessage m = (IndexationMessage) receiver.nextElement();
				Vec2 focusedVec = m.getOpponent(abstract_robot.getPlayerNumber(timestamp));
				Vec2 pos =  abstract_robot.getPosition(timestamp);
				Vec2 playerToBroadcaster = m.getBroadcaster();
				playerToBroadcaster.sub(abstract_robot.getPosition(timestamp));
				pos.add(playerToBroadcaster);

				Vec2[] opponents = abstract_robot.getOpponents(timestamp);
				double minimal_diff = 999;
				int minimal_opponent= -1;
				for (int i = 0; i < opponents.length; i++) {
					Vec2 oppVec = opponents[i];
					oppVec.sub(playerToBroadcaster);
					oppVec.sub(focusedVec);
					double diff = Math.abs(oppVec.x + oppVec.y);
					if(diff <= minimal_diff){
						minimal_opponent = i;
						minimal_diff = diff;
					};
				}
				focusedIndex = minimal_opponent;

				System.out.println("PLAYER "+abstract_robot.getPlayerNumber(timestamp)+" WILL MIRROR OPPONENT "+focusedIndex+" (diff="+minimal_diff+")");
			}
		}
		Vec2 opponent = abstract_robot.getOpponents(timestamp)[focusedIndex];
		Vec2 pos = abstract_robot.getPosition(timestamp);

		Vec2 targetY = new Vec2(0, opponent.y);
		double MIN_DIFF_X = 5*abstract_robot.RADIUS;
		double CAGES = pos.x + abstract_robot.getOurGoal(timestamp).x + 4*abstract_robot.RADIUS;
		Vec2 targetX = new Vec2(Math.max(opponent.x-MIN_DIFF_X, CAGES - pos.x),0);
		
		targetX.add(targetY);
		targetX.normalize(1);
		return targetX;
	}
}

