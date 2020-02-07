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
	
	/**
	 * Configure the control system. This method is called once at initialization
	 * time. You can use it to do whatever you like.
	 */
	public void Configure() {
		updateTeamDirections(abstract_robot);
		updateOpponentDirection(abstract_robot);
		// this.lastFocusedOpponent =
		// abstract_robot.getOpponents(this.abstract_robot.getTime())[this.abstract_robot.getID()];
	}

	/**
	 * Called every timestep to allow the control system to run.
	 */
	public int TakeStep() {
		final long timestamp = abstract_robot.getTime();
		
		updateTeamDirections(abstract_robot);

		final boolean MIRROR = true;
		if (MIRROR) {
			Vec2 move = getMirrorPosition(abstract_robot);
			updateOpponentDirection(abstract_robot);
			this.abstract_robot.setSteerHeading(timestamp, move.t);
			this.abstract_robot.setSpeed(timestamp, move.r);
			if (abstract_robot.canKick(timestamp) && abstract_robot.getBall(timestamp).x>0.1 ) abstract_robot.kick(timestamp);
			return (CSSTAT_OK);
		}
		final Vec2 dir = this.applySwarmBehaviour(abstract_robot);
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

		final Vec2 bf = getBallField(abstract_robot);
		bf.normalize(bf.r * BALL_FIELD_FACTOR);
		dir.add(bf);

		final Vec2 of = getOpponentField(abstract_robot);
		of.normalize(of.r * OPPONENT_FIELD_FACTOR);
		dir.add(of);

		final Vec2 agf = getAdverseGoalField(abstract_robot);
		agf.normalize(agf.r * ADVERSE_GOAL_FIELD_FACTOR);
		dir.add(agf);

		final Vec2 ksf = getKickSpotField(abstract_robot);
		ksf.normalize(ksf.r * KICKSPOT_FIELD_FACTOR);
		dir.add(ksf);

		// dir.add(kickspot);
		dir.normalize(1);
		this.abstract_robot.setSteerHeading(timestamp, dir.t);
		this.abstract_robot.setSpeed(timestamp, dir.r);

		if (this.abstract_robot.canKick(timestamp))
			this.abstract_robot.kick(timestamp);
		updateTeamDirections(abstract_robot);

		return (CSSTAT_OK);

	}

}

