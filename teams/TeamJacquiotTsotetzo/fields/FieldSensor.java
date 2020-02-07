package TeamJacquiotTsotetzo.fields;

import EDU.gatech.cc.is.abstractrobot.SocSmall;
import EDU.gatech.cc.is.util.Vec2;

/**
 * FieldSensor
 */
public interface FieldSensor {

	public default Vec2 getBallField(final SocSmall abstract_robot) {
		return abstract_robot.getBall(abstract_robot.getTime());
	};
    public default Vec2 getAdverseGoalField(final SocSmall abstract_robot){
		return abstract_robot.getOpponentsGoal(abstract_robot.getTime());
	};
    public default Vec2 getKickSpotField(final SocSmall abstract_robot){
		final Vec2 ball = getBallField(abstract_robot);
		final Vec2 adverse = getAdverseGoalField(abstract_robot);
		// Here we get the ball to goal vector
		adverse.sub(ball);
		final Vec2 kickspot = new Vec2(0, 0);
		// we invert its direction
		kickspot.sub(adverse);
		// we normalize it the actual kicking distance
		kickspot.normalize(SocSmall.RADIUS);
		// we calculate the actual kicking spot relative to the ball
		ball.add(kickspot);
		// we have our field.
		return ball;
	};
    public default Vec2 getOpponentField(final SocSmall abstract_robot){
		final Vec2 field = new Vec2(0, 0);
		for (final Vec2 opponent : abstract_robot.getOpponents(abstract_robot.getTime())) {
			field.sub(new Vec2(opponent.x / opponent.r, opponent.y / opponent.r));
		}
		return field;
	};
    
}