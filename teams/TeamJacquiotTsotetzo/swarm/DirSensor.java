package TeamJacquiotTsotetzo.swarm;

import EDU.gatech.cc.is.abstractrobot.SocSmall;
import EDU.gatech.cc.is.util.Vec2;

public interface DirSensor {
    /**
     * 0 contains the player' current last position, the other one are his teammates'.
     */
	Vec2[] lastTeamPos = new Vec2[5];

    double CLOSE_THRESHOLD = 20;

    public default void updateTeamDirections(SocSmall abstract_robot){
        final long timestamp = abstract_robot.getTime();

		// update the last positions only if the values are not identical to keep track
        // of the last angle
        if (getLastPosition(abstract_robot) != abstract_robot.getPosition(timestamp)) lastTeamPos[0] = abstract_robot.getPosition(timestamp);
        final Vec2[] teammates = abstract_robot.getTeammates(timestamp);
        for (int i = 0; i < teammates.length; i++) {
            if (teammates[i] != lastTeamPos[i+1]) {
                lastTeamPos[i+1] = teammates[i];
            }
        }
    }

    public default Vec2 getLastPosition(SocSmall abstract_robot){
        return lastTeamPos[0];
    }

	public default Vec2 getDirection(SocSmall abstract_robot) {
		final Vec2 res = new Vec2(abstract_robot.getPosition(abstract_robot.getTime()));
		res.sub(getLastPosition(abstract_robot));
		return res;
	}

    public default boolean closeEnough(SocSmall abstract_robot, Vec2 teammate){
        Vec2 pos = abstract_robot.getPosition(abstract_robot.getTime());
        pos.sub(teammate);
        double dist = Math.sqrt(Math.pow(pos.x,2) + Math.pow(pos.y,2));
        return dist <= CLOSE_THRESHOLD;
    }

	public default Vec2 getTeammatesMeanDirection(SocSmall abstract_robot) {
		final Vec2[] teammates = abstract_robot.getTeammates(abstract_robot.getTime());
		final Vec2 mean = new Vec2(0, 0);
		int n = teammates.length;
		for (int i = 0; i < teammates.length; i++) {
			final Vec2 teammate = new Vec2(teammates[i]);
			if (closeEnough(abstract_robot, teammate)) {
				teammate.sub(lastTeamPos[i]);
				mean.add(teammate);
				n++;
			}
		}
		if (n == 0)
			return new Vec2(0, 0);
		return new Vec2(mean.x / n, mean.y / n);
	}

}
