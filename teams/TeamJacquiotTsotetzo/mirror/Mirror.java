package TeamJacquiotTsotetzo.mirror;


import EDU.gatech.cc.is.abstractrobot.SocSmall;
import EDU.gatech.cc.is.util.CircularBufferEnumeration;
import EDU.gatech.cc.is.util.Vec2;
import TeamJacquiotTsotetzo.communication.IndexationMessage;

/**
 * Mirror
 */
public interface Mirror {
    final int UNASSIGNED = -1;

    /**
     * this array assigns an opponent ID to follow to each player among their list of opponents.
     * The order of opponents when using getOpponents() is fixed but different for each player.
     * This index allows to assign a different specific opponent target to each player.
     */
    int[] focusedIndexes = new int[] { -1, -1, -1, -1, -1 };

    /**
     * This array contains the last position of the focused opponent according to each player.
     * This index allows extrapoling opponent direction.
     */
    Vec2[] lastOpponentPos = new Vec2[]{new Vec2(0,0),new Vec2(0,0),new Vec2(0,0),new Vec2(0,0),new Vec2(0,0)};

    public default void updateOpponentDirection(final SocSmall abstract_robot){
        final long timestamp = abstract_robot.getTime();
        if (abstract_robot.getOpponents(timestamp).length == 0 || getFocusIndex(abstract_robot) == UNASSIGNED) return;
        lastOpponentPos[abstract_robot.getPlayerNumber(timestamp)] = abstract_robot.getOpponents(timestamp)[getFocusIndex(abstract_robot)];
    }

    public default Vec2 getFocusedOpponentLastPosition(final SocSmall abstract_robot){
        return lastOpponentPos[getFocusIndex(abstract_robot)];
    }


    public default Vec2 getFocusedOpponent(final SocSmall abstract_robot){
        return abstract_robot.getOpponents(abstract_robot.getTime())[getFocusIndex(abstract_robot)];
    }

    public default Vec2 getFocusedOpponentDirection(final SocSmall abstract_robot){

        final Vec2 opponent = getFocusedOpponent(abstract_robot);
        opponent.sub(getFocusedOpponentLastPosition(abstract_robot));
        return opponent;
    }

    public default void setFocusIndex(final SocSmall abstract_robot, final int index) {
        focusedIndexes[abstract_robot.getPlayerNumber(abstract_robot.getTime())] = index;
    }

    public default int getFocusIndex(final SocSmall abstract_robot) {
        return focusedIndexes[abstract_robot.getPlayerNumber(abstract_robot.getTime())];
    }

    /**
     * This strategy is made in two phases:
     * 1. Enemy focus assignment.
     * 2. Focused enemy mirroring.
     * 
     * The first phase uses communication, the first player sends it's own point of view of opponents, it's own position and assigns one of his teammates to each opponent.
     * The other players then wait for the message. Upon receiving it, they detect which opponent they were assigned to by calculating the closest one they see from the one seen by the broadcaster.
     * 
     * Then the x axis is mirrored to mimic it's opponent, within a safe distance from the goals.
     * The y axis is mirrored using an approximation of their opponent direction.
     * @param abstract_robot
     * @return the position to target to mirror the enemy
     */
    public default Vec2 getMirrorPosition(final SocSmall abstract_robot) {
        final long timestamp = abstract_robot.getTime();
        //PHASE 1
        if (abstract_robot.getOpponents(timestamp).length == 0) {
            return new Vec2(0, 0);
        }
        if (getFocusIndex(abstract_robot) == UNASSIGNED) { 
            if (abstract_robot.getPlayerNumber(timestamp) == 0) {// broadcaster
                abstract_robot.broadcast(new IndexationMessage(abstract_robot.getOpponents(timestamp),
                        abstract_robot.getPosition(timestamp)));
                setFocusIndex(abstract_robot, 0);
            } else {
                final CircularBufferEnumeration receiver = abstract_robot.getReceiveChannel();
				if(!receiver.hasMoreElements()){
					return new Vec2(0,0);
				}
				final IndexationMessage m = (IndexationMessage) receiver.nextElement();
                final Vec2 focusedVec = m.getOpponent(abstract_robot.getPlayerNumber(timestamp));
                final Vec2 pos = abstract_robot.getPosition(timestamp);
                final Vec2 playerToBroadcaster = m.getBroadcaster();
                playerToBroadcaster.sub(abstract_robot.getPosition(timestamp));
                pos.add(playerToBroadcaster);

                final Vec2[] opponents = abstract_robot.getOpponents(timestamp);
                double minimal_diff = 999;
                int minimal_opponent = UNASSIGNED;
                for (int i = 0; i < opponents.length; i++) {
                    final Vec2 oppVec = opponents[i];
                    oppVec.sub(playerToBroadcaster);
                    oppVec.sub(focusedVec);
                    final double diff = Math.abs(oppVec.x + oppVec.y);
                    if (diff <= minimal_diff) {
                        minimal_opponent = i;
                        minimal_diff = diff;
                    }
                    
                }
                setFocusIndex(abstract_robot, minimal_opponent);

                System.out.println("PLAYER " + abstract_robot.getPlayerNumber(timestamp) + " WILL MIRROR OPPONENT "
                        + getFocusIndex(abstract_robot) + " (diff=" + minimal_diff + ")");
            }
        }
        // PHASE 2
        final Vec2 opponent = getFocusedOpponent(abstract_robot);
        final Vec2 pos = abstract_robot.getPosition(timestamp);

        final double MIN_DIFF_X = 5 * SocSmall.RADIUS;
        final double CAGES = pos.x + abstract_robot.getOurGoal(timestamp).x + 4 * SocSmall.RADIUS;
        final Vec2 targetX = new Vec2(Math.max(opponent.x - MIN_DIFF_X, CAGES - pos.x), 0);

        final Vec2 targetY = new Vec2(0, opponent.y);
        // now we calculate the angle we want to aim, we must try to block the players when they move diagonally
        targetY.add(new Vec2( 0 , -getFocusedOpponentDirection(abstract_robot).y));

		targetX.add(targetY);
		targetX.normalize(1);
		return targetX;
	};
    
}