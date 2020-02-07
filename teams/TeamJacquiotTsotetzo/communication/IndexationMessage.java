package TeamJacquiotTsotetzo.communication;
import java.util.HashMap;
import java.util.Map;

import EDU.gatech.cc.is.communication.Message;
import EDU.gatech.cc.is.util.Vec2;

public class IndexationMessage extends Message {

	/**
	 *
	 */
	private static final long serialVersionUID = 1L;

	Map<Integer, Vec2> indexes = new HashMap<>();
	Vec2 origin;

	public IndexationMessage(final Vec2[] opponents, Vec2 origin) {
		for (int i = 1; i < opponents.length; i++) {
			indexes.put(i, opponents[i]);
		}
		this.origin = origin;
	}

	public Vec2 getOpponent(final int id) {
		return indexes.get(id);
	}

	public Vec2 getBroadcaster(){
		return new Vec2(origin);
	}
}