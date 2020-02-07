package TeamJacquiotTsotetzo.fields;

import EDU.gatech.cc.is.util.Vec2;

/**
 * FieldSensor
 */
public interface FieldSensor {

    public abstract Vec2 getBallField(long timestamp);
    public abstract Vec2 getAdverseGoalField(long timestamp);
    public abstract Vec2 getKickSpotField(long timestamp);
    public abstract Vec2 getOpponentField(long timestamp);
    
}