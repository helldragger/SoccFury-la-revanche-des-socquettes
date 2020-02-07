package TeamJacquiotTsotetzo.swarm;

import EDU.gatech.cc.is.abstractrobot.KinSensor;
import EDU.gatech.cc.is.util.Vec2;
import java.lang.Math;

public interface Swarm extends KinSensor, DirSensor{

    double COHESION_FACTOR = 1;
    double SEPARATION_MAX_DISTANCE = 10*0.06;
    double SEPARATION_FACTOR = 0.35;
    double ALIGNMENT_FACTOR = 0.2;
    double CLOSE_THRESHOLD = 20;

    abstract Vec2 getPos();

    //public default void recalculateThreshold

    public default boolean closeEnough(long timestamp, Vec2 teammate){
        Vec2 pos = getPos();
        pos.sub(teammate);
        double dist = Math.sqrt(Math.pow(pos.x,2) + Math.pow(pos.y,2));
        return dist <= CLOSE_THRESHOLD;
    }

    public default Vec2 getMeanPos(long timestamp) {
        Vec2 meanPos = new Vec2(0, 0);
        int n = 0;
        for (Vec2 teammate : getTeammates(timestamp)) {
            if (closeEnough(timestamp, teammate)){
                meanPos.add(teammate);
                n++;
            } 
        }
        if(n == 0) return new Vec2(0,0);
        return new Vec2(meanPos.x / n, meanPos.y / n);
    }

    public default Vec2 cohesion(long timestamp) {
        Vec2 pos = getMeanPos(timestamp);
        pos.sub(getPos());
        return new Vec2(pos.x*COHESION_FACTOR, pos.y*COHESION_FACTOR);
    }

    public default Vec2 separation(long timestamp){
        Vec2 pos = new Vec2(0,0);
        for (Vec2 teammate : getTeammates(timestamp)) {
            if (closeEnough(timestamp, teammate)){
                teammate.sub(getPos());

                teammate = new Vec2(Math.max(-SEPARATION_MAX_DISTANCE, Math.min(teammate.x*SEPARATION_FACTOR, SEPARATION_MAX_DISTANCE)), 
                                    Math.max(-SEPARATION_MAX_DISTANCE, Math.min(teammate.y*SEPARATION_FACTOR, SEPARATION_MAX_DISTANCE)));
                pos.add(teammate);
            }
        }
        return new Vec2(pos.x*-1, pos.y*-1);
    }

    public default Vec2 alignment(long timestamp){
        Vec2 pos = getDirection();
        pos.sub(getTeammatesMeanDirection());
        return new Vec2(pos.x*ALIGNMENT_FACTOR, pos.y*ALIGNMENT_FACTOR);
    }

    public default Vec2 applySwarmBehaviour(long timestamp){
        Vec2 dir = getDirection();
        try {
            dir.add(cohesion(timestamp));
            dir.add(separation(timestamp));
            dir.add(alignment(timestamp));
            return dir;
        } catch (java.lang.Exception e) {
            // Normal, c est l initialisation, il n y a pas de teammates a moyenner.\
            System.err.println(e);
            return new Vec2(0,0);
        }
    };
}

