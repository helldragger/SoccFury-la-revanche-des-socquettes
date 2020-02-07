package TeamJacquiotTsotetzo.swarm;

import EDU.gatech.cc.is.abstractrobot.SocSmall;
import EDU.gatech.cc.is.util.Vec2;
import java.lang.Math;

public interface Swarm extends DirSensor{

    double COHESION_FACTOR = 1;
    double SEPARATION_MAX_DISTANCE = 10*0.06;
    double SEPARATION_FACTOR = 0.35;
    double ALIGNMENT_FACTOR = 0.05;

    //public default void recalculateThreshold


    public default Vec2 getMeanPos(SocSmall abstract_robot) {
        final long timestamp = abstract_robot.getTime();
        Vec2 meanPos = new Vec2(0, 0);
        int n = 0;
        for (Vec2 teammate : abstract_robot.getTeammates(timestamp)) {
            if (closeEnough(abstract_robot, teammate)){
                meanPos.add(teammate);
                n++;
            } 
        }
        if(n == 0) return new Vec2(0,0);
        return new Vec2(meanPos.x / n, meanPos.y / n);
    }

    public default Vec2 cohesion(SocSmall abstract_robot) {
        Vec2 pos = getMeanPos(abstract_robot);
        pos.sub(abstract_robot.getPosition(abstract_robot.getTime()));
        return new Vec2(pos.x*COHESION_FACTOR, pos.y*COHESION_FACTOR);
    }

    public default Vec2 separation(SocSmall abstract_robot){
        Vec2 pos = new Vec2(0,0);
        for (Vec2 teammate : abstract_robot.getTeammates(abstract_robot.getTime())) {
            if (closeEnough(abstract_robot, teammate)){
                teammate.sub(abstract_robot.getPosition(abstract_robot.getTime()));

                teammate = new Vec2(Math.max(-SEPARATION_MAX_DISTANCE, Math.min(teammate.x*SEPARATION_FACTOR, SEPARATION_MAX_DISTANCE)), 
                                    Math.max(-SEPARATION_MAX_DISTANCE, Math.min(teammate.y*SEPARATION_FACTOR, SEPARATION_MAX_DISTANCE)));
                pos.add(teammate);
            }
        }
        return new Vec2(pos.x*-1, pos.y*-1);
    }

    public default Vec2 alignment(SocSmall abstract_robot){
        Vec2 pos = getDirection(abstract_robot);
        pos.sub(getTeammatesMeanDirection(abstract_robot));
        return new Vec2(pos.x*ALIGNMENT_FACTOR, pos.y*ALIGNMENT_FACTOR);
    }

    public default Vec2 applySwarmBehaviour(SocSmall abstract_robot){
        Vec2 dir = getDirection(abstract_robot);
        try {
            dir.add(cohesion(abstract_robot));
            dir.add(separation(abstract_robot));
            dir.add(alignment(abstract_robot));
            return dir;
        } catch (java.lang.Exception e) {
            // Normal, c est l initialisation, il n y a pas de teammates a moyenner.\
            System.err.println(e);
            return new Vec2(0,0);
        }
    };
}

