package swarm;

import EDU.gatech.cc.is.util.Vec2;

public interface DirSensor {
    abstract public Vec2 getDirection();
    abstract public Vec2 getTeammatesMeanDirection();

}
