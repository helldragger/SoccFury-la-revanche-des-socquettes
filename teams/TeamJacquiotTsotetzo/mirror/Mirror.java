package TeamJacquiotTsotetzo.mirror;

import EDU.gatech.cc.is.util.Vec2;

/**
 * Mirror
 */
public interface Mirror {

    public abstract Vec2 getMirrorPosition(long timestamp);
    
}