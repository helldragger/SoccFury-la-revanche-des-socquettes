package TeamJacquiotTsotetzo.subsomption;

import EDU.gatech.cc.is.abstractrobot.SocSmall;
import EDU.gatech.cc.is.util.Vec2;

public interface SubsomptionLayer {

	boolean isActivated(SocSmall abstract_robot);

	Vec2 action(SocSmall abstract_robot);

}
