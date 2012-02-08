#pragma once

#include <deltarobotnode/motionSrv.h>
#include <vision/CrateEventMsg.h>
#include <huniplacer_3d/TopViewMsg.h>

void moveCallback(const deltarobotnode::motions::ConstPtr& msg);
void crateCallback(const vision::CrateEventMsg::ConstPtr& msg);
void positionCallback(const deltarobotnode::motions::ConstPtr& msg);
void topviewCallback(const huniplacer_3d::TopViewMsg::ConstPtr& msg);
