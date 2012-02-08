#include <vision/CrateTracker.h>

#include <huniplacer_3d/huniplacer_3d.h>
#include <huniplacer_3d/Callbacks.h>

std::map<std::string, CrateModel*> crates;

void moveCallback(const deltarobotnode::motions::ConstPtr& msg) {
	getHuniplacerData()->modeldata.robot->setPosition(msg->x[0], msg->y[0], msg->z[0]);
	ROS_INFO("Move To: x=%f y=%f z=%f", msg->x[0], msg->y[0], msg->z[0]);
}

void crateCallback(const vision::CrateEventMsg::ConstPtr& msg) {
	ROS_INFO("MSG RECV: %i %s", msg->event, msg->crate.name.c_str());
	CrateModel* this_crate =
			crates.find(msg->crate.name) != crates.end() ?
					crates.find(msg->crate.name)->second : NULL;
	if (this_crate == NULL) {
		ROS_INFO(
				"Crate %s not present. Create new one", msg->crate.name.c_str());
		this_crate = new CrateModel(getHuniplacerData()->modeldata.robot, point3(50, 10, 50));
		crates.insert(
				std::pair<std::string, CrateModel*>(msg->crate.name,
						this_crate));
		point3 p(msg->crate.x, GROUND_PANE, msg->crate.y);
		this_crate->setLocation(p);
		this_crate->setRotation(msg->crate.angle);
		this_crate->setAlpha(1.0);
	}

	switch (msg->event) {
	case CrateEvent::type_in:
		//Already done above
		break;
	case CrateEvent::type_out:
		ROS_INFO("Crate %s deleted", msg->crate.name.c_str());
		crates.erase(msg->crate.name);
		delete this_crate;
		break;
	case CrateEvent::type_moving: {
		ROS_INFO("Crate %s moving", msg->crate.name.c_str());
		point3 p(msg->crate.x, GROUND_PANE, msg->crate.y);
		this_crate->setAlpha(0.3);
	}
		break;
	case CrateEvent::type_moved: {
		ROS_INFO("Crate %s s", msg->crate.name.c_str());
		point3 p(msg->crate.x, GROUND_PANE, msg->crate.y);
		this_crate->setLocation(p);
		this_crate->setRotation(msg->crate.angle);
		this_crate->setAlpha(1);
	}
		break;
	}
}

void positionCallback(const deltarobotnode::motions::ConstPtr& msg){
	getHuniplacerData()->modeldata.robot->setNewAnimation(msg.get()->x, msg.get()->y, msg.get()->z,msg.get()->speed);
}

void topviewCallback(const huniplacer_3d::TopViewMsg::ConstPtr& msg){
	CrateModel* this_crate =
				crates.find(msg->name) != crates.end() ?
						crates.find(msg->name)->second : NULL;

	if(this_crate != NULL){
		bool* balletjes = new bool [msg->rows * msg->cols];
		for(int i1 = 0; i1 < msg->rows; i1++){
			for(int i2 = 0; i2 < msg->cols; i2++){
				balletjes[i1 + i2*msg->rows] = (msg->data[i1 + i2*msg->rows] != 0);
			}
		}
		this_crate->balletjes = balletjes;
		this_crate->setNrBallsWidth(msg->rows);
		this_crate->setNrBallsDepth(msg->cols);
	}

}
