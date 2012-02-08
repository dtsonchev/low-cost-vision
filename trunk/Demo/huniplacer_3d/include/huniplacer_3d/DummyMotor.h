#pragma once

class DummyMotor: public huniplacer::imotor3 {
	virtual void moveto(const huniplacer::motionf& mf, bool async) {
	}

	virtual void moveto_within(const huniplacer::motionf& mf, double time,
			bool async) {
	}

	virtual double get_min_angle(void) const {
		return huniplacer::measures::MOTOR_ROT_MIN;
	}

	virtual double get_max_angle(void) const {
		return huniplacer::measures::MOTOR_ROT_MAX;
	}

	virtual void stop(void) {
	}

	virtual bool wait_for_idle(long timeout) {
		return false;
	}

	virtual bool is_idle(void) {
		return false;
	}

	virtual void power_off(void) {
	}

	virtual void power_on(void) {
	}

	virtual bool is_powerd_on(void) {
		return true;
	}

	virtual void override_current_angles(double * angles) {
	}
};
