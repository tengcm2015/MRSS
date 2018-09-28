//
//  SwingTester.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/28.
//

#ifndef SwingTester_hpp
#define SwingTester_hpp

#include "BaseObject.hpp"

#include "Swing.hpp"
#include "Mannequin.hpp"

enum class TESTER_POSE {
	STAND, CROUCH
};

class SwingTester : BaseObject {
public:
	SwingTester(double groundX, double groundY);

	~SwingTester();

	void addToPxScene(PxScene *scene) override;
	void removeFromPxScene(PxScene *scene) override;
	void draw() override;

	bool isLocked() { return m_mannequin->isLocked(); }
	void unlock() { m_mannequin->unlock(); }

	bool driveIsOn() { return m_mannequin->driveIsOn(); }
	void setDriveOn(bool driveOn);

	TESTER_POSE getPose() { return m_pose; }
	void setPose(TESTER_POSE pose);
	void autoPose();
	void resetAutoState();

	float getSwingAngle() { return m_swing->getAngle(); }

private:
	// for auto swing
	float m_max_angle;
	float m_min_angle;

	TESTER_POSE m_pose;
	std::unique_ptr<Swing> m_swing;
	std::unique_ptr<Mannequin> m_mannequin;
};

#endif /* SwingTester_hpp */
