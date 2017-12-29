package edu.berkeley.bid.Bullet;

import java.io.Serializable;

public class RobotSimulatorLoadUrdfFileArgs implements Serializable {
    private static final long serialVersionUID = 525347320574832535L;
    
    Vector3 m_startPosition;
    Quaternion m_startOrientation;
    boolean m_forceOverrideFixedBase;
    boolean m_useMultiBody;
    int m_flags;

    RobotSimulatorLoadUrdfFileArgs(Vector3 startPos, Quaternion startOrn) {
	m_startPosition = startPos.cpy();
	m_startOrientation = startOrn.cpy();
	m_forceOverrideFixedBase = false;
	m_useMultiBody= true;
	m_flags = 0;
    }

    RobotSimulatorLoadUrdfFileArgs() {
	m_startPosition = new Vector3(0, 0, 0);
	m_startOrientation = new Quaternion(0, 0, 0, 1);
	m_forceOverrideFixedBase = false;
	m_useMultiBody = true;
	m_flags = 0;
    }

 }
