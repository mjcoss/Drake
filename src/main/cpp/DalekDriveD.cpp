#include <DalekDrive.h>

// DalakDriveD provides support for differential drive in either 2 motor or
// 4 motor configuration.  In the 4 motor case, we will put the left and right
// side in follow mode, and only issue commands to the front/master motors.
// The follow mode is a special mode supported by CANTalon and CANSparkMax
// if at some point we need to support other motorcontrollers we will need
// to revist this.  Also this code doesn't support gear shift.

using namespace frc;
using namespace rev;

DalekDrive::DalekDrive(int leftMotorChannel, int rightMotorChannel)
{
	m_leftMotor[kFront]  = new CANSparkMax(leftMotorChannel);
	m_rightMotor[kFront] = new CANSparkMax(rightMotorChannel);
    m_left               = new SpeedControllerGroup(*m_leftMotor[kFront])
    m_right              = new SpeedControllerGroup(*m_rightMotor[kFront])
	m_drive              = new DifferentialDrive(*m_left, *m_right);
	m_needFree           = true;
	InitDalekDrive();

}

DalekDrive::DalekDrive(int leftMotorChannel, int leftSlaveMotorChannel,
         int rightMotorChannel, int rightSlaveMotorChannel,
		 int shiftChannel)
{
	m_leftMotor[kFront]  = new CANSparkMax(leftMotorChannel);
	m_rightMotor[kFront] = new CANSparkMax(rightMotorChannel);
	m_leftMotor[kRear]   = new CANSparkMax(leftSlaveMotorChannel);
	m_rightMotor[kRear]  = new CANSparkMax(rightSlaveMotorChannel);
    m_left               = new SpeedControllerGroup(*m_leftMotor[kFront]);
    m_right              = new SpeedControllerGroup(*m_rightMotor[kFront]);
	m_drive              = new DifferentialDrive(*m_left, *m_right);
	m_needFree           = true;
	InitDalekDrive();
}

DalekDrive::DalekDrive(CANSparkMax* leftMotor, CANSparkMax* rightMotor)
{
	m_leftMotor[kFront]  = leftMotor;
	m_rightMotor[kFront] = rightMotor;
    m_left               = new SpeedControllerGroup(*m_leftMotor[kFront])
    m_right              = new SpeedControllerGroup(*m_rightMotor[kFront])
	m_drive              = new DifferentialDrive(*m_left, *m_right);
	m_needFree           = false;
	InitDalekDrive();
}

DalekDrive::DalekDrive(CANSparkMax& leftMotor, CANSparkMax& rightMotor)
{
	m_leftMotor[kFront]  = &leftMotor;
	m_rightMotor[kFront] = &ightMotor;
    m_left               = new SpeedControllerGroup(*m_leftMotor[kFront])
    m_right              = new SpeedControllerGroup(*m_rightMotor[kFront])
	m_drive              = new DifferentialDrive(*m_left, *m_right);
	m_needFree           = false;
	InitDalekDrive();
}

DalekDrive::DalekDrive(CANSparkMax* leftMotor, CANSparkMax* leftSlaveMotor,
         CANSparkMax* rightMotor, CANSparkMax* rightSlaveMotor)
{
	m_leftMotor[kFront]  = leftMotor;
	m_rightMotor[kFront] = rightMotor;
	m_leftMotor[kRear]   = leftSlaveMotor;
	m_rightMotor[kRear]  = rightSlaveMotor;
    m_left               = new SpeedControllerGroup(*m_leftMotor[kFront]);
    m_right              = new SpeedControllerGroup(*m_rightMotor[kFront]);
	m_drive              = new DifferentialDrive(*m_left, *m_right);
	m_needFree           = false;
	InitDalekDrive();
}

DalekDrive::DalekDrive(CANSparkMax& leftMotor, CANSparkMax& leftSlaveMotor,
         CANSparkMax& rightMotor, CANSparkMax& rightSlaveMotor)
{
	m_leftMotor[kFront]  = leftMotor;
	m_rightMotor[kFront] = rightMotor;
	m_leftMotor[kRear]   = leftSlaveMotor;
	m_rightMotor[kRear]  = rightSlaveMotor;
    m_left               = new SpeedControllerGroup(*m_leftMotor[kFront]);
    m_right              = new SpeedControllerGroup(*m_rightMotor[kFront]);
	m_drive              = new DifferentialDrive(*m_left, *m_right);
	m_needFree           = false;
	InitDalekDrive();
}

DalekDrive::~DalekDrive()
{
	delete m_drive;
    delete m_left;
    delete m_right;
	if(m_needFree) {
		delete m_leftMotor[kFront];
		delete m_rightMotor[kFront];
		if(m_leftMotor[kRear])
			delete m_leftMotor[kRear];
		if(m_rightMotor[kRear])
			delete m_rightMotor[kRear];
	}
	m_needFree = false;
	return;
}

void
DalekDrive::Drive(double outputMagnitude, double curve)
{
	if(m_drive)
		m_drive->Drive(outputMagnitude, curve);
	return;
}

void
DalekDrive::TankDrive(GenericHID* leftStick, GenericHID* rightStick,
             bool squaredInputs)
{
	if(m_drive)
		m_drive->TankDrive(leftStick, rightStick, squaredInputs);
}

void
DalekDrive::TankDrive(GenericHID& leftStick, GenericHID& rightStick,
             bool squaredInputs)
{
	if(m_drive)
		m_drive->TankDrive(leftStick, rightStick, squaredInputs);
}

void
DalekDrive::TankDrive(double leftValue, double rightValue,
             bool squaredInputs)
{
	if(m_drive)
		m_drive->TankDrive(leftValue, rightValue, squaredInputs);
}

void
DalekDrive::ArcadeDrive(GenericHID* stick, bool squaredInputs)
{
	if(m_drive)
		m_drive->ArcadeDrive(stick, squaredInputs);
}

void
DalekDrive::ArcadeDrive(GenericHID& stick, bool squaredInputs)
{
	if(m_drive)
		m_drive->ArcadeDrive(stick, squaredInputs);
}

void
DalekDrive::ArcadeDrive(double moveValue, double rotateValue,
               bool squaredInputs)
{
	if(m_drive)
		m_drive->ArcadeDrive(moveValue, rotateValue, squaredInputs);
}

void
DalekDrive::SetLeftRightMotorOutputs(double leftOutput, double rightOutput)
{
	if(m_drive)
		m_drive->SetLeftRightMotorOutputs(leftOutput, rightOutput);
}

void
DalekDrive::SetInvertedMotor(RobotSide side, bool isInverted)
{
	switch(side) {
	case kLeft:
		if(m_leftMotor[kFront])
			m_leftMotor[kFront]->SetInverted(isInverted);
		break;

	case kRight:
		if(m_rightMotor[kFront])
			m_rightMotor->SetInverted(isInverted);
		break;

	default:
		break;
	}
}

void
DalekDrive::SetSensitivity(double sensitivity)
{
	if(m_drive)
		m_drive->SetSensitivity(sensitivity);
}

void
DalekDrive::SetMaxOutput(double maxOutput)
{
	if(m_drive)
		m_drive->SetMaxOutput(maxOutput);
}

void
DalekDrive::InitDalekDrive(void)
{
    // Setup for LiveWindow
    lw->AddActuator("Drive System Left", "Drive Motor", m_leftMotor[kFront]);
    lw->AddActuator("Drive System Right", "Drive Motor", m_rightMotor[kFront]);
    if(m_leftMotor[kRear])
        lw->AddActuator("Drive System Left", "Slave Drive Motor", 
                m_leftMotor[kRear]);
    if(m_rightSlaveMotor)
        lw->AddActuator("Drive System Right", "Slave Drive Motor", 
                m_rightMotor[kRear]);

	// Configure the SparkMax
    m_leftMotor[kFront]->SetCANTimeout(CAN_TIMEOUT);
	m_leftMotor[kFront]->SetIdleMode(CANSparkMax::kBrake);
    m_leftMotor[kFront]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	m_leftMotor[kFront]->SetRampRate(RAMP_RATE);
	m_leftMotor[kFront]->SetInverted(false);

    m_rightMotor[kFront]->SetCANTimeout(CAN_TIMEOUT);
	m_rightMotor[kFront]->SetIdleMode(CANSparkMax::kBrake);
    m_rightMotor[kFront]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	m_rightMotor[kFront]->SetRampRate(RAMP_RATE);
	m_rightMotor[kFront]->SetInverted(false);

    if(m_leftMotor[kRear]) {
        m_leftMotor[kRear]->SetCANTimeout(CAN_TIMEOUT);
        m_leftMotor[kRear]->Follow(*m_leftMotor[kFront], false);
	    m_leftMotor[kRear]->SetIdleMode(CANSparkMax::kBrake);
        m_leftMotor[kRear]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	    m_leftMotor[kRear]->SetRampRate(RAMP_RATE);
	    m_leftMotor[kRear]->SetInverted(false);
    }

    if(m_rightMotor[kRear]) {
        m_rightMotor[kRear]->SetCANTimeout(CAN_TIMEOUT);
        m_rightMotor[kRear]->Follow(*m_rightMotor[kFront], false);
	    m_rightMotor[kRear]->SetIdleMode(CANSparkMax::kBrake);
        m_rightMotor[kRear]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	    m_rightMotor[kRear]->SetRampRate(RAMP_RATE);
	    m_rightMotor[kRear]->SetInverted(false);
    }
    m_leftMotor->StopMotor();
    m_rightMotor->StopMotor();
}

void
DalekDrive::printFaults(RobotSide p, int faults)
{
    switch(p) {
	case kLeft: 
		frc::SmartDashboard::PutNumber("Left Drive Motor reported faults", 
            faults);
		if(m_leftMotor[kRear]) {
			frc::SmartDashboard::PutNumber("Left slave status",
					m_leftMotor[kRear]->GetFaults());
		}
        break; 
    case kRight:
		frc::SmartDashboard::PutNumber("Right Drive Motor reported faults",
            faults);
		if(m_rightMotor[kRear]) {
			frc::SmartDashboard::PutNumber("Right slave status",
					m_rightMotor[kRear]->GetFaults());
		}
        break;
    default:
        break;
    }
	return;
}

bool
DalekDrive::DriveOk()
{
	CANSparkMax::FeedbackDeviceStatus estat;
	int mstat;

	// update dashboard of current draw for motors
	frc::SmartDashboard::PutNumber("Left Motor current",
			m_leftMotor[kFront]->GetOutputCurrent());
	if(m_leftMotor[kRear])
		frc::SmartDashboard::PutNumber("Left Slave Motor current",
			m_leftMotor[kRear]->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Right Motor current",
			m_rightMotor[kFront]->GetOutputCurrent());
	if(m_rightMotor[kRear])
		frc::SmartDashboard::PutNumber("Right Slave Motor current",
			m_rightMotor[kRear]->GetOutputCurrent());

	// check for motor faults
	mstat = m_leftMotor[kFront]->GetFaults();
	if(mstat != 0) {
		printFaults(kLeft, mstat);
		return false;
	}
	mstat = m_leftMotor[kFront]->GetStickyFaults();
	if(mstat) {
		printFaults(kLeft, mstat);
		m_leftMotor[kFront]->ClearStickyFaults();
		return false;
	}

	mstat = m_rightMotor[kFront]->GetFaults();
	if(mstat) {
		printFaults(kRight, mstat);
		return false;
	}
	mstat = m_rightMotor[kFront]->GetStickyFaults();
	if(mstat) {
		printFaults(kRight, mstat);
		return false;
	}
	return true;
}
