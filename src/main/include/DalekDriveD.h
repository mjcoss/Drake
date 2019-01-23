/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 * Utility class for handling Robot differential drive 
 */
#pragma once
#include <Drake.h>

#define CAN_TIMEOUT 100
#define STALL_LIMIT 60
#define FREE_LIMIT  2
#define RAMP_RATE   1.25

class DalekDriveD {
  public:
    enum class RobotSide { kLeft = 0, kRight = 1, kNumSides  };
    enum class RobotMotors { kFront = 0, kRear = 1, kNumMotors  };

	DalekDrive(int leftMotorChannel, int rightMotorChannel);
	DalekDrive(int leftMotorChannel, int leftSlaveMotorChannel,
             int rightMotorChannel, int rightSlaveMotorChannel);

	DalekDrive(CANSparkMax* leftMotor, CANSparkMax* rightMotor);
	DalekDrive(CANSparkMax& leftMotor, CANSparkMax& rightMotor);
	DalekDrive(CANSparkMax* leftMotor, CANSparkMax* leftSlaveMotor,
             CANSparkMax* rightMotor, CANSparkMax* rightSlaveMotor)
	DalekDrive(CANSparkMax& leftMotor, CANSparkMax& leftSlaveMotor,
             CANSparkMax& rightMotor, CANSparkMax& rightSlaveMotor)

	~DalekDrive();

	void TankDrive(frc::GenericHID* leftStick, frc::GenericHID* rightStick,
                 bool squaredInputs = true);
	void TankDrive(frc::GenericHID& leftStick, frc::GenericHID& rightStick,
                 bool squaredInputs = true);
	void TankDrive(double leftValue, double rightValue,
                 bool squaredInputs = true);

	void ArcadeDrive(frc::GenericHID* stick, bool squaredInputs = true);
	void ArcadeDrive(frc::GenericHID& stick, bool squaredInputs = true);
	void ArcadeDrive(double moveValue, double rotateValue,
                   bool squaredInputs = true);

	void SetLeftRightMotorOutputs(double leftOutput, double rightOutput);
	void SetInvertedMotor(RobotSide side, bool isInverted);
	bool DriveOk();

 private:
	void InitDalekDriveD();
	void printFaults(RobotSide side, int faults);
	rev::CANSparkMax *m_leftMotor[kNumSides];
	rev::CANSparkMax *m_rightMotor[kNumSides];
    frc::SpeedControlGroup *m_left;
    frc::SpeedControlGroup *m_right;
	frc::DifferentialDrive *m_drive;
	bool m_needFree;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
};
