// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
	//Create compressor object with CAN ID 10(This will use the same Pneumatic Hub as mentioned below).
	private final Compressor m_compressor = new Compressor(10, PneumaticsModuleType.REVPH);
	//Create Pneumatic Hub object with CAN ID 10.
	private final PneumaticHub m_pH = new PneumaticHub(10);

	//Create Solenoids
	DoubleSolenoid m_arm1 = m_pH.makeDoubleSolenoid(0,1);
	DoubleSolenoid m_arm2 = m_pH.makeDoubleSolenoid(2,3);
	DoubleSolenoid m_arm3 = m_pH.makeDoubleSolenoid(4,5);
	DoubleSolenoid m_arm4 = m_pH.makeDoubleSolenoid(6,7);
	
		public PneumaticSubsystem() {
	
		}
	
	public double getCurrent() {
		return m_compressor.getCurrent();
	}

	public boolean getStatus() {
		return m_compressor.isEnabled();
	}

	public boolean getPressureSwitchValue() {
		return m_compressor.getPressureSwitchValue();
	}

	/**
	* @param armID		ID of the solenoid being referenced in the operation
	* @param value		Value of the solenoid to set. Valid arguments are Value.kOff, Value.kForward, and Value.kReverse.
	*/
	public void setArmStatus(int armID, Value value){
		DoubleSolenoid arm = null;
		switch(armID){
			case(1):
				arm = m_arm1;
			case(2):
				arm = m_arm2;
			case(3):
				arm = m_arm3;
			case(4):
				arm = m_arm4;
		}
		arm.set(value);
	}

	@Override
	public void periodic() {
	}


}