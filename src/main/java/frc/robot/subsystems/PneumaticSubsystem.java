// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@SuppressWarnings("unused")
public class PneumaticSubsystem extends SubsystemBase {
	//Create compressor object with CAN ID 10(This will use the same Pneumatic Hub as mentioned below).
	private final Compressor m_compressor = new Compressor(21, PneumaticsModuleType.REVPH);
	//Create Pneumatic Hub object with CAN ID 10.
	private final PneumaticHub m_pH = new PneumaticHub(21);

	//Create Solenoids
	Solenoid m_sArm1 = m_pH.makeSolenoid(1);
	Solenoid m_sArm2 = m_pH.makeSolenoid(2);
	Solenoid m_sArm3 = m_pH.makeSolenoid(3);
	Solenoid m_sArm4 = m_pH.makeSolenoid(4);
	Solenoid m_sArm5 = m_pH.makeSolenoid(5);
	DoubleSolenoid m_dArm1 = m_pH.makeDoubleSolenoid(6,7);
	DoubleSolenoid m_dArm2 = m_pH.makeDoubleSolenoid(8,9);

	
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

	public void toggleSolenoid(int armID) {
		switch(armID){
			case(1):
				m_sArm1.set(!m_sArm1.get()); // Toggles solenoid 1 between ON and OFF
				System.out.println("Solenoid 1 toggled: " + m_sArm1.get()); // Debugging message
				return;
			case(2):
				m_sArm2.set(!m_sArm2.get()); // Toggles solenoid 2 between ON and OFF
				System.out.println("Solenoid 2 toggled: " + m_sArm2.get()); // Debugging message
				return;
			case(3):
				m_sArm3.set(!m_sArm3.get()); // Toggles solenoid 3 between ON and OFF
				System.out.println("Solenoid 3 toggled: " + m_sArm3.get()); // Debugging message
				return;
			case(4):
				m_sArm4.set(!m_sArm4.get()); // Toggles solenoid 4 between ON and OFF
				System.out.println("Solenoid 4 toggled: " + m_sArm1.get()); // Debugging message
				return;
			case(5):
				m_sArm5.set(!m_sArm5.get()); // Toggles solenoid 5 between ON and OFF
				System.out.println("Solenoid 5 toggled: " + m_sArm5.get()); // Debugging message	
				return;
		}}
/**@param armID		ID of the solenoid being referenced in the operation
 * @param value		Value of the solenoid to set. Valid arguments are Value.kOff, Value.kForward, and Value.kReverse.
*/
		public void toggleDSolenoid(int armID, DoubleSolenoid.Value value) {
			switch(armID){
				case(6):
					m_dArm1.set(value);
					System.out.println("Solenoid 6 toggled: " + m_dArm1.get()); // Debugging message
				case(7):
					m_dArm2.set(value);
					System.out.println("Solenoid 7 toggled: " + m_dArm2.get()); // Debugging message
		}}
	/**
	* @param armID		ID of the solenoid being referenced in the operation
	* @param value		Value of the solenoid to set. Valid arguments are Value.kOff, Value.kForward, and Value.kReverse.
	*/
	/*public void setArmStatus(int armID, Value value){
		DoubleSolenoid dArm = null;
		Solenoid sArm
		
		m_p
	}*/

	@Override
	public void periodic() {
	}


}