// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
	//Create compressor object.
	private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
	
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

	@Override
	public void periodic() {
	}


}