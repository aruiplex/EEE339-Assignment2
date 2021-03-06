// megafunction wizard: %ROM: 1-PORT%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: lpm_rom 

// ============================================================
// File Name: IMemory.v
// Megafunction Name(s):
// 			lpm_rom
//
// Simulation Library Files(s):
// 			lpm
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 7.2 Build 151 09/26/2007 SJ Full Version
// ************************************************************


//Copyright (C) 1991-2007 Altera Corporation
//Your use of Altera Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Altera Program License 
//Subscription Agreement, Altera MegaCore Function License 
//Agreement, or other applicable license agreement, including, 
//without limitation, that your use is for the sole purpose of 
//programming logic devices manufactured by Altera and sold by 
//Altera or its authorized distributors.  Please refer to the 
//applicable agreement for further details.


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module IMemory (
	address,
	q);

	input	[4:0]  address;
	output	[31:0]  q;

	wire [31:0] sub_wire0;
	wire [31:0] q = sub_wire0[31:0];

	lpm_rom	lpm_rom_component (
				.address (address),
				.q (sub_wire0),
				.inclock (1'b1),
				.memenab (1'b1),
				.outclock (1'b1));
	defparam
		lpm_rom_component.intended_device_family = "APEX20KE",
		lpm_rom_component.lpm_address_control = "UNREGISTERED",
		lpm_rom_component.lpm_file = "IMemory.mif",
		lpm_rom_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
		lpm_rom_component.lpm_outdata = "UNREGISTERED",
		lpm_rom_component.lpm_type = "LPM_ROM",
		lpm_rom_component.lpm_width = 32,
		lpm_rom_component.lpm_widthad = 5;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ADDRESSSTALL_A NUMERIC "0"
// Retrieval info: PRIVATE: AclrAddr NUMERIC "0"
// Retrieval info: PRIVATE: AclrByte NUMERIC "0"
// Retrieval info: PRIVATE: AclrOutput NUMERIC "0"
// Retrieval info: PRIVATE: BYTE_ENABLE NUMERIC "0"
// Retrieval info: PRIVATE: BYTE_SIZE NUMERIC "8"
// Retrieval info: PRIVATE: BlankMemory NUMERIC "0"
// Retrieval info: PRIVATE: CLOCK_ENABLE_INPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: CLOCK_ENABLE_OUTPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: Clken NUMERIC "0"
// Retrieval info: PRIVATE: IMPLEMENT_IN_LES NUMERIC "0"
// Retrieval info: PRIVATE: INIT_FILE_LAYOUT STRING "PORT_A"
// Retrieval info: PRIVATE: INIT_TO_SIM_X NUMERIC "0"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "APEX20KE"
// Retrieval info: PRIVATE: JTAG_ENABLED NUMERIC "0"
// Retrieval info: PRIVATE: JTAG_ID STRING "NONE"
// Retrieval info: PRIVATE: MAXIMUM_DEPTH NUMERIC "0"
// Retrieval info: PRIVATE: MIFfilename STRING "IMemory.mif"
// Retrieval info: PRIVATE: NUMWORDS_A NUMERIC "32"
// Retrieval info: PRIVATE: OutputRegistered NUMERIC "0"
// Retrieval info: PRIVATE: RAM_BLOCK_TYPE NUMERIC "0"
// Retrieval info: PRIVATE: RegAdd NUMERIC "0"
// Retrieval info: PRIVATE: RegAddr NUMERIC "0"
// Retrieval info: PRIVATE: RegOutput NUMERIC "0"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: SingleClock NUMERIC "1"
// Retrieval info: PRIVATE: UseDQRAM NUMERIC "0"
// Retrieval info: PRIVATE: WidthAddr NUMERIC "5"
// Retrieval info: PRIVATE: WidthData NUMERIC "32"
// Retrieval info: PRIVATE: rden NUMERIC "0"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "APEX20KE"
// Retrieval info: CONSTANT: LPM_ADDRESS_CONTROL STRING "UNREGISTERED"
// Retrieval info: CONSTANT: LPM_FILE STRING "IMemory.mif"
// Retrieval info: CONSTANT: LPM_HINT STRING "ENABLE_RUNTIME_MOD=NO"
// Retrieval info: CONSTANT: LPM_OUTDATA STRING "UNREGISTERED"
// Retrieval info: CONSTANT: LPM_TYPE STRING "LPM_ROM"
// Retrieval info: CONSTANT: LPM_WIDTH NUMERIC "32"
// Retrieval info: CONSTANT: LPM_WIDTHAD NUMERIC "5"
// Retrieval info: USED_PORT: address 0 0 5 0 INPUT NODEFVAL address[4..0]
// Retrieval info: USED_PORT: q 0 0 32 0 OUTPUT NODEFVAL q[31..0]
// Retrieval info: CONNECT: @address 0 0 5 0 address 0 0 5 0
// Retrieval info: CONNECT: q 0 0 32 0 @q 0 0 32 0
// Retrieval info: LIBRARY: lpm lpm.lpm_components.all
// Retrieval info: GEN_FILE: TYPE_NORMAL IMemory.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL IMemory.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL IMemory.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL IMemory.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL IMemory_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL IMemory_bb.v FALSE
// Retrieval info: LIB_FILE: lpm
