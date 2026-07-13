-- Intraperitoneal Transmitter (IPT, A3054) Firmware, Top-Level Unit

-- V1.1 [25-MAR-26] Based upon P3041 V3.6. Set up inputs and outputs.

-- V1.2 [01-APR-26] Bring in the P3051 ring oscillator implementation: a big ring
-- with no divider creates FCK. Remove the Clock Calibrator process and hard-wire
-- fck_divisor in firmware. Compile for a 31-gate ring, set route priority on all
-- thirty-one gates, then re-compile for fewer gates and ignore warnings. Adapt
-- the P3041 Stimulus Controller for our LED output. In software, we have disabled
-- the P3041 Stimulus Controller for our LED output. In software, we have disabled
-- the Clock Calibration call. Tested and fully functional without any readout of
-- sensors or converters. Compiled size 1206 LUTs.

-- V1.3 [02-APR-26] Add I2C interface. Adapt Sensor Controller for the ADS7052.
-- Re-number the control registers 0-31, there are exactly thirty-two right now.
-- We can read ADC HI and LO bytes, as well as I2C byte. Sensor Controller is
-- untested, but I2C communication with TMP117 fully operational.

-- V1.4 [09-APR-26] Sensor Controller becomes the ADC Controller. Rename sensor
-- signals. Controller supports one and two left shifts of fourteen-bit data.
-- The ADC control register sets shift and selects the ADC. Amplifier control
-- register turns on and off DC coupling and impedance measurement switch. The 
-- ADC readout runs off FCK with SCK at 5 MHz.

-- V1.5 [01-MAY-26] Add readback of amplifier configuration register. Eliminate 
-- support for random stimuli in software. Remove multiplier from software. Now
-- have I2C.asm include file. Transmit signal can be directed to any input, AC
-- or DC, temperature sensor, accelerometer, EEPROM, or synchronizing signal.
-- Simplify LED controller.

-- V1.6 [04-JUN-26] Remove shutdown counter from software. Add main loop counter
-- to control 0.5-Hz updates of temperature sensor. Read out temperature sensor
-- just before we initate a new measurement. Add identifier transmission as a
-- transmit current cost reference.

-- V1.7 [08-JUN-26] Move all thermometer timing into the interrupt routine. Make
-- sure CPU is in boost when performing ADC calibration, and insert calibration
-- delays. Remove stimulus support, except that on stimulus start, the LED turns
-- on, and on stimulus stop, it turns off. Extend adc data to eighteen bits with
-- zero to four left shifts at end of readout. 

-- V1.8 [09-JUN-26] Add four eighteen-bit accumulators for box filters. Code is 14 
-- LUTs and 12 SLICEs too large. Eliminate the X3 and X4 box filters and allow 
-- direct access to adc_data. Code fits. We have accumulators for X1 and X2.
-- Eliminate CRC check of command. We plan to implement later in software. Convert 
-- the command initiate and terminate processes to using the millisecond clock with
-- synchronous reset, which reduces the number of registers used for counting. 
-- Eliminate access to raw ADC data.

-- V1.9 [10-JUN-26] Reduce the CPU address from eleven to ten bits. Compress the
-- memory mapy. Restrict user code to 1 KByte and place in CPU range 0x0400 to
-- 0x07FF. In first 1 KByte we have 256 bytes each for control space, stack, user
-- variables and main program variables. The control space is now shadowed by
-- the RAM, so we eliminate some read-back code from the MMU. Restore access to 
-- raw ADC data. Sampling always at 1024 SPS, box filter provides 128, 256, 512,
-- and 1024 SPS at request of Stimulator, any request unequal to these defaulting
-- to 1024 SPS. Tested for AC, believe now working for DC too. 

-- V1.10 [11-JUN-26] Instead of four accumulators, use a sample memory made out
-- of our last EBR block, and share a single accumulator to calculate the transmit
-- sample value at transmit time. Size drops from 1250 to 1150 LUTs.

-- V1.11 [27-JUN-26] Replace TXA, which we never use, with MCK in the status register.
-- Now we can use MCK as a millisecond timer that works in both boost and slow
-- modes. Expand MMU comments. In software, insist that NVM writes be on page
-- boundaries.

-- V1.12 [03-JUL-26] Add shifting back into the Sample Controller. All sensors and
-- converters can be configured in software. Readout and writing to NVM working.
-- Only problem is 30 uA overhead of a 1024 Hz interrupt.

-- V1.13 [05-JUL-26] Reduce Sample Memory to four locations 18 bits wide. Route
-- output of Sample Memory to one input of the Sample Accumulator. The other 
-- input comes from the ADC Controller. Upon assertion of ADCRD, the ADC Controller
-- reads a fourteen-bit sample from the ADC selected by the two-bit Sample Address,
-- shifts it in accordance with the Accumulation Shift bits, adds it to the 
-- output of the Sample Memory, and loads the sum back into the same location in
-- sample memory. Eliminate direct readout of the ADC samples. Add ACCRST and 
-- SMWRCPU so we can clear values in the sample memory. Build Sample Controller
-- to direct the ADC Controller to read out the ADCs one after another.

-- V1.14 [07-JUL-26] Add sample counters for X1-X4 so the planned Telemetry 
-- Controller can instruct the Sample Controller to skip an ADC or to sample
-- an ADC less frequently than 1024 SPS. Build Telemetry Manager to turn on 
-- FCK and start the launch the Sample Controller at 1024 Hz. Add telemetry
-- time shift generator to ADC Controller. Remove all interrupt bits except 
-- bit 0. Generate int0 with Telemetry Manager. Enable Telemetry Manager with 
-- interrupt mask bit0. Add X4SS for "Input X4 Single Sample" to allow 
-- sampling of chaotic signals without low-pass filtering.

-- V1.15 [08-JUL-26] Ring oscillator now software-programmable with eight-bit
-- mask written to location mmu_fck. Fix bug in X4SS implementation.

-- V1.16 [09-JUL-26] Calibration now being read from NVM. Can change RF
-- calibration, ring oscillator frequency, and set the device identifier from
-- our Telemetry Manager. Check for bad values of ID and mask. Combine Lon and 
-- Loff into Flash operation. Add and implement software reset operation. New
-- operation code values.

-- V1.17 [10-JUL-26] Configuration from NVM implemented and tested. Can now
-- configure all sensors and sample rates.

-- V1.18 [13-JUL-26] The IPT starts executing its telemetry protocol when we
-- wake it up.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		RP, -- Receive Power
		RCK, -- Reference Clock
		SDO -- Serial Data Out for ADCs
		: in std_logic; 
		LV, -- Lower Logic Core Voltage
		XEN, -- Transmit Enable, for data transmission
		TP, -- Test Point, available on P1-7
		SCL, -- Serial Clock for I2C Bus
		LED, -- Turn On Indicator Lamp
		SCK, -- Serial Clock for ADC Bus
		DC, -- Direct Coupled Amplifiers
		MSR, -- Measure Input Impedance
		NADC1, -- Select ADC1, Negative True
		NADC2, -- Select ADC2, Negative True
		NADC3, -- Select ADC3, Negative True
		NADC4  -- Select ADC4, Negative True
		: out std_logic;
		SDA -- Serial Data Access for I2C Bus
		: inout std_logic;
		xdac -- Transmit DAC Output, to set data transmit frequency
		: out std_logic_vector(4 downto 0));
-- Configuration of the Ring Oscillator
	constant fck_num_feeds : integer := 8;
	constant fck_num_gates : integer := 24;

-- Configuration and Calibration of Transmitter.
	constant tx_channel_default : integer := 1;
	constant frequency_step : integer := 1; 
	constant default_frequency_low : integer := 5;
		
-- Configuration of OSR8 CPU.
	constant prog_cntr_len : integer := 12;
	constant cpu_addr_len : integer := 10;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;
	constant ram_addr_len : integer := 10;

-- Memory Map Constants, sizes and base addresses.
	constant ctrl_bot : integer := 16#0000#;
	constant ctrl_top : integer := 16#00FF#;
	constant ram_bot  : integer := 16#0100#;
	constant ram_top  : integer := 16#03FF#;
	
-- Memory Map Constants, low nibble addresses in units of bytes. When a location is
-- a control register shadowed in RAM, we say it is "Write/Readback". When writing to
-- a location initiates a process, we say it is "Write". When a location can be read
-- but not written, we say it is "Read".
	constant mmu_irqb  : integer := 16#00#; -- Interrupt Request Bits (Read)
	constant mmu_imsk  : integer := 16#01#; -- Interrupt Mask Bits (Write/Readback)
	constant mmu_irst  : integer := 16#02#; -- Interrupt Reset Bits (Write)
	constant mmu_acfg  : integer := 16#03#; -- Amplifier Configuration (Write/Readback)
	constant mmu_led   : integer := 16#04#; -- Lamp Switch (Write/Readback)
	constant mmu_rst   : integer := 16#05#; -- Software Reset (Write)
	constant mmu_xhb   : integer := 16#06#; -- Transmit HI Byte (Write/Readback)
	constant mmu_xlb   : integer := 16#07#; -- Transmit LO Byte (Write/Readback)
	constant mmu_xch   : integer := 16#08#; -- Transmit Channel Number (Write/Readback)
	constant mmu_xcr   : integer := 16#09#; -- Transmit Control (Write)
	constant mmu_rfc   : integer := 16#0A#; -- Radio Frequency Calibration (Write/Readback)	
	constant mmu_ccr   : integer := 16#0B#; -- Clock Control (Write/Readback)
	constant mmu_dfr   : integer := 16#0C#; -- Diagnostic Flags (Write/Readback)
	constant mmu_sr    : integer := 16#0D#; -- Status Register (Read)
	constant mmu_cmp   : integer := 16#0E#; -- Command Memory Portal (Read)
	constant mmu_cpr   : integer := 16#0F#; -- Command Processor Reset (Write)
	constant mmu_fck   : integer := 16#10#; -- Fast Clock Mask (Write/Readback)
	constant mmu_i0p   : integer := 16#11#; -- Interrupt Zero Period (Write/Readback)
	constant mmu_i2c00 : integer := 16#12#; -- i2c SDA=0 SCL=0 (Write/Readback)
	constant mmu_i2c01 : integer := 16#13#; -- i2c SDA=0 SCL=1 (Write/Readback)
	constant mmu_i2cA0 : integer := 16#14#; -- i2c SDA=A SCL=0 (Write/Readback)
	constant mmu_i2cA1 : integer := 16#15#; -- i2c SDA=A SCL=1 (Write/Readback)
	constant mmu_i2cZ0 : integer := 16#16#; -- i2c SDA=Z SCL=0 (Write/Readback)
	constant mmu_i2cZ1 : integer := 16#17#; -- i2c SDA=Z SCL=1 (Write/Readback)
	constant mmu_i2cMR : integer := 16#18#; -- i2C Most Recent Eight Bits (Read)
	constant mmu_smcr  : integer := 16#19#; -- Sample Control (Write/Readback)
	constant mmu_x1cfg : integer := 16#1A#; -- X1 Configuration (Write/Readback)
	constant mmu_x2cfg : integer := 16#1B#; -- X2 Configuration (Write/Readback)
	constant mmu_x3cfg : integer := 16#1C#; -- X3 Configuration (Write/Readback)
	constant mmu_x4cfg : integer := 16#1D#; -- X4 Configuration (Write/Readback)
	constant mmu_smemh : integer := 16#1E#; -- Sample HI Byte (Read)
	constant mmu_smeml : integer := 16#1F#; -- Sample LO Byte (Read)
	
-- Sample and Accumulate Constants
	constant x1_sel : std_logic_vector(1 downto 0) := "00";
	constant x2_sel : std_logic_vector(1 downto 0) := "01";
	constant x3_sel : std_logic_vector(1 downto 0) := "10";
	constant x4_sel : std_logic_vector(1 downto 0) := "11";
end;

architecture behavior of main is

-- Attributes that request signals to be retained instead of distributed.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Power Controller
	signal USERSTDBY, CLRFLAG : std_logic := '0';
	signal RESET : std_logic := '1';
	attribute syn_keep of RESET : signal is true;
	attribute nomerge of RESET : signal is "";
	signal SFLAG, STDBY : std_logic;
	signal SWRST : boolean := false;
	
-- Ring Oscillator, Fast Clock, and Transmit Clock. We retain all these signals
-- as nodes in the logic to stop the compiler from spreading them around, which
-- makes the ring oscillator period vary from one compile to the next.
	signal TCK, FCK, FCKEN, CK, MCK : std_logic;
	attribute syn_keep of TCK, FCK, FCKEN, CK, MCK : signal is true;
	attribute nomerge of TCK, FCK, FCKEN, CK, MCK : signal is ""; 
	signal fck_mask : std_logic_vector(fck_num_feeds-1 downto 0);
	constant default_fck_mask :
		std_logic_vector(fck_num_feeds-1 downto 0) :=
		(fck_num_feeds-1 => '1', others => '0');

-- Message Transmission. The syn_keep and nomerge reduce code size slightly.
	signal TXI, -- Transmit Initiate
		TXWP, -- Transmit Warmup
		TXA, -- Transmit Active
		TXB, -- Transmit Bit
		FHI -- Frequency High
		: boolean := false;
	attribute syn_keep of TXI, TXA : signal is true;
	attribute nomerge of TXI, TXA : signal is "";  
	signal xmit_bits : std_logic_vector(15 downto 0);
	signal tx_channel : integer range 0 to 255;
	signal frequency_low : integer range 0 to 31;
	signal transmit_shift : std_logic_vector(3 downto 0);
		
-- Telemetry Manager
	signal TMINT, -- Telemetry Manager Interrupt
		TMINTD -- TMINT Delayed
		: boolean := false;

-- Sample Controller
	signal SCRUN, -- Sample Controller Run
		SCBSY, -- Sample Controller Busy
		SCDONE, -- Sample Controller Done
		X4SS -- Channel X4 Single Sample
		: boolean := false;
	attribute syn_keep of SCRUN, SCBSY, SCDONE : signal is true;
	attribute nomerge of SCRUN, SCBSY,SCDONE : signal is "";  
	signal x1_shift, x2_shift, x3_shift, x4_shift : std_logic_vector(2 downto 0);

-- Sample Memory and Accumulator
	signal SMWRADC, -- Sample Memory Write by ADC Controller
		SMWRCPU, -- Sample Memory Write by CPU
		ACCADD, -- Accumulator Add
		ACCRST -- Accumulator Reset
		: std_logic;
	attribute syn_keep of ACCADD, ACCRST : signal is true;
	attribute nomerge of ACCADD, ACCRST : signal is "";  
	signal smem_data, acc_data : std_logic_vector(17 downto 0);
	signal smem_addr, sample_sel : std_logic_vector(1 downto 0);

-- ADC Controller.
	signal ADCCAL, -- Calibrate the Selected ADC
		ADCRD, -- Initiate ADC Read
		ADCBSY -- ADC Busy
		: boolean := false;
	attribute syn_keep of ADCRD, ADCBSY : signal is true;
	attribute nomerge of ADCRD, ADCBSY : signal is "";  
	signal adc_data : std_logic_vector(17 downto 0);
	signal adc_sel : std_logic_vector(1 downto 0);
	signal adc_shift : std_logic_vector(2 downto 0);

-- I2C Bus Controller
	signal i2c_in : std_logic_vector(7 downto 0); -- I2C Serial Byte

-- Clock Control. 
	signal FCKENCPU, -- Fast Clock Enable from CPU
		FCKENTM, -- Fast Clock from Telemetry Manager
		BOOST, -- Boost CPU Clock
		KEEPFCK, -- Keep FCK Running
		RCKLO -- RCK is LO
		: boolean; 
	signal SRCK, -- RCK Synchronized with FCK
		SSRCK -- SRCK Delayed by one FCK Period
		: std_logic; 
	attribute syn_keep of BOOST, FCKENCPU, FCKENTM : signal is true;
	attribute nomerge of BOOST, FCKENCPU, FCKENTM : signal is "";
	
-- Diagnostic Flag Register
	signal df_reg : std_logic_vector(3 downto 0);

-- Program Memory Signals
	signal prog_data : std_logic_vector(7 downto 0); -- ROM Data
	signal prog_cntr : std_logic_vector(prog_cntr_len-1 downto 0); -- Prog Address
	
-- Process Memory Signals
	signal ram_addr : std_logic_vector(ram_addr_len-1 downto 0); -- RAM Address
	signal ram_out, ram_in : std_logic_vector(7 downto 0); -- RAM Data In and Out
	signal RAMWR : std_logic; -- Command Memory Write
	
-- Central Processing Unit Signals. The syn_keep and nomerge reduce code size slightly.
	signal cpu_data_out, cpu_data_in : std_logic_vector(7 downto 0); 
	signal cpu_addr : std_logic_vector(cpu_addr_len-1 downto 0);
	attribute syn_keep of cpu_addr : signal is true;
	attribute nomerge of cpu_addr : signal is "";  
	signal CPUWR, -- Write (Not Read)
		CPUDS, -- Data Strobe
		CPUIRQ, -- Interrupt Request
		CPUISRV -- Interrupt Service
		: boolean; 
	signal CPUSIG : std_logic_vector(3 downto 0); -- Signals for debugging

-- Interrupt Handler signals.
	signal int_mask, int_bits, int_rst : std_logic_vector(7 downto 0);
	signal int_rst_d, int_rst_s : std_logic_vector(7 downto 0);
	signal int_period_0 : std_logic_vector(4 downto 0);
	
-- Byte Receiver
	signal RPS, -- Radio Frequency Power Synchronized
		ICMD, -- Initiate Command Reception
		TCMD, -- Terminate Command Reception
		RCMD, -- Receive Command
		RBI, -- Receive Command Byte Initiate
		RBD, -- Receive Command Byte Done
		BYTERR, -- Byte Error
		CRCERR, -- Checksum Error
		BYTS, -- Command Byte Strobe
		CBS -- Command Bit Strobe (CBS)
		: boolean := false; 
	
-- Command Memory
	signal cmd_in : std_logic_vector(7 downto 0); -- Command Memory Data In
	signal cmd_out : std_logic_vector(7 downto 0); -- Command Memory Data Out
	signal CME, -- Command Memory Empty
		CMF, -- Command Memory Full
		CMRST, -- Command Memory Reset
		CMWR,  -- Command Memory Write
		CMRD -- Command Memory Read
		: std_logic; 

-- Command Processor
	signal CPA, -- Command Processor Active
		CMDRDY, -- Command Ready
		CPRST -- Command Processor Reset
		: boolean := false;
			
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin

-- We turn off the logic chip bandgap references and other power-hungry
-- circuits with the power controller unit (PCU). Within a few milliseconds
-- of power-up, the chip is fully operational, but consuming several 
-- milliamps. We must still wait for RCK to start up, which will take
-- roughly 150 ms. Once we have RCK, we move the chip into standby mode by
-- clearing the standby flag with CLRFLAG and asserting USERSTDBY. This
-- begins the transition to standby mode. The PCU has two outputs: STDBY and 
-- SFLAG. The STDBY signal is intended as a command to put circuits to sleep, 
-- while SFLAG is intended as a signal that the system has entered standby 
-- mode. We return to full-power mode when we program the chip.
	Power_Controller: entity PCU port map (
		CLRFLAG => CLRFLAG,
		USERSTDBY => USERSTDBY, 
		STDBY => STDBY,
		SFLAG => SFLAG);	

-- The Power-Up Process. We have CLRFLAG and USERSTDBY cleared LO on power-up,
-- and RESET set HI. When RCK starts up, we us the falling edge to move the 
-- chip into standby mode, then unassert RESET once we receive SFLAG from the
-- Power Control Unit (PCU).
	PowerUp: process (RCK) is
		constant end_state : integer := 7;
		constant clr_state : integer := 3;
		constant stdby_state : integer := clr_state + 2;
		variable state : integer range 0 to end_state := 0;
	begin
		if falling_edge(RCK) then
			CLRFLAG <= to_std_logic(state = clr_state);
			USERSTDBY <= to_std_logic(state >= stdby_state);
			RESET <= to_std_logic((state < end_state) or SWRST);
			LV <= to_std_logic(state = end_state);
			
			if (state < stdby_state) then state := state + 1;
			elsif (SFLAG = '0') then state := stdby_state;
			elsif (state < end_state) then state := state + 1; 
			else state := end_state; end if;
		end if;
	end process;	

-- The Fast Clock process produces FCK when one of several flags are asserted.
-- The ring oscillator should be calibrated so that it produces FCK of 10 MHz.
	FCKEN <= to_std_logic(FCKENCPU or FCKENTM or KEEPFCK or CPUISRV);
	Fast_Clock: entity ring_oscillator 
		generic map (
			num_gates => fck_num_gates,
			num_feeds => fck_num_feeds
		)	
		port map (
			ENABLE => FCKEN, 
			mask => fck_mask,
			CK => FCK
		);
		
-- The Millisecond Clock takes RCK, which is 32.768 kHz and divides
-- by 32 to get 1.024 kHz, which we use as our millisecond clock.
	Millisecond_Clock : process (RCK) is
		variable mcnt : integer range 0 to 31;
	begin
		if falling_edge(RCK) then
			mcnt := mcnt + 1;
			if (mcnt <= 15) then
				MCK <= '0';
			else
				MCK <= '1';
			end if;
		end if;	
	end process;
		
-- The Process Memory is the RAM available to the code executing on the CPU.
	Process_Memory : entity RAM port map (
		Clock => not CK,
		ClockEn => '1',
        Reset => '0',
		WE => RAMWR,
		Address => ram_addr, 
		Data => ram_in,
		Q => ram_out);

-- The Program Memory is a ROM that holds the CPU program. 
	Program_Memory : entity ROM port map (
		Address => prog_cntr,
        OutClock => not CK,
        OutClockEn => '1',
        Reset => '0',	
        Q => prog_data);

-- The OSR8 processor, configured for this application by its 
-- generic map.
	CPU : entity OSR8_CPU 
		generic map (
			prog_cntr_len => prog_cntr_len,
			cpu_addr_len => cpu_addr_len,
			start_pc => start_pc,
			interrupt_pc => interrupt_pc
		)
		port map (
			prog_data => prog_data,
			prog_cntr => prog_cntr,
			cpu_data_out => cpu_data_out,
			cpu_data_in => cpu_data_in,
			cpu_addr => cpu_addr,
			WR => CPUWR,
			DS => CPUDS,
			IRQ => CPUIRQ,
			ISRV => CPUISRV,
			SIG => CPUSIG,
			RESET => RESET,
			CK => CK
		);
		
-- The Memory Manager maps eight-bit read and write access to the Sensor 
-- Controller, Sample Transmitter, Random Access Memory, and Interrupt 
-- Handler. Byte ordering is big-endian (most significant byte at lower 
-- address). 
	MMU : process (all) is
		variable all_bits : integer range 0 to 2048;
		variable bottom_bits : integer range 0 to 31;
	begin		
		-- By default, don't write to RAM or PROG memories, nor do we read from
		-- the command memory FIFO.
		RAMWR <= '0';
		CMRD <= '0';
		
		-- The RAM address we take from the lower bits of the cpu
		-- address. The RAM data in is always the cpu data out.
		ram_in <= cpu_data_out;
		ram_addr <= cpu_addr(ram_addr_len-1 downto 0);
		
		-- These signals develop after the CPU asserts a new address
		-- along with CPU Write. They will be ready before the falling 
		-- edge of the CPU clock.
		all_bits := to_integer(unsigned(cpu_addr));
		bottom_bits := to_integer(unsigned(cpu_addr(5 downto 0)));
		
		-- Combinatorial memory map, which serves all access except 
		-- for writing to control registers. We have the control
		-- space reads, which are either supplied by registers or
		-- by shadow locations in RAM. We have the RAM itself, and
		-- the program memory as well.
		cpu_data_in <= (others => '0');
		case all_bits is
			when ctrl_bot to ctrl_top =>
				if not CPUWR then 
					case bottom_bits is
					
						-- The interrupt request bits, used by the CPU interrupt
						-- routine, of which there can by only one, to determine
						-- which of the interrupt sources requires servicing.						
						when mmu_irqb => cpu_data_in <= int_bits;
						
						-- The status register, which gives access to an array of
						-- flags that signal the state of the peripheral logic.
						-- and clocks, including the 32.768 kHz and 1.024 kHz 
						-- clocks, which the CPU can use for timing.
						when mmu_sr => 
							cpu_data_in(0) <= to_std_logic(CMDRDY); -- Command Ready
							cpu_data_in(1) <= to_std_logic(RPS);    -- Receive Power
							cpu_data_in(2) <= MCK;                  -- Millisecond Clock
							cpu_data_in(3) <= to_std_logic(TXA);    -- Transmit Active
							cpu_data_in(4) <= to_std_logic(CPA);    -- Command Processor Active 
							cpu_data_in(5) <= to_std_logic(SCBSY);  -- Sample Controller Busy
							cpu_data_in(6) <= CME;                  -- Command Memory Empty
							cpu_data_in(7) <= RCK;                  -- Reference Clock
							
						-- The location from which the CPU reads command bytes out
						-- of the command FIFO. Includes a strobe that increments the
						-- FIFO after the read.
						when mmu_cmp =>
							cpu_data_in <= cmd_out;
							CMRD <= to_std_logic(CPUDS);
							
						-- The location from which the CPU reads the byte most 
						-- recently received from the I2C interface.
						when mmu_i2cMR => cpu_data_in <= i2c_in;
						
						-- The two locations that provide the output of the sample 
						-- memory, which holds accumulated samples. We read the
						-- top sixteen bits of its eighteen-bit output.
						when mmu_smemh => cpu_data_in <= smem_data(17 downto 10);							
						when mmu_smeml => cpu_data_in <= smem_data(9 downto 2);
						
						-- Whenever we read any other location, we get the value 
						-- previously written to the shadow RAM.
						when others => cpu_data_in <= ram_out;
					end case;
				else 
					-- When we write to any control space register, record the 
					-- written value in the shadow RAM.
					RAMWR <= to_std_logic(CPUDS);
				end if;
			when ram_bot to ram_top => 
				-- The variable and stack RAM space, writing and reading.
				if not CPUWR then
					cpu_data_in <= ram_out;
				else
					RAMWR <= to_std_logic(CPUDS);
				end if;
			when others =>
				null;
		end case;
		
		-- Here is the memory map for control register write access. We use RESET 
		-- to clear some registers and signals, but not all. We do not clear the
		-- software reset signal, SWRST, on RESET, since we want SWRST to assert 
		-- RESET for one CK period. After a reset, the cpu address will not select 
		-- the SWRST location, so SWRST will be cleared on the next falling edge 
		-- of CK. All writes to control space are shadowed by RAM locations so that
		-- we can be sure to read them back.
		if (RESET = '1') then
			TXI <= false;
			TXWP <= false;
			FCKENCPU <= false;
			BOOST <= false;
			tx_channel <= tx_channel_default;
			df_reg <= (others => '0');
			int_mask <= (others => '0');
			int_rst <= (others => '1');
			int_period_0 <= (others => '0');
			CPRST <= true;
			frequency_low <= default_frequency_low;
			fck_mask <= default_fck_mask;
			SDA <= 'Z';
			SCL <= '1';
			MSR <= '0';
			DC <= '0';
			ACCRST <= '1';
			SMWRCPU	<= '1';
			X4SS <= false;
			
		-- We use the falling edge of RCK to write to registers and to initiate sensor 
		-- and transmit activity. Some signals we assert only for one CK period, and 
		-- these we assert as false by default.
		elsif falling_edge(CK) then
			CPRST <= false;
			SWRST <= false;
			TXI <= false;
			ACCRST <= '0';
			SMWRCPU <= '0';
			int_rst <= (others => '0');
			if CPUDS and CPUWR then 
				if (all_bits >= ctrl_bot) and (all_bits <= ctrl_top) then
					case bottom_bits is
						-- The Amplifier Configuration register selects between
						-- AC and DC coupling for the unipolar inputs with bit
						-- zero. With bit one it activates the impedance 
						-- measurement switch, which displaces the ground
						-- potential of unipolar and bipolar inputs so as
						-- to introduce a step downwards in the unipolar 
						-- inputs, the size of which depends upon the ratio
						-- of the electrode impedance to the amplifier input
						-- impedance.
						when mmu_acfg => 
							DC <= cpu_data_out(0);
							MSR <= cpu_data_out(1);
						
						-- The two locations in which the CPU places the sixteen
						-- telemetry sample bits that will be transmitted after the
						-- next write to the transmission control register.
						when mmu_xlb  => xmit_bits(7 downto 0) <= cpu_data_out;
						when mmu_xhb  => xmit_bits(15 downto 8) <= cpu_data_out;
						
						-- The telemetry channel number for the next transmission.
						when mmu_xch  => tx_channel <= to_integer(unsigned(cpu_data_out));
						
						-- The telemetry control register. The TXI initiates a transmission,
						-- while TXWP turns on the VCO to warm it up before it transmits, 
						-- which is necessary if the VCO has been dormant for more than 
						-- 20 ms. We must turn off the warm-up by writing a zero to TXWP
						-- after a warm-up of no more than 20 ms so we do not collide with
						-- other transmitters.
						when mmu_xcr  => 
							TXI <= (cpu_data_out(0) = '1');
							TXWP <= (cpu_data_out(1) = '1');
							
						-- The frequency calibration of the VCO. We write the VCO control
						-- five-bit DAC value that sets the frequency of a transmit zero.
						when mmu_rfc  => frequency_low <= to_integer(unsigned(cpu_data_out));
						
						-- The interrupt mask. Bits set to one enable their corresponding
						-- interrupt request bits and any peripheral machines that generate
						-- interrupts.
						when mmu_imsk => int_mask <= cpu_data_out;
						
						-- The interrupt-0 period. In this case we set a four-bit period
						-- between 0 and 31 to indicate the transmit interrupt period in
						-- multiples of 1/1024 s. When the period is zero, the interrupt
						-- will take place every 32 periods, or at 32 Hz.
						when mmu_i0p  => int_period_0 <= cpu_data_out(4 downto 0);

						-- The interrupt reset register. Bits set to one reset their
						-- corresponding interrupt request bits, signalling that the
						-- interrupt has been serviced.
						when mmu_irst => int_rst <= cpu_data_out;
						
						-- Turn on the indicator lamp.
						when mmu_led  => LED <= cpu_data_out(0);
						
						-- A strobe that reboots the firmware and CPU.
						when mmu_rst  => SWRST <= (cpu_data_out(0) = '1');
						
						-- Control bits to turn on the fast clock, which in turn
						-- produces the transmit clock, and to move the CPU into
						-- boost mode. These bits are in addition to the automatic
						-- enable of the fast clock and move into boost that is 
						-- performed by the Boost Controller when it sees the
						-- CPU's interrupt service flag has been set by the CPU
						-- to indicate that it is servicing an interrupt.
						when mmu_ccr  => 
							FCKENCPU <= (cpu_data_out(0) = '1');
							BOOST <= (cpu_data_out(1) = '1');
							
						-- The diagnostic flag register. These bits can be routed
						-- to test points for diagnostics with an oscilloscope.
						when mmu_dfr  => df_reg <= cpu_data_out(3 downto 0);
						
						-- The command processor reset bit: clears the command
						-- FIFO and returns the command processor to its rest state.
						when mmu_cpr  => CPRST <= true;
						
						-- The fast clock mask has only one bit out of eight set,
						-- and this bit selects which gate in the ring will be
						-- used for feedback, thus setting the ring frequency.
						when mmu_fck => fck_mask <= cpu_data_out;
						
						-- The I2C bit-banging interface. Each write to one of these
						-- locations sets both SDA and SCL to one of zero, one, or
						-- high-impedance. We operate the I2C by writing register A
						-- to these locations. Some locations take the top bit of A
						-- and, when this bit is zero, drive SDA low. Others cause
						-- the value of SDA to be shifted into the i2C byte register,
						-- from which it can later be read out. With the CPU in boost
						-- mode, the interface runs at 500 kHz. See the I2C assembler
						-- routine for more details.
						when mmu_i2c00 => 
							SDA <= '0';
							SCL <= '0';
						when mmu_i2c01 =>
							SDA <= '0';
							SCL <= '1';
						when mmu_i2cA0 => 
							if (cpu_data_out(7) = '0') then
								SDA <= '0';
							else
								SDA <= 'Z';
							end if;
							SCL <= '0';
						when mmu_i2cA1 =>
							if (cpu_data_out(7) = '0') then
								SDA <= '0';
							else
								SDA <= 'Z';
							end if;
							SCL <= '1';
						when mmu_i2cZ0 => 
							SDA <= 'Z';
							SCL <= '0';
						when mmu_i2cZ1 => 
							SDA <= 'Z';
							SCL <= '1';
							i2c_in(7 downto 1) <= i2c_in(6 downto 0);
							i2c_in(0) <= SDA;
							
						-- The sample select control register begins with two
						-- bits to select which accumulated sample the CPU
						-- will get when it reads the sample memory HI and
						-- LO bytes. The sample memory will accept this 
						-- sample select value provided that ADCBSY is not
						-- asserted. The ACCRST and SMWRCPU strobes reset the
						-- accumulator output and write the accumulator output
						-- to the selected accumulated sample.
						when mmu_smcr =>
							sample_sel <= cpu_data_out(1 downto 0);
							ACCRST <= cpu_data_out(2);
							SMWRCPU <= cpu_data_out(3);
							
						-- We have four locations to describe how each input
						-- is to be sampled.
						when mmu_x1cfg =>
							x1_shift <= cpu_data_out(2 downto 0);
						when mmu_x2cfg =>
							x2_shift <= cpu_data_out(2 downto 0);
						when mmu_x3cfg =>
							x3_shift <= cpu_data_out(2 downto 0);
						when mmu_x4cfg =>
							x4_shift <= cpu_data_out(2 downto 0);
							X4SS <= (cpu_data_out(3) = '1');

						-- For all other addresses, we have no bits to set.
						-- he shadow RAM is recording all writes to these 
						-- addresses, and will respond to reads for which 
						-- no readable register is implemented.
						when others => null;
					end case;
				end if;
			end if;
		end if;
	end process;
	
-- The Boost Controller switches the CPU between RCK and a 5-MHz it
-- generates using FCK. We call this 5-MHz clock the Transmit Clock 
-- (TCK). When the CPU's clock CK is set to TCK, we are in "boost" mode. 
-- When CK = RCK we are in "slow" mode. Switching to boost is easy
-- because we know the state of RCK when we want to switch into boost. 
-- Either the CPU just asserted BOOST with a regiseter write, or it 
-- just asserted Interrupt Service (ISRV). Both occur on the falling 
-- edge of CK, so RCK  will be LO for at least 15 us. We come out of 
-- boost when both BOOST and ISRV are un-asserted. We use signal 
-- RCKLO to coordinate the transition from TCK to RCK. We will perform
-- this transition only when both TCK and RCK are LO and guaranteed
-- to remain LO for at least two FCK cycles. We care about the value of 
-- RCKLO only when FCK is running and BOOST has been unasserted. We
-- assert RCKLO after each falling edge of RCK for a one hundred FCK
-- cycles, which will be 10 us if FCK is exactly 10 MHz, but longer
-- if FCK is running slower, as it will before calibration. We assume
-- that FCK will always be at least 7 MHz so that RCKLO will never
-- exceed 15 us and remain asserted during the next rising edge of RCK.
-- During transitions between boost and slow modes, TCK will skip some
-- cycles. We must refrain from moving in and out of boost while some
-- other process is relying on TCK to be sustained. For example, we must 
-- not boost or un-boost during a telemetry sample transmission.
	Boost_Controller : process (RESET, FCK) is
	variable state, next_state : integer range 0 to 3;
	constant end_count : integer := 100;
	variable counter : integer range 0 to 127;
	begin
	
		-- Generate KEEPFCK, RCKLO, and RCKHI.
		if (RESET = '1') then
			SRCK <= '0';
			SSRCK <= '0';
			counter := 0;
			RCKLO <= false;
			KEEPFCK <= false;
		elsif rising_edge(FCK) then
			KEEPFCK <= (state /= 0);
			SRCK <= RCK;
			SSRCK <= SRCK;
			
			if not KEEPFCK then
				RCKLO <= true;
			elsif (SRCK = '0') and (SSRCK = '1') then
				RCKLO <= true;
			elsif (counter = end_count) then 
				RCKLO <= false;
			end if;
			
			if (not RCKLO) or (not KEEPFCK) then
				counter := 0;
			else
				counter := counter + 1;
			end if;
		end if;
		
		-- Manage transition from slow to boost and back to slow.
		if RESET = '1' then
			state := 0;
			TCK <= '0';
		elsif falling_edge(FCK) then
			case state is
				when 0 =>
					if BOOST or CPUISRV then 
						TCK <= '0';
						next_state := 1;
					else
						TCK <= to_std_logic(TCK = '0');
						next_state := 0;
					end if;
				when 1 => 
					next_state := 3;
					TCK <= '0';
				when 3 =>
					if (not BOOST) and (not CPUISRV) then
						TCK <= '0';
						next_state := 2;
					else
						TCK <= to_std_logic(TCK = '0');
						next_state := 3;
					end if;
				when 2 => 
					if RCKLO then
						next_state := 0;
					else
						next_state := 2;
					end if;
					TCK <= '0';
			end case;
			state := next_state;
		end if;
		
		-- The clock selector: boost or slow according to state.
		CK <= to_std_logic(
			((RCK = '1') and (state = 0)) 
			or ((TCK = '1') and (state = 3)));
	end process;

-- The Interrupt Controller provides the interrupt signal to the CPU in response to
-- timer events. By default, at power-up, all interrupts are masked. We can set the
-- period of each timer by writing to locations in the CPU control space. If we want
-- the counter to have period N ticks, we write value N-1 to the period registers.
	Interrupt_Controller : process (RCK, int_rst) is
	begin
	
		-- Synchronize the int_rst bits with RCK. Our interrupt counters run off RCK, 
		-- and the interrupt lines themselves are generated by RCK, but the interrupt
		-- reset signals are asserted by the CPU during interrupt servicing, for which
		-- the CPU is in boost mode, running of the 5-MHz TCK. Our registers can handle 
		-- asynchronous assertion of reset, but not asynchronous unassertion of reset. 
		-- We must make the unassertion synchronous with the clock that drives the register.
	
		-- Interrupt One Reset
		if int_rst(0) = '1' then
			int_rst_d(0)  <= '1';
			int_rst_s(0) <= '1';
		elsif falling_edge(RCK) then
			int_rst_d(0)  <= '0';
			int_rst_s(0) <= int_rst_d(0);
		end if;
		
		-- The interrupt timers run all the time, counting down from their period value
		-- to zero. The only way to stop them counting is to set their period value to
		-- zero so that they stick at zero. Just disabling the interrupt does not stop 
		-- the counter.

		-- Interrupt One: Set by a rising edge on TMINT from the Telemetry Manager.
		if rising_edge(RCK) then
			TMINTD <= TMINT;
		end if;
		
		-- Using the interrupt reset bits that we synchronized with RCK, we 
		-- reset the interrupt bits. Otherwise, we set the interrupt bits
		-- either with some external flag or when a timer reaches zero.
				
		-- Interrupt One Control
		if (int_rst_s(0) = '1') then
			int_bits(0) <= '0';
		elsif rising_edge(RCK) then
			if (TMINT and (not TMINTD)) then
				int_bits(0) <= '1';
			end if;
		end if;
		
		-- We disable the remaining interrupt lines.
		for i in 1 to 7 loop
			int_bits(i) <= '0';
		end loop;		
	end process;

-- The Interrupt Generator takes the interrupt bits and the interrupt mask
-- and combines them to create an interrupt reques for the CPU, which we 
-- synchronize with CK.
	Interrupt_Generator : process (CK) is
	begin
		if RESET = '1' then
			CPUIRQ <= false;
		elsif falling_edge(CK) then
			CPUIRQ <= (int_bits and int_mask) /= "00000000";
		end if;
	end process;
	
-- The Telemetry Manager organizes sampling of the inputs X1-X4 a
-- frequency that is an integer fraction of 1024 SPS and generates
-- the transmit interrupt offset from sampling by a random number
-- of RCK periods. The manager runs off RCK, but turns on FCK for
-- the Sample Controller.
	Telemetry_Manager : process (RCK) is 
		constant tm_max : integer := 31;
		constant tm_idle : integer := tm_max;
		constant tm_update : integer := tm_max - 1;
		constant tm_sample : integer := tm_max - 2;
		variable state, next_state : integer range 0 to tm_max := 0;
		variable TMRUN : boolean;
		variable tx_index : integer range 0 to 31;
		
	begin
		if RESET = '1' then
			SCRUN <= false;
			TMINT <= false;
			TMRUN := false;
			state := tm_idle;
		elsif rising_edge(RCK) then
			TMRUN := (int_mask(0) = '1');
			
			if not TMRUN then
				next_state := tm_idle;
			else
				if state = tm_idle then
					next_state := 0;
				else
					next_state := state + 1;
				end if;
			end if;
			
			if (state = to_integer(unsigned(transmit_shift))) then
				if tx_index = 1 then
					TMINT <= true;
				end if;
			elsif (state = tm_sample) then
				SCRUN <= true;
				TMINT <= false;
			elsif (state = tm_update) then
				SCRUN <= false;
				TMINT <= false;
				if tx_index = 1 then
					tx_index := to_integer(unsigned(int_period_0));
				else
					tx_index := tx_index - 1;
				end if;
			elsif (state = tm_idle) then
				if not TMRUN then
					tx_index := to_integer(unsigned(int_period_0));
				end if;
				SCRUN <= false;
				TMINT <= false;
			else
				SCRUN <= false;
				TMINT <= false;
			end if;
			
			state := next_state;
		end if;
		
		if SCDONE then
			FCKENTM <= false;
		elsif rising_edge(RCK) then
			if (state = tm_sample) then
				FCKENTM <= true;
			else
				FCKENTM <= false;
			end if;
		end if;
	end process;

-- The Sample Memory is fourteen bits wide so as to accommodate
-- our fourteen-bit ADC samples. The sample memory runs off 
-- the fast clock to work well with the Sample Controller, which
-- also runs on fast clock.
	Sample_Memory : entity SMRAM port map (
		Clock => not FCK,
		ClockEn => '1',
        Reset => '0',
		WE => SMWRADC or SMWRCPU,
		Address => smem_addr, 
		Data => acc_data,
		Q => smem_data);
	
-- The Sample Memory Address Multiplexer sets the sample memory 
-- address equal to adc_sel when the ADC Controller is busy, and
-- sample_sel otherwise. The former is incremented by the Sample 
-- Controller as it steps through the ADCs performing samples. The
-- latter is set by the CPU with a write to the sample memory
-- address register.
	Sample_Addr_Mux : process (FCK) is 
	begin
		if rising_edge(FCK) then
			if SCBSY then
				smem_addr <= adc_sel;
			else
				smem_addr <= sample_sel;
			end if;
		end if;
	end process;
		
-- The Sample Accumulator adds fourteen-bit samples from the Sample 
-- Memory together so as to produce an eighteen-bit value from which
-- we will read the top sixteen bits as our sample for transmission.
	Sample_Accumulator : entity SMADD port map (
		DataA => adc_data,
		DataB => smem_data,
		Result => acc_data,
		Clock => not FCK,
		ClockEn => ACCADD,
		Reset => ACCRST
		);

-- The Sample Controller organizes sampling of all four inputs X1-X4 using
-- the four converters ADC1-ADC4. It runs off FCK. It starts running when
-- it sees the ACRUN flag set. It will not return to its idle state until
-- this flag is cleared. It obtains a sample from each ADC for which the
-- period index is equal to one. The Sample Controller does not change 
-- the period index. That task is left to the Telemetry Controller. Any
-- input for which the index is not one will be skipped over. The Sample
-- Controller sets the adc_sel and adc_shift arrays to make sure that
-- the sample is shifted the correct number of places to the left and then
-- stored in the correct location in the sample memory. For the X4 channel,
-- the controller will 
	Sample_Controller : process (FCK) is
	variable state, next_state : integer range 0 to 15 := 0;
	constant sc_idle : integer := 0;
	constant x1_start : integer := 1;
	constant x1_wait : integer := 2;
	constant x2_start : integer := 3;
	constant x2_wait : integer := 4;
	constant x3_start : integer := 5;
	constant x3_wait : integer := 6;
	constant x4_start : integer := 7;
	constant x4_wait : integer := 8;
	constant sc_wait : integer:= 9;
	variable X4EN : boolean;
	
	begin
		if not SCRUN then
			ADCRD <= false;
			SCBSY <= false;
			SCDONE <= false;
			adc_sel <= x1_sel;
			state := sc_idle;
		elsif falling_edge(FCK) then
			if (state = sc_idle) then 
				adc_sel <= x1_sel;
				adc_shift <= x1_shift;
				ADCRD <= false;
				next_state := x1_start;
			elsif (state = x1_start) then
				adc_sel <= x1_sel;
				adc_shift <= x1_shift;
				ADCRD <= true;
				if ADCBSY then
					next_state := x1_wait;	
				else
					next_state := x1_start;
				end if;
			elsif (state = x1_wait) then
				adc_sel <= x1_sel;
				adc_shift <= x1_shift;
				ADCRD <= false;
				if ADCBSY then
					next_state := x1_wait;
				else
					next_state := x2_start;
				end if;
			elsif (state = x2_start) then
				adc_sel <= x2_sel;
				adc_shift <= x2_shift;
				ADCRD <= true;
				if ADCBSY then
					next_state := x2_wait;	
				else
					next_state := x2_start;
				end if;
			elsif (state = x2_wait) then
				adc_sel <= x2_sel;
				adc_shift <= x2_shift;
				ADCRD <= false;
				if ADCBSY then
					next_state := x2_wait;
				else
					next_state := x3_start;
				end if;
			elsif (state = x3_start) then
				adc_sel <= x3_sel;
				adc_shift <= x3_shift;
				ADCRD <= true;
				if ADCBSY then
					next_state := x3_wait;	
				else
					next_state := x3_start;
				end if;
			elsif (state = x3_wait) then
				adc_sel <= x3_sel;
				adc_shift <= x3_shift;
				ADCRD <= false;
				if ADCBSY then
					next_state := x3_wait;
				else
					next_state := x4_start;
				end if;
			elsif (state = x4_start) then
				adc_sel <= x4_sel;
				adc_shift <= x4_shift;
				if X4EN then
					ADCRD <= true;
					if ADCBSY then
						next_state := x4_wait;	
					else
						next_state := x4_start;
					end if;
				else 
					ADCRD <= false;
					next_state := sc_wait;
				end if;
			elsif (state = x4_wait) then
				adc_sel <= x4_sel;
				adc_shift <= x4_shift;
				ADCRD <= false;
				if ADCBSY then
					next_state := x4_wait;
				else
					next_state := sc_wait;
				end if;
			elsif (state = sc_wait) then
				adc_sel <= x4_sel;
				ADCRD <= false;
				adc_shift <= x4_shift;
				next_state := sc_wait;
			else 
				next_state := sc_wait;
			end if;
			
			state := next_state;
			SCBSY <= (state > sc_idle) and (state < sc_wait);
			SCDONE <= (state = sc_wait);
		end if;
		
		if RESET = '1' then
			ADCCAL <= true;
		elsif rising_edge(FCK) then
			if (state = sc_wait) then
				ADCCAL <= false;
			end if;
		end if;
		
		if RESET = '1' then
			X4EN := true;
		elsif rising_edge(FCK) then
			if (ACCRST = '1') and (sample_sel = x4_sel) then
				X4EN := true;
			elsif (state = x4_wait) and X4SS then
				X4EN := false;
			end if;
		end if;
	end process;

-- ADC Controller starts reading one of the 14-bit ADCs when it detects 
-- ADC Read (ADCRD). It waits until ADCRD is unasserted before returning
-- to its rest state. When the controller has acquired the 14-bit sample,
-- it updates the transmit time shift value using a linear shift 
-- register that will, if the ADC is working properly, generate a random
-- number for transmit scatter. After that, the controller shifts 14-bit 
-- samples to the left from zero to four place, as directed by adc_shift. 
-- After shifting, the controller adds the sample to the accumulator, and 
-- writes the output of the accumulator to the sample memory. The ADC it 
-- selects for readout is the one specified by adc_sel. If ADCRD is 
-- accompanied by ADCCAL, the ADC Controller performs a calibration read 
-- of twenty-four bits, with no shifting, no accumulation, and no storage. 
-- This calibration read will cause the ADC to self-calibrate provided that 
-- the calibration access is the first one after power-up. 
	ADC_Controller : process (RESET, FCK) is
		variable state, next_state : integer range 0 to 63 := 0;
		constant calib_end : integer := 50;
		constant read_end : integer := 40;
		
 	begin
		-- Upon startup, we make sure we are in the idle state.
		if (RESET = '1') then 
			state := 0;
			ADCBSY <= false;
			ACCADD <= '0';
			SMWRADC <= '0';
			adc_data <= (others => '0');
			transmit_shift <= (others => '1');
			
		-- The ADC Contoller proceeds through states so as initiate a conversion,
		-- read out one zero, fourteen data bits, and three trailing zeros. If
		-- the ADCCAL flag is set, it reads out nine trailing zeros so as to 
		-- initiate an ADC self-calibration. When ADCCAL is not set, the fourteen 
		-- data bits are shifted into the adc_data register. Once we have them, we
		-- write the sample to the sample memory.
		elsif rising_edge(FCK) then
			if (state = 0) then 
				if ADCRD then 
					next_state := 1;
				else 
					next_state := 0;
				end if;
			end if;
			if ADCCAL then
				if (state > 0) and (state < calib_end) then
					next_state := state + 1;
				end if;
				if (state = calib_end) then
					next_state := 0;
				end if;
				SCK <= to_std_logic(
					(state = 0) 
					or ((state < 48) and (state mod 2) = 1));
				adc_data <= (others => '0');
			else 
				if (state > 0) and (state < read_end) then
					next_state := state + 1;
				end if;
				if (state = read_end) then
					next_state := 0;
				end if;
				SCK <= to_std_logic(
					(state = 0) 
					or ((state < 36) and ((state mod 2) = 1))
				);
				if (state = 1) then
					adc_data <= (others => '0');
				end if;
				if (state >= 4) and (state <= 30) and ((state mod 2) = 0) then
					adc_data(17 downto 1) <= adc_data(16 downto 0);
					adc_data(0) <= SDO;
				end if;
				if ((state = 32) and (unsigned(adc_shift) >= 1))
					or ((state = 34) and (unsigned(adc_shift) >= 2)) 
					or ((state = 36) and (unsigned(adc_shift) >= 3))
					or ((state = 38) and (unsigned(adc_shift) >= 4)) then
					adc_data(17 downto 1) <= adc_data(16 downto 0);
					adc_data(0) <= '0';
				end if;
			end if;
			
			if (state = 31) then
				transmit_shift(3) <= adc_data(0) xor transmit_shift(0);
				transmit_shift(2) <= transmit_shift(3) xor transmit_shift(0);
				transmit_shift(1) <= transmit_shift(2);
				transmit_shift(0) <= transmit_shift(1);
			end if;
			
			ACCADD <= to_std_logic((state = read_end - 1) and (not ADCCAL));
			SMWRADC <= to_std_logic((state = read_end) and (not ADCCAL));
			ADCBSY <= ((not ADCCAL) and (state > 0) and (state < read_end))
				or (ADCCAL and (state > 0) and (state < calib_end));

			state := next_state;
		end if;
		
		-- The ADC we read out or calibrate is selected by the top two bits 
		-- of the sample address.
		NADC1 <= to_std_logic(not (ADCBSY and (adc_sel = x1_sel)));
		NADC2 <= to_std_logic(not (ADCBSY and (adc_sel = x2_sel)));
		NADC3 <= to_std_logic(not (ADCBSY and (adc_sel = x3_sel)));
		NADC4 <= to_std_logic(not (ADCBSY and (adc_sel = x4_sel)));
	end process;

-- The Message Transmitter responds to Transmit Initiate (TXI) by turning on the 
-- radio-frequency oscillator, reading sixteen bits from one of the sensors and
-- transmitting the bits. The process runs off TCK, so the CPU must assert ENFCK
-- for the process to run. The TXI signal will be asserted for one period of CK
-- following a CPU write to the TXI location. Further writes to the same location
-- will be ignored until the Message Transmitter returns to its idle state.
	Message_Transmitter : process (RESET, TCK) is
		variable channel_num, set_num, completion_code : 
			integer range 0 to 15; -- set number for data
		constant num_sync_bits : integer := 11; -- Num synchronizing bits at start.
		constant num_id_bits : integer := 4; -- Number of ID bits.
		constant num_start_bits : integer := 1; -- Num zero start bits.
		constant num_stop_bits : integer := 2; -- For state machine termination only.
		constant num_data_bits : integer := 16; -- Number of ADC data bits.
		constant num_xmit_bits : integer := -- Number of transmission bit periods.
			num_sync_bits + num_start_bits + num_id_bits + num_data_bits + num_id_bits; 
		constant st_idle : integer := 0; -- Idle state value.
		constant first_sync_bit : integer := 1; -- First transmit state.
		constant first_start_bit : integer := first_sync_bit + num_sync_bits;
		constant first_id_bit : integer := first_start_bit + num_start_bits;
		constant first_data_bit : integer := first_id_bit + num_id_bits;
		constant first_cc_bit : integer := first_data_bit + num_data_bits;
		constant st_done : integer := -- Final state of sample transmit machine.
			num_xmit_bits + num_stop_bits; 
		variable channel_bits : std_logic_vector(3 downto 0);
		variable cc_bits : std_logic_vector(3 downto 0);
		variable state, next_state : integer range 0 to 63; -- Stample Transmit State
		
	begin
		-- The channel number, set number, and comletion code are a function of the 
		-- device id and the channel offset, which we calculate here.
		channel_num := tx_channel mod 16;
		set_num := tx_channel / 16;
		completion_code := 15 - channel_num + set_num;
		channel_bits := std_logic_vector(to_unsigned(channel_num,4));
		cc_bits := std_logic_vector(to_unsigned(completion_code,4));
		
		-- Upon startup, we make sure we are in the idle state.
		if (RESET = '1') then 
			state := 0;
			
		elsif rising_edge(TCK) then
			-- The process starts when we assert TXI. We move through all subsequen
			-- states until we reach the final state, where we wait until TXI is
			-- un-asserted for our return to the idle state. 
			case state is
				when st_idle => 
					if TXI then
						next_state := 1;
					else
						next_state := 0;
					end if;
				
				when st_done =>
					if not TXI then
						next_state := st_idle;
					else
						next_state := st_done;
					end if;
				
				when others =>
					next_state := state + 1;
			end case;
		
			-- The data bit is the outgoing bit value for transmission of the sensor signal.
			TXB <= ((state >= 0) and (state < first_start_bit))
				or ((state = first_id_bit + 0) and (channel_bits(3) = '1'))
				or ((state = first_id_bit + 1) and (channel_bits(2) = '1'))
				or ((state = first_id_bit + 2) and (channel_bits(1) = '1'))
				or ((state = first_id_bit + 3) and (channel_bits(0) = '1'))
				or ((state = first_data_bit) and (xmit_bits(15) = '1'))
				or ((state = first_data_bit + 1) and (xmit_bits(14) = '1'))
				or ((state = first_data_bit + 2) and (xmit_bits(13) = '1'))
				or ((state = first_data_bit + 3) and (xmit_bits(12) = '1'))
				or ((state = first_data_bit + 4) and (xmit_bits(11) = '1'))
				or ((state = first_data_bit + 5) and (xmit_bits(10) = '1'))
				or ((state = first_data_bit + 6) and (xmit_bits(9) = '1'))
				or ((state = first_data_bit + 7) and (xmit_bits(8) = '1'))
				or ((state = first_data_bit + 8) and (xmit_bits(7) = '1'))
				or ((state = first_data_bit + 9) and (xmit_bits(6) = '1'))
				or ((state = first_data_bit + 10) and (xmit_bits(5) = '1'))
				or ((state = first_data_bit + 11) and (xmit_bits(4) = '1'))
				or ((state = first_data_bit + 12) and (xmit_bits(3) = '1'))
				or ((state = first_data_bit + 13) and (xmit_bits(2) = '1'))
				or ((state = first_data_bit + 14) and (xmit_bits(1) = '1'))
				or ((state = first_data_bit + 15) and (xmit_bits(0) = '1'))
				or ((state = first_cc_bit + 0) and (cc_bits(3) = '1'))
				or ((state = first_cc_bit + 1) and (cc_bits(2) = '1'))
				or ((state = first_cc_bit + 2) and (cc_bits(1) = '1'))
				or ((state = first_cc_bit + 3) and (cc_bits(0) = '1'));
				
			-- TXA indicates that a transmission is on-going.
			TXA <= (state /= st_idle) and (state /= st_done);
			
			-- Assert the next state value.
			state := next_state;
		end if;
	end process;

-- With XEN we enable the VCO. We assert XEN while the Message Transmitter is active,
-- provided that the Command Processor is not receiving a command. We also turn on
-- the VCO when the CPU asserts Transmit Warmup (TXWP). 
	XEN <= to_std_logic((TXA or TXWP) and (CMDRDY or (not CPA)));
			
-- The Frequency Modulation process takes the transmit bit values provided by
-- the Message Transmitter, turns them into a sequence of rising and falling
-- edges so as to balance the ratio of HI and LO, and modulates the transmit DAC
-- output (xdac) between the HI and LO frequency values. These values are turned
-- into analog voltages on the TUNE input of the radio frequency oscillator, and
-- so modulate the frequency of the transmission.
	Frequency_Modulation : process (RESET, FCK) is
	begin
		if RESET = '1' then
			xdac <= (others => '0');
			FHI <= false;
			
		-- Frequency modulation runs off the 10-MHz FCK clock. This clock is
		-- synchronous with TCK. It presents a rising edge over 10 ns after 
		-- both the rising and falling edges of TCK. Thus, when we see a
		-- rising edge on FCK, the value of TCK and TXB are both established.
		elsif rising_edge(FCK) then
		
			-- When we are not transmitting RF power, we set the DAC output to
			-- zero so as to eliminate current consumption by the DAC resistors.
			if not TXA then
				xdac <= (others => '0');
				FHI <= false;
				
			-- If TXB is asserted, we want the modulation frequency to go from low
			-- to high on the falling edge of TCK. When TXB is unasserted, we want
			-- the modulation frequency to go from high to low on the falling edge of
			-- TCK.
			elsif (TXB xor (TCK = '1')) then
				xdac <= std_logic_vector(to_unsigned(frequency_low + frequency_step,5));
				FHI <= true;
			else
				xdac <= std_logic_vector(to_unsigned(frequency_low,5));
				FHI <= false;
			end if;
		end if;
	end process;

-- The Receive Power signal must be synchronized with the RCK clock.
	Synchronize_RP: process (RESET, RCK) is 
	begin
		if RESET = '1' then
			RPS <= false;
		elsif rising_edge(RCK) then
			RPS <= (RP = '1');
		end if;
	end process;
	
-- We detect a long enough burst of command power to initiate
-- command reception, and set the ICMD signal.
	Initiate_Command: process (RESET, RCK) is 
		constant endcount : integer := 2;
		variable counter : integer range 0 to 3;
	begin
		if (not RPS) then
			ICMD <= false;
			counter := 0;
		elsif rising_edge(MCK) then
			if (counter = endcount) then 
				counter := endcount;
				ICMD <= true;
			else 
				counter := counter + 1;
				ICMD <= false;
			end if;
		end if;
	end process;
	
-- We detect a long enough period without command power to 
-- terminate command reception, and set the TCMD signal.
	Terminate_Command: process (RESET, RCK) is 
		constant endcount : integer := 7;
		variable counter : integer range 0 to 7;
	begin
		if RPS then
			counter := 0;
			TCMD <= false;
		elsif rising_edge(MCK) then
			if (counter = endcount) then 
				counter := endcount;
				TCMD <= true;
			else 
				counter := counter + 1;
				TCMD <= false;
			end if;
		end if;
	end process;
	
-- The Receive Command (RCMD) signal indicates that a command is being 
-- received. We set RCMD when Initiate Command (ICMD) occurs, and we clear
-- RCMD when Terminate Command (TCMD) occurs.
	Receive_Command: process (RESET, RCK) is
	begin
		if RESET = '1' then
			RCMD <= false;
		elsif rising_edge(RCK) then
			if not RCMD then
				RCMD <= ICMD;
			else 
				RCMD <= not TCMD;
			end if;
		end if;
	end process;

-- We watch for a start bit and receive serial bytes when instructed
-- to do so by the Command Processor with the RBI signal.
	Byte_Receiver: process (RESET, RCK) is
		variable state, next_state : integer range 0 to 63;
		variable no_stop_bit : boolean;
	begin
		if RESET = '1' then
			state := 0;
		elsif rising_edge(RCK) then
		
			-- Idle state, waiting for Receive Byte Initiate.
			if (state = 0) then
				if RBI and (not RPS) then 
					next_state := 1;
				else 
					next_state := 0;
				end if;
			end if;
			
			-- Wait for a start bit. If we wait long enough, we will see the 
			-- termination signal, in which case we abort and wait for not RPI.
			-- We clear no stop bit variable, which clears the global BYTERR 
			-- signal.
			if (state = 1) then
				if TCMD then 
					next_state := 63; 
				else 
					if RPS then 
						next_state := 2;
					else 
						next_state := 1; 
					end if;
				end if;
				no_stop_bit := false;
			end if;
			BYTERR <= no_stop_bit;
			
			-- Once we have a start bit, we proceed through the eight bits of
			-- a command byte, each bit taking four states. The first bit occurs
			-- at state 7 and the stop bit at state 39.
			if (state >= 2) and (state <= 38) then 
				next_state := state + 1; 
			end if;
			
			-- If the stop bit is present, we go to our end state. If it's missing,
			-- we go to our byte error state. The stop bit is zero, so RPS should 
			-- at this point be false.
			if (state = 39) then
				if not RPS then 
					next_state := 63;
				else 
					next_state := 62;
				end if;
			end if;
			
			-- Here we deal with unused states by directing them towards the byte
			-- error state.
			if (state > 39) and (state < 62) then 
				next_state := 62; 
			end if;
			
			-- In the byte error state, we set the "no stop bit" flag, which asserts the 
			-- global BYTERR signal. We will not reset this flag until the Byte Receiver
			-- starts a new byte reception. This flag tells the Command Processor to ignore
			-- the entire command. We wait in the byte error state until RBI is unasserted. 
			-- Because we do not assert RBD, the un-assertion of RBI will occur only when
			-- the Command Receiver encounters a Terminate Command signal.
			if (state = 62) then
				if not RBI then 
					next_state := 0;
				else 
					next_state := 62;
				end if;
				no_stop_bit := true;
			end if;
			
			-- In the end state, we assert Receive Byte Done and we wait for the command
			-- processor to un-assert Receive Byte Initiate. When we see not RBI, we return
			-- to the idle state and unassert RBD. When we see Terminate Command (TCMD) we
			-- unassert RBD.
			if (state = 63) then 
				if not RBI then 
					next_state := 0; 
				else 
					next_state := 63; 
				end if;
			end if;
			RBD <= (state = 63) and (not TCMD);
					
			-- The eight bits of the command are set every four states during
			-- the command reception.
			for i in 0 to 7 loop
				if (state = 35 - i * 4) then 
					if RPS then 
						cmd_in(i) <= '1'; 
					else 
						cmd_in(i) <= '0'; 
					end if;
				else 
					cmd_in(i) <= cmd_in(i); 
				end if;
			end loop;
			
			-- We assert Command Bit Strobe (CBS) one RCK period before the best moment
			-- to sample each bit value.
			if (state = 34) or (state = 30) or (state = 26) or (state = 22) 
				or (state = 18) or (state = 14) or (state = 10) or (state = 6) then
				CBS <= true;
			else 
				CBS <= false;
			end if;
			
			-- The Byte Strobe signal indicates that we have a start bit and is 
			-- useful as a test point trigger. It provides a pulse of two RCK 
			-- periods.
			BYTS <= (state = 2) or (state = 3);
			
			-- Assert the new state.
			state := next_state;
		end if;
	end process;

-- This process runs all the bits of a command through a sixteen-bit linear 
-- feedback shift register, with local name "crc" for "cyclic redundancy check". 
-- We preset crc to all ones. The final sixteen bits of every command are chosen 
-- so that they reset the crc register to all zeros. If crc is not zero at the 
-- end of a command, there was some error during reception. We use the Command
-- Bit Strobe (CBS) signal to clock crc, because CBS is asserted only when a command 
-- data bit is received, not when we receive a start or stop bit.
	Error_Check : process (RESET, RCK) is
		variable crc, next_crc : std_logic_vector(15 downto 0);
	begin
		if RESET = '1' then
			crc := (others => '1');
		elsif rising_edge(RCK) then
			if ICMD then
				-- When a new command transmission starts, we preload the cyclic redundancy
				-- check register to all ones.
				crc := (others => '1');
			else
				-- We use Command Bit Strobe (CBS) to clock each command bit into the CRC.
				-- The transmitter calculates the checksum with zeros in the last
				-- sixteen bits, reverses the order of these checksum bits, and sends
				-- them as the last two bytes of the actual transmission, instead of the
				-- zeros it used when it calculated its own checksum. These last sixteen
				-- bits, thus obtained, will reset the receiver CRC to zero, provided there
				-- has been no corruption of the data on the way.
				if CBS then
					for i in 0 to 9 loop next_crc(i) := crc(i+1); end loop;
					next_crc(10) := crc(11) xor crc(0);
					next_crc(11) := crc(12);
					next_crc(12) := crc(13) xor crc(0);
					next_crc(13) := crc(14) xor crc(0);
					next_crc(14) := crc(15);
					next_crc(15) := to_std_logic(RPS) xor crc(0);	
					crc := next_crc;
				end if;		
			end if;
		end if;
		
		-- The CRCERR flag tells us when the CRC is not zero. It will be zero when it
		-- has been reset by the two bytes of a correct checksum.
		CRCERR <= (crc /= "0000000000000000");
	end process;

-- Command Memory
	Command_Memory : entity CMD_FIFO port map (
		Reset => CMRST, 
		RPReset => '0',
		WrClock => not RCK,
		WrEn => CMWR,
		Data => cmd_in,
		RdClock => not CK,
		RdEn => CMRD,
		AlmostEmpty => CME,
		Full => CMF,
		Q => cmd_out);
	
-- The Command Processor detects Inititiate Command (ICMD) and activates the Byte Receiver. 
-- It stores command bytes in the Command Memory until it detects Terminate Command (TCMD). If
-- the Error Check reports no error, the Command Processor asserts Command Ready (CMDRDY) and
-- waits until the CPU asserts Command Processor Reset (CPRST) before returning to its rest
-- state. When the command is ready, the CPU can read all bytes out of the Command Memory. 
-- The Command Processor runs on the reference clock, which is 32.768 kHz, and proceeds to a 
-- new state every clock cycle. 
	Command_Processor: process (RESET, RCK) is
		
		-- General-purpose state names for the Command Processor
		constant idle_s : integer := 0;
		constant receive_cmd_s : integer := 1;
		constant store_cmd_s : integer := 2;
		constant check_fifo_s : integer := 3;
		constant check_cmd_s : integer := 4;
		constant complete_s : integer := 5;
		
		-- Variables and constants for the Command Processor
		variable state, next_state : integer range 0 to 7;
		
	begin
		-- We reset to the idle state on global RESET or the Command Processor
		-- Reset (CPRST).
		if (RESET = '1') or CPRST then
			state := idle_s;
			CMRST <= '1';
			
		-- The Command Processor state machine runs off RCK, which allows it to
		-- work with the Byte Receiver.
		elsif rising_edge(RCK) then
			-- Default next state and reset value.
			next_state := idle_s;
			CMRST <= '0';
			RBI <= false;
			CMWR <= '0';
		
			-- Idle State.
			if (state = idle_s) then
				if ICMD then 
					next_state := receive_cmd_s; 
				else 
					next_state := idle_s;
				end if;
				CMRST <= '1';
			end if;
			
			-- Receive a command byte. We assert RBI and wait for RBD. If we see 
			-- Terminate Command (TCMD), we move on. Note that the Byte Receiver 
			-- aborts on TCMD also.
			if (state = receive_cmd_s) then 
				if TCMD then 
					next_state := check_cmd_s;
				else 
					if RBD then 
						next_state := store_cmd_s;
					else 
						next_state := receive_cmd_s;
					end if;
				end if;
				RBI <= true;
			end if;
			
			-- Store the new command byte in the command memory. We assert Command
			-- Memory Write (CMWR) for one clock cycle.
			if (state = store_cmd_s) then 
				next_state := check_fifo_s;
				CMWR <= '1';
			end if;
			
			-- Check if the Command Memory is full. If so, abort. Otherwise, we 
			-- wait for RBD to be unasserted before receiving the next command byte.
			if (state = check_fifo_s) then
				if (CMF = '1') then
					next_state := idle_s;
				elsif not RBD then
					next_state := receive_cmd_s;
				else
					next_state := check_fifo_s;
				end if;
			end if;		
			
			-- There are two possible sources of error: a failure in the cyclic redundancy
			-- check (CRCERR) or an error in the structure of a command byte (BYTERR). If
			-- either is asserted, we go back to idle and ignore the command.
			if (state = check_cmd_s) then
				if BYTERR or CRCERR then 
					next_state := idle_s;
				else 
					next_state := complete_s;
				end if;
			end if;

			-- We have a completed command in memory, waiting for the CPU to read it out.
			-- We assert CMDRDY and wait until the CPU asserts CPRST. The command processor
			-- will ignore any further command transmission.
			if (state = complete_s) then
				next_state := complete_s;
			end if;
			
			-- Advance the state variable.
			state := next_state;
		end if;
		
		-- Command Ready tells the CPU that a command is available.
		CMDRDY <= (state = complete_s);
			
		-- Command Processor Active is true whenever the state is not idle.
		CPA <= (state /= idle_s);
	end process;
	
-- Test Point appears on P1-7.
--	TP <= df_reg(0);
	TP <= FCKEN;
end behavior;