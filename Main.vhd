-- Intraperitoneal Transmitter (IPT, A3054) Firmware, Top-Level Unit

-- V1.1 [25-MAR-26] Based upon P3041 V3.6 of 23-FEB-26. Set up inputs and outputs.

-- V1.2 [01-APR-26] Bring in the P3051 ring oscillator implementation: a big ring
-- with no divider creates FCK. Remove the Clock Calibrator process and hard-wire
-- fck_divisor in firmware. Compile for a 31-gate ring, set route priority on all
-- thirty-one gates, then re-compile for fewer gates and ignore warnings. Adapt
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
-- register turns on and off DC coupling and impedance measurement switch. The ADC
-- readout runs off FCK with SCK at 5 MHz.

-- V1.5 [10-APR-26] Add readback of amplifier configuration register. Eliminate 
-- support for random stimuli in software. Remove multiplier from software. Now
-- have I2C.asm include file. Transmit signal can be directed to any input, AC
-- or DC, temperature sensor, accelerometer, EEPROM, or synchronizing signal.
-- Simplify LED controller.


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
		
-- Configuration and Calibration of Transmitter.
	constant fck_divisor : integer := 25;
		
-- Configuration of OSR8 CPU.
	constant prog_cntr_len : integer := 12;
	constant cpu_addr_len : integer := 12;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;
	constant ram_addr_len : integer := 10;
	constant cmd_addr_len : integer := 10;

-- Memory Map Constants, sizes and base addresses.
	constant ram_bot : integer  := 16#0000#;
	constant ram_top : integer  := 16#03FF#;
	constant ctrl_bot : integer := 16#0400#;
	constant ctrl_top : integer := 16#07FF#;
	constant prog_bot : integer := 16#0800#;
	constant prog_top : integer := 16#0FFF#;
	
-- Memory Map Constants, low nibble addresses in units of bytes;
	constant mmu_irqb  : integer := 16#00#; -- Interrupt Request Bits (Read)
	constant mmu_imsk  : integer := 16#01#; -- Interrupt Mask Bits (Read/Write)
	constant mmu_irst  : integer := 16#02#; -- Interrupt Reset Bits (Write)
	constant mmu_acfg  : integer := 16#03#; -- Amplifier Configuration (Read/Write)
	constant mmu_stc   : integer := 16#04#; -- Stimulus Current (Write)
	constant mmu_rst   : integer := 16#05#; -- System Reset (Write)
	constant mmu_xhb   : integer := 16#06#; -- Transmit HI Byte (Write)
	constant mmu_xlb   : integer := 16#07#; -- Transmit LO Byte (Write)
	constant mmu_xch   : integer := 16#08#; -- Transmit Channel Number (Write)
	constant mmu_xcr   : integer := 16#09#; -- Transmit Control Register (Write)
	constant mmu_rfc   : integer := 16#0A#; -- Radio Frequency Calibration (Write)	
	constant mmu_ccr   : integer := 16#0B#; -- Clock Control Register (Write)
	constant mmu_dfr   : integer := 16#0C#; -- Diagnostic Flag Register (Read/Write)
	constant mmu_sr    : integer := 16#0D#; -- Status Register (Read)
	constant mmu_cmp   : integer := 16#0E#; -- Command Memory Portal(Read)
	constant mmu_cpr   : integer := 16#0F#; -- Command Processor Reset (Write)
	constant mmu_i1ph  : integer := 16#10#; -- Interrupt Timer One Period MSB (Write)
	constant mmu_i1pl  : integer := 16#11#; -- Interrupt Timer One Period LSB (Write)
	constant mmu_i2ph  : integer := 16#12#; -- Interrupt Timer Two Period MSB (Write)
	constant mmu_i2pl  : integer := 16#13#; -- Interrupt Timer Two Period LSB (Write)
	constant mmu_i3p   : integer := 16#14#; -- Interrupt Timer Three Period MSB (Write)
	constant mmu_i4p   : integer := 16#15#; -- Interrupt Timer Four Period MSB (Write)
	constant mmu_i2c00 : integer := 16#16#; -- i2c SDA=0 SCL=0 (Write)
	constant mmu_i2c01 : integer := 16#17#; -- i2c SDA=0 SCL=1 (Write)
	constant mmu_i2cA0 : integer := 16#18#; -- i2c SDA=A SCL=0 (Write)
	constant mmu_i2cA1 : integer := 16#19#; -- i2c SDA=A SCL=1 (Write)
	constant mmu_i2cZ0 : integer := 16#1A#; -- i2c SDA=Z SCL=0 (Write)
	constant mmu_i2cZ1 : integer := 16#1B#; -- i2c SDA=Z SCL=1 (Write)
	constant mmu_i2cMR : integer := 16#1C#; -- i2C Most Recent Eight Bits (Read)
	constant mmu_spicr : integer := 16#1D#; -- SPI Control Register (Write)
	constant mmu_spidh : integer := 16#1E#; -- SPI Data MSB (Read)
	constant mmu_spidl : integer := 16#1F#; -- SPI Data LSB (Read)
end;

architecture behavior of main is

-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Power Controller
	signal USERSTDBY, CLRFLAG : std_logic := '0';
	signal RESET : std_logic := '1';
	attribute syn_keep of RESET : signal is true;
	attribute nomerge of RESET : signal is "";
	signal SFLAG, STDBY : std_logic;
	signal SWRST : boolean := false;
	signal DACTIVE : boolean := true; 
	
-- Ring Oscillator, Fast Clock, and Transmit Clock
	signal TCK, FCK, CK : std_logic;
	attribute syn_keep of TCK, FCK, CK : signal is true;
	attribute nomerge of TCK, FCK, CK : signal is ""; 

-- Message Transmission.
	signal TXI, -- Transmit Initiate
		TXWP, -- Transmit Warmup
		TXA, -- Transmit Active
		TXB, -- Transmit Bit
		FHI -- Frequency High
		: boolean := false;
	attribute syn_keep of TXI, TXA : signal is true;
	attribute nomerge of TXI, TXA : signal is "";  
	signal xmit_bits : std_logic_vector(15 downto 0);
	constant tx_channel_default : integer := 1;
	signal tx_channel : integer range 0 to 255 := tx_channel_default;
	constant frequency_step : integer := 2; 
	constant default_frequency_low : integer := 5;
	signal frequency_low : integer range 0 to 31 := default_frequency_low;
		
-- Sensor Controller
	signal ADCCAL, -- Calibrate the Selected ADC
		ADCRD, -- ADC Access Initiate 
		ADCRL, -- ADC Rotate Left
		ADCRRL, -- ADC Rotate Left Again
		ADCBSY -- ADC Busy
		: boolean := false;
	attribute syn_keep of ADCRD, ADCBSY : signal is true;
	attribute nomerge of ADCRD, ADCBSY : signal is "";  
	signal adc_data : std_logic_vector(15 downto 0);
	signal spi_ctrl : std_logic_vector(1 downto 0);

-- Sensor Readout
	signal i2c_in : std_logic_vector(7 downto 0); -- I2C Serial Byte

-- Clock Control
	signal ENFCK : boolean; -- Enable Fast Clock
	signal BOOST : boolean; -- Boost CPU Clock
	attribute syn_keep of BOOST : signal is true;
	attribute nomerge of BOOST : signal is "";
	signal KEEPFCK : boolean; -- Keep FCK Running
	signal SRCK, SSRCK : std_logic;
	signal RCKLO : boolean;
	
-- Diagnostic Flag Register
	signal df_reg : std_logic_vector(3 downto 0) := (others => '0');

-- Program Memory Signals
	signal prog_data : std_logic_vector(7 downto 0); -- ROM Data
	signal prog_cntr : std_logic_vector(prog_cntr_len-1 downto 0); -- Prog Address
	signal prog_in : std_logic_vector(7 downto 0); 
	signal prog_in_addr : std_logic_vector(prog_cntr_len-1 downto 0); -- ROM Address
	signal PROGWR : std_logic; -- ROM Write
	
-- Process Memory Signals
	signal ram_addr : std_logic_vector(ram_addr_len-1 downto 0); -- RAM Address
	signal ram_out, ram_in : std_logic_vector(7 downto 0); -- RAM Data In and Out
	signal RAMWR : std_logic; -- Command Memory Write
	
-- Central Processing Unit Signals
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
	signal int_period_1, int_period_2 : std_logic_vector(15 downto 0);
	signal int_period_3, int_period_4 : std_logic_vector(7 downto 0);
	signal INTZ1, INTZ2, INTZ3, INTZ4 : boolean; -- Interrupt Counter Zero Flag
	
-- Byte Receiver
	signal RPS, -- Radio Frequency Power Synchronized
		ICMD, -- Initiate Command Reception
		TCMD, -- Terminate Command Reception
		RCMD, -- Receive Command
		RBI, -- Receive Command Byte Initiate
		RBD, -- Receive Command Byte Done
		CRCERR, -- Cyclic Redundancy Checksum Error
		BYTERR, -- Byte Error
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
		
-- Stimulus Current Controller
	signal stimulus_current : integer range 0 to 15;
	
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
-- Power Control Unit (PCU). The process asserts OND to keep the power on.
	PowerUp: process (RCK, CPA, DACTIVE) is
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
-- The Fast Clock process produces FCK when the microprocessor asserts Enable
-- Fast Clock (ENFCK). The fast clock must be running during sample transmission
-- or else the transmission will fail. The fast clock should be running during
-- ADC readout as well, or else it will proceed too slowly. All I2C accesses are
-- much quicker if the CPU is running in boost mode on TCK, which is half the 
-- frequency of FCK. The ring oscillator should be calibrated so that it produces
-- FCK of 10 MHz.
	Fast_Clock: entity ring_oscillator 
		generic map (ring_len => fck_divisor)	
		port map (
			ENABLE => to_std_logic(ENFCK or KEEPFCK or CPUISRV), 
			CK => FCK
		);

-- User memory and configuration code for the CPU. This RAM will be initialized at
-- start-up with a configuration file, and so may be read after power up to configure
-- sensor. The configuration data will begin at address zero.
	RAM : entity RAM port map (
		Clock => not CK,
		ClockEn => '1',
        Reset => '0',
		WE => RAMWR,
		Address => ram_addr, 
		Data => ram_in,
		Q => ram_out);

-- Instruction Memory for CPU. This memory will be initialized with the CPU program, 
-- the first instruction of the program being stored at address zero. The CPU reads 
-- the instruction memory with a separate address bus, which we call the program counter. 
-- The CPU may also write to the program memory through the cpu_addr, but only to the 
-- top kilobyte of the ROM.
	ROM : entity ROM port map (
		RdAddress => prog_cntr,
        RdClock => not CK,
        RdClockEn => '1',
        Reset => '0',	
        Q => prog_data,
		WrAddress => prog_in_addr,
		WrClock => CK,
		WrClockEn => PROGWR,
		WE => '1',
		Data => prog_in);

-- The processor itself, and eight-bit microprocessor with thirteen-bit address bus.
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
		
-- The Memory Manager maps eight-bit read and write access to the Sensor Controller, Sample 
-- Transmitter, Random Access Memory, and Interrupt Handler. Byte ordering is big-endian 
-- (most significant byte at lower address). 
	MMU : process (all) is
		variable all_bits : integer range 0 to 4096;
		variable bottom_bits : integer range 0 to 31;
	begin		
		-- By default, don't write to RAM or PROG memories, nor do we read from
		-- the command memory FIFO.
		RAMWR <= '0';
		PROGWR <= '0';
		CMRD <= '0';
		
		-- The RAM address we take from the lower bits of the cpu
		-- address. The RAM data in is always the cpu data out.
		ram_in <= cpu_data_out;
		ram_addr <= cpu_addr(ram_addr_len-1 downto 0);
		
		-- We can write to the user program memory with the CPU. We restrict 
		-- the program memory write address to the upper two kilobytes of the 
		-- program memory, making it impossible to write to the lower two 
		-- kilobytes. On no account do we want the user program to be able to 
		-- over-write the main program, which is loaded from the logic chip's 
		-- conguration EEPROM on power-up.
		prog_in <= cpu_data_out;
		prog_in_addr(11) <= '1';
		prog_in_addr(10 downto 0) <= cpu_addr(10 downto 0);
		
		-- These signals develop after the CPU asserts a new address
		-- along with CPU Write. They will be ready before the falling 
		-- edge of the CPU clock.
		all_bits := to_integer(unsigned(cpu_addr));
		bottom_bits := to_integer(unsigned(cpu_addr(5 downto 0)));
		cpu_data_in <= (others => '0');
		case all_bits is
			when ram_bot to ram_top => 
				if not CPUWR then
					cpu_data_in <= ram_out;
				else
					RAMWR <= to_std_logic(CPUDS);
				end if;
			when ctrl_bot to ctrl_top =>
				if not CPUWR then 
					case bottom_bits is
						when mmu_irqb => cpu_data_in <= int_bits;
						when mmu_imsk => cpu_data_in <= int_mask;
						when mmu_acfg =>
							cpu_data_in(0) <= DC;
							cpu_data_in(1) <= MSR;
						when mmu_dfr => cpu_data_in(3 downto 0) <= df_reg;
						when mmu_sr => 
							cpu_data_in(0) <= to_std_logic(CMDRDY); -- Command Ready Flag
							cpu_data_in(1) <= to_std_logic(ENFCK);  -- Fast Clock Enabled
							cpu_data_in(2) <= to_std_logic(ADCBSY); -- ADC Busy Flag
							cpu_data_in(3) <= to_std_logic(TXA);    -- Transmit Active Flag
							cpu_data_in(4) <= to_std_logic(CPA);    -- Command Processor Active Flag
							cpu_data_in(5) <= to_std_logic(BOOST);  -- Boost CPU Flag
							cpu_data_in(6) <= CME;                  -- Command Memory Empty
							cpu_data_in(7) <= RCK;                  -- Reference Clock
						when mmu_cmp =>
							cpu_data_in <= cmd_out;
							CMRD <= to_std_logic(CPUDS);
						when mmu_i2cMR => cpu_data_in <= i2c_in;
						when mmu_spidh => cpu_data_in <= adc_data(15 downto 8);
						when mmu_spidl => cpu_data_in <= adc_data(7 downto 0);
						when others => null;
					end case;
				end if;
			when prog_bot to prog_top =>
				if CPUWR then
					PROGWR <= to_std_logic(CPUDS);
				end if;
			when others =>
				null;
		end case;
		
		-- We use RESET to clear some registers and signals, but not all. We do not clear the
		-- software reset signal, SWRST, on RESET, since we want SWRST to assert RESET for one
		-- CK period. After a reset, the cpu address will not select the SWRST location, so
		-- SWRST will be cleared on the next falling edge of CK.
		if (RESET = '1') then
			ADCRD <= false;
			TXI <= false;
			TXWP <= false;
			ENFCK <= false;
			BOOST <= false;
			tx_channel <= 0;
			int_period_1 <= (others => '0');
			int_period_2 <= (others => '0');
			int_period_3 <= (others => '0');
			int_period_4 <= (others => '0');
			stimulus_current <= 0;
			df_reg <= (others => '0');
			int_mask <= (others => '0');
			int_rst <= (others => '1');
			CPRST <= true;
			DACTIVE <= false;
			frequency_low <= default_frequency_low;
			SDA <= 'Z';
			SCL <= '1';
			MSR <= '0';
			DC <= '0';
			
		-- We use the falling edge of RCK to write to registers and to initiate sensor 
		-- and transmit activity. Some signals we assert only for one CK period, and 
		-- these we assert as false by default.
		elsif falling_edge(CK) then
			CPRST <= false;
			SWRST <= false;
			ADCRD <= false;
			TXI <= false;
			int_rst <= (others => '0');
			if CPUDS and CPUWR then 
				if (all_bits >= ctrl_bot) and (all_bits <= ctrl_top) then
					case bottom_bits is
						when mmu_acfg =>
							DC <= cpu_data_out(0);
							MSR <= cpu_data_out(1);
						when mmu_xlb  => xmit_bits(7 downto 0) <= cpu_data_out;
						when mmu_xhb  => xmit_bits(15 downto 8) <= cpu_data_out;
						when mmu_xch  => tx_channel <= to_integer(unsigned(cpu_data_out));
						when mmu_xcr  => 
							TXI <= (cpu_data_out(0) = '1');
							TXWP <= (cpu_data_out(1) = '1');
						when mmu_rfc  => frequency_low <= to_integer(unsigned(cpu_data_out));
						when mmu_imsk => int_mask <= cpu_data_out;
						when mmu_irst => int_rst <= cpu_data_out;
						when mmu_stc  => stimulus_current <= to_integer(unsigned(cpu_data_out));
						when mmu_rst  => SWRST <= (cpu_data_out(0) = '1');
						when mmu_ccr  => 
							ENFCK <= (cpu_data_out(0) = '1');
							BOOST <= (cpu_data_out(1) = '1');
						when mmu_dfr  => df_reg <= cpu_data_out(3 downto 0);
						when mmu_cpr  => CPRST <= true;
						when mmu_i1ph => int_period_1(15 downto 8) <= cpu_data_out;
						when mmu_i1pl => int_period_1(7 downto 0) <= cpu_data_out;
						when mmu_i2ph => int_period_2(15 downto 8) <= cpu_data_out;
						when mmu_i2pl => int_period_2(7 downto 0) <= cpu_data_out;
						when mmu_i3p  => int_period_3(7 downto 0) <= cpu_data_out;
						when mmu_i4p  => int_period_4(7 downto 0) <= cpu_data_out;
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
						when mmu_spicr => 
							spi_ctrl <= cpu_data_out(1 downto 0);
							ADCRD <= (cpu_data_out(2) = '1');
							ADCCAL <= (cpu_data_out(3) = '1');
							ADCRL <= (cpu_data_out(4) = '1');
							ADCRRL <= (cpu_data_out(5) = '1');
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
	variable counter_1, counter_2 : integer range 0 to 65535;
	variable counter_3, counter_4 : integer range 0 to 255;
	variable mcnt : integer range 0 to 31;
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
		
		-- Interrupt Two Reset
		if int_rst(1) = '1' then
			int_rst_d(1)  <= '1';
			int_rst_s(1) <= '1';
		elsif falling_edge(RCK) then
			int_rst_d(1)  <= '0';
			int_rst_s(1) <= int_rst_d(1);
		end if;
		
		-- Interrupt Three Reset
		if int_rst(2) = '1' then
			int_rst_d(2)  <= '1';
			int_rst_s(2) <= '1';
		elsif falling_edge(RCK) then
			int_rst_d(2)  <= '0';
			int_rst_s(2) <= int_rst_d(2);
		end if;
		
		-- Interrupt Four Reset
		if int_rst(3) = '1' then
			int_rst_d(3) <= '1';
			int_rst_s(3) <= '1';
		elsif falling_edge(RCK) then
			int_rst_d(3) <= '0';
			int_rst_s(3) <= int_rst_d(3);
		end if;	
		
		-- The millisecond timer.
		if falling_edge(RCK) then
			if mcnt = 31 then
				mcnt := 0;
			else
				mcnt := mcnt + 1;
			end if;
		end if;	
		
		-- The interrupt timers run all the time, counting down from their period value
		-- to zero. The only way to stop them counting is to set their period value to
		-- zero so that they stick at zero. Just disabling the interrupt does not stop 
		-- the counter.

		-- Interrupt Timer One: a sixteen-bit delay timers. Counts milliseconds using 
		-- the millisecond clock, which is synchronous with RCK.
		if (int_rst_s(0) = '1') then
			counter_1 := 0;
		elsif rising_edge(RCK) then
			if (mcnt = 0) then
				if (counter_1 = 0) then
					counter_1 := to_integer(unsigned(int_period_1));
				else
					counter_1 := counter_1 - 1;
				end if;
			end if;
		end if;

		-- Interrupt Timer Two: a sixteen-bit delay timers. Counts milliseconds using the
		-- millisecond clock, which is synchronous with RCK.
		if (int_rst_s(1) = '1') then
			counter_2 := 0;
		elsif rising_edge(RCK) then
			if (mcnt = 0) then
				if (counter_2 = 0) then
					counter_2 := to_integer(unsigned(int_period_2));
				else
					counter_2 := counter_2 - 1;
				end if;
			end if;
		end if;
		
		-- Interrupt Timer Three: an eight bit repeating clock. Counts RCK.
		if rising_edge(RCK) then
			if (counter_3 = 0) then
				counter_3 := to_integer(unsigned(int_period_3));
			else
				counter_3 := counter_3 - 1;
			end if;
		end if;

		-- Interrupt Timer Four: an eight-bit repeating clock. Counts RCK.
		if rising_edge(RCK) then
			if (counter_4 = 0) then
				counter_4 := to_integer(unsigned(int_period_4));
			else
				counter_4 := counter_4 - 1;
			end if;
		end if;

		-- Using the interrupt reset bits that we synchronized with RCK, we 
		-- reset the interrupt bits. Otherwise, we set the interrupt timer
		-- bit when the timer reaches zero.
		
		-- Timer One Control
		if (int_rst_s(0) = '1') then
			int_bits(0) <= '0';
			INTZ1 <= true;
		elsif rising_edge(RCK) then
			INTZ1 <= (counter_1 = 0);
			if ((counter_1 = 0) and (not INTZ1)) then
				int_bits(0) <= '1';
			end if;
		end if;
		
		-- Timer Two Control
		if (int_rst_s(1) = '1') then
			int_bits(1) <= '0';
			INTZ2 <= true;
		elsif rising_edge(RCK) then
			INTZ2 <= (counter_2 = 0);
			if ((counter_2 = 0) and (not INTZ2)) then
				int_bits(1) <= '1';
			end if;
		end if;
		
		-- Timer Three Control
		if (int_rst_s(2) = '1') then
			int_bits(2) <= '0';
			INTZ3 <= true;
		elsif rising_edge(RCK) then
			INTZ3 <= (counter_3 = 0);
			if ((counter_3 = 0) and (not INTZ3)) then
				int_bits(2) <= '1';
			end if;
		end if;
		
		-- Timer Four Control
		if (int_rst_s(3) = '1') then
			int_bits(3) <= '0';
			INTZ4 <= true;
		elsif rising_edge(RCK) then
			INTZ4 <= (counter_4 = 0);
			if ((counter_4 = 0) and (not INTZ4)) then
				int_bits(3) <= '1';
			end if;
		end if;

		-- We disable the remaining interrupt lines.
		for i in 4 to 7 loop
			int_bits(i) <= '0';
		end loop;
		
		-- We generate an interrupt if any one interrupt bit is 
		-- set and unmasked.
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

	-- The ADC Controller reads out one of the fourteen-bit ADCs when ADC Read
	-- (ADCRD) is asserted. While running, it asserts ADC BUSY (ADCBSY).
	-- The CPU can poll ADCBSY until the access is complete or it can wait for
	-- 16 CPU clock cycles for data to be available or 25 CPU cycles for a self-
	-- calibration to complete. The ADC Controller runs on the Fast Clock (FCK). 
	-- If FCK is not running, the ADC Controller will do nothing. The ADCRD signal 
	-- must be asserted for at least one FCK period, which is certain to be the
	-- case even when the CPU is running in boost mode at 5 mHz. The ADCCAL flag
	-- tells the controller to read twenty-four bits instead of eighteen, which 
	-- causes a self-calibration provided that the calibration access is the first 
	-- one after power-up. No new data is recorded during a self-calibration.
	ADC_Controller : process (RESET, FCK) is
		variable state, next_state : integer range 0 to 63 := 0;
		constant end_access : integer := 50;
		
 	begin
		-- Upon startup, we make sure we are in the idle state.
		if (RESET = '1') then 
			state := 0;
			ADCBSY <= false;
			
		-- The ADC Contoller proceeds through states so as initiate a conversion,
		-- read out one zero, fourteen data bits, and three trailing zeros. If
		-- the ADCCAL flag is set, it reads out nine trailing zeros so as to 
		-- initiate an ADC self-calibration. When ADCCAL is not set, the fourteen 
		-- data bits are shifted into the adc_data register.
		elsif rising_edge(FCK) then
			if (state = 0) then 
				if ADCRD then 
					next_state := 1;
				else 
					next_state := 0;
				end if;
			end if;
			if (state > 0) and (state < end_access) then
				next_state := state + 1;
			end if;
			if (state = end_access) then
				if ADCRD then
					next_state := state;
				else
					next_state := 0;
				end if;
			end if;
			if ADCCAL then
				SCK <= to_std_logic(
					(state = 0) 
					or ((state < end_access - 2) and (state mod 2) = 1));
				ADCBSY <= (state > 0) and (state <= end_access - 2);
			else 
				SCK <= to_std_logic(
					(state = 0) 
					or ((state < end_access - 14) and ((state mod 2) = 1))
				);
				ADCBSY <= (state > 0) and (state <= end_access - 14);
				if (state = 1) then
					adc_data <= (others => '0');
				end if;
				if (state >= 4) and (state <= 30) and ((state mod 2) = 0) then
					adc_data(15 downto 1) <= adc_data(14 downto 0);
					adc_data(0) <= SDO;
				end if;
				if (state = 31) and ADCRL then
					adc_data(15 downto 1) <= adc_data(14 downto 0);
					adc_data(0) <= '0';
				end if;
				if (state = 32) and ADCRRL then
					adc_data(15 downto 1) <= adc_data(14 downto 0);
					adc_data(0) <= '0';
				end if;
			end if;
			state := next_state;
		end if;
		
		NADC1 <= to_std_logic(not (ADCBSY and (spi_ctrl = "00")));
		NADC2 <= to_std_logic(not (ADCBSY and (spi_ctrl = "01")));
		NADC3 <= to_std_logic(not (ADCBSY and (spi_ctrl = "10")));
		NADC4 <= to_std_logic(not (ADCBSY and (spi_ctrl = "11")));
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

-- The Indicator Controller takes the stimulus current and sets the value of LED to
-- turn on or off the lamp.
	Lamp_Controller: process (RESET, RCK) is 
	begin
		if RESET = '1' then
			LED <= '0';
		elsif rising_edge(RCK) then
			if stimulus_current > 0 then
				LED <= '1';
			else 
				LED <= '0';
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
		constant endcount : integer := 63;
		variable counter : integer range 0 to endcount;
	begin
		if RESET = '1' then
			counter := endcount;
		elsif rising_edge(RCK) then
			if RPS then 
				if (counter = endcount) then 
					counter := endcount;
					ICMD <= true;
				else 
					counter := counter + 1;
					ICMD <= false;
				end if;
			else
				counter := 0;
				ICMD <= false;
			end if;
		end if;
	end process;
	
-- We detect a long enough period without command power to 
-- terminate command reception, and set the TCMD signal.
	Terminate_Command: process (RESET, RCK) is 
		constant endcount : integer := 255;
		variable counter : integer range 0 to endcount;
	begin
		if RESET = '1' then
			counter := endcount;
		elsif rising_edge(RCK) then
			if not RPS then 
				if (counter = endcount) then 
					counter := endcount;
					TCMD <= true;
				else 
					counter := counter + 1;
					TCMD <= false;
				end if;
			else
				counter := 0;
				TCMD <=  false;
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
		
		-- Variables for the Command Processor
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
			-- check (CRCERR) or an error in the structure of a command byte (BYTERR). 
			-- If either is asserted, we go back to idle.
			if (state = check_cmd_s) then
				if CRCERR or BYTERR then 
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
	TP <= df_reg(0);

end behavior;