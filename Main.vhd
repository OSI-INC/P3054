-- <pre> Intraperitoneal Transmitter (IPT, A3054) Firmware, Toplevel Unit

-- V1.1, 10-FEB-26: Based on P3041 V3.1. With Synplify set to area optimization
-- the code takes up 1267 LUTs in the QFN-32 LCMXO2-1200ZE. Reduce the two millisecond
-- interrupt timers from 16 bits to 8 bits. Now 1195 LUTs.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		RP, -- Receive Power
		RCK -- Reference Clock
		: in std_logic; 
		XEN, -- Transmit Enable, for data transmission
		TP1, -- Test Point One, available on P3-1
		TP2, -- Test Point Two, available on P3-2
		TP3, -- Test Point Three, available on P3-3, also TMS
		TP4, -- Test Point Four, available on P3-4, also TCK
		OND, -- Keep Device On
		NCS, -- Chip Select for DAC, Negative-True
		SCK -- Serial Clock for Battery Voltage DAC
		: out std_logic;
		ONL, -- Turn Lamp On
		SDO -- Serial Data Out for DAC
		: inout std_logic;
		xdac -- Transmit DAC Output, to set data transmit frequency
		: out std_logic_vector(4 downto 0));
		
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
	constant mmu_sdb  : integer := 0;  -- Sensor Data Byte (Write)
	constant mmu_scr  : integer := 1;  -- Sensor Control Register (Write)
	constant mmu_irqb : integer := 2;  -- Interrupt Request Bits (Read)
	constant mmu_imsk : integer := 3;  -- Interrupt Mask Bits (Read/Write)
	constant mmu_irst : integer := 4;  -- Interrupt Reset Bits (Write)
	constant mmu_dact : integer := 5;  -- Device Active (Write)
	constant mmu_stc  : integer := 6;  -- Stimulus Current (Write)
	constant mmu_rst  : integer := 7;  -- System Reset (Write)
	constant mmu_xhb  : integer := 8;  -- Transmit HI Byte (Write)
	constant mmu_xlb  : integer := 9;  -- Transmit LO Byte (Write)
	constant mmu_xch  : integer := 10; -- Transmit Channel Number (Write)
	constant mmu_xcr  : integer := 11; -- Transmit Control Register (Write)
	constant mmu_rfc  : integer := 12; -- Radio Frequency Calibration (Write)	
	constant mmu_etc  : integer := 13; -- Enable Transmit Clock (Write)
	constant mmu_tcf  : integer := 14; -- Transmit Clock Frequency (Read)
	constant mmu_tcd  : integer := 15; -- Transmit Clock Divider (Write)
	constant mmu_bcc  : integer := 16; -- Boost CPU Clock (Write)
	constant mmu_dfr  : integer := 17; -- Diagnostic Flag Register (Read/Write)
	constant mmu_sr   : integer := 18; -- Status Register (Read)
	constant mmu_cmp  : integer := 19; -- Command Memory Portal(Read)
	constant mmu_cpr  : integer := 21; -- Command Processor Reset (Write)
	constant mmu_i1p  : integer := 22; -- Interrupt Timer One Period MSB (Write)
	constant mmu_i2p  : integer := 24; -- Interrupt Timer Two Period MSB (Write)
	constant mmu_i3p  : integer := 26; -- Interrupt Timer Three Period MSB (Write)
	constant mmu_i4p  : integer := 27; -- Interrupt Timer Four Period MSB (Write)
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
	
-- Ring Oscillator and Transmit Clock
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
	signal xmit_bits : std_logic_vector(15 downto 0) := (others => '0');
	constant tx_channel_default : integer := 1;
	signal tx_channel : integer range 0 to 255 := tx_channel_default;
	constant frequency_step : integer := 1; 
	constant default_frequency_low : integer := 5;
	signal frequency_low : integer range 0 to 31 := default_frequency_low;
		
-- Sensor Controller
	signal CS : boolean; -- Chip Select for DAC
	signal SAI, -- Sensor Access Initiate 
		SAA -- Sensor Access Active
		: boolean := false;
	attribute syn_keep of SAI, SAA : signal is true;
	attribute nomerge of SAI, SAA : signal is "";  
	signal sensor_bits_in : std_logic_vector(7 downto 0) := (others => '0');
		
-- Clock Calibrator
	signal ENTCK : boolean; -- Enable the Transmit Clock
	signal tck_frequency : integer range 0 to 255; -- Transmit Clock Counter
	constant default_tck_divisor : integer := 11;
	signal tck_divisor : integer range 0 to 15 := default_tck_divisor;
	
-- Boost Controller
	signal BOOST : boolean;
	attribute syn_keep of BOOST : signal is true;
	attribute nomerge of BOOST : signal is "";
	
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
		CPUIRQ -- Interrupt Request
		: boolean; 
	signal CPUSIG : std_logic_vector(2 downto 0); -- Signals for debugging.

-- Interrupt Handler signals.
	signal int_mask, int_bits, int_rst, int_set : std_logic_vector(7 downto 0);
	signal int_period_1, int_period_2 : std_logic_vector(7 downto 0);
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
	signal BYTSEL, -- Command Memory Select
		CME, -- Command Memory Empty
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

			if (state < stdby_state) then state := state + 1;
			elsif (SFLAG = '0') then state := stdby_state;
			elsif (state < end_state) then state := state + 1; 
			else state := end_state; end if;
		end if;
		
		-- The OND signal keeps power applied to the logic chip after
		-- RP is unasserted, which is the case after the initializing
		-- pulse, when command bits are incoming, and after the end of
		-- command transmistion. We assert OND when we have the command
		-- processor is active (CPA) or the microprocessor has asserted
		-- Device Active (DACTIVE).
		OND <= to_std_logic(CPA or DACTIVE);
	end process;	
	
-- Ring Oscillator. This oscillator turns on when the microprocessor asserts
-- Enable Transmit Clock (ENTCK). The transmit clock must be running during a
-- sample transmission in order for the timing of the transmission to be correct.
-- The transmit clock should be turned on during a sensor access as well, so that
-- the sensor access will be quick and the sensor can power down again sooner.
	Fast_CK : entity ring_oscillator port map (
		ENABLE => to_std_logic(ENTCK), 
		calib => tck_divisor,
		CK => FCK);
	
-- The Transmit Clock process divides FCK in two so as to produce a clock with
-- exactly 50% duty cycle and frequency close to 5 MHz, which we call the 
-- Transmit Clock (TCK). We clock TCK on the falling edge of FCK.
	Tx_CK : process (FCK) is 
	begin
		if falling_edge(FCK) then TCK <= to_std_logic(TCK = '0'); end if;
	end process;

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
			SIG => CPUSIG,
			RESET => RESET,
			CK => CK
		);
		
-- The Memory Manager maps eight-bit read and write access to the Sensor Controller, Sample 
-- Transmitter, Random Access Memory, and Interrupt Handler. Byte ordering is big-endian 
-- (most significant byte at lower address). 
	MMU : process (CK,RESET) is
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
		bottom_bits := to_integer(unsigned(cpu_addr(7 downto 0)));
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
						when mmu_sdb => cpu_data_in <= sensor_bits_in;
						when mmu_irqb => cpu_data_in <= int_bits;
						when mmu_imsk => cpu_data_in <= int_mask;
						when mmu_dfr => cpu_data_in(3 downto 0) <= df_reg;
						when mmu_tcf => cpu_data_in <= std_logic_vector(to_unsigned(tck_frequency,8));
						when mmu_sr => 
							cpu_data_in(0) <= to_std_logic(CMDRDY); -- Command Ready Flag
							cpu_data_in(1) <= to_std_logic(ENTCK);  -- Transmit Clock Enabled
							cpu_data_in(2) <= to_std_logic(SAA);    -- Sensor Access Active Flag
							cpu_data_in(3) <= to_std_logic(TXA);    -- Transmit Active Flag
							cpu_data_in(4) <= to_std_logic(CPA);    -- Command Processor Active Flag
							cpu_data_in(5) <= to_std_logic(BOOST);  -- Boost CPU Flag
							cpu_data_in(6) <= CME;                  -- Command Memory Empty
						when mmu_cmp =>
							cpu_data_in <= cmd_out;
							CMRD <= to_std_logic(CPUDS);
					end case;
				end if;
			when prog_bot to prog_top =>
				if CPUWR then
					PROGWR <= to_std_logic(CPUDS);
				end if;
				cpu_data_in <= (others => '0');
		end case;
		
		-- We use RESET to clear some registers and signals, but not all. We do not clear the
		-- software reset signal, SWRST, on RESET, since we want SWRST to assert RESET for one
		-- CK period. After a reset, the cpu address will not select the SWRST location, so
		-- SWRST will be cleared on the next falling edge of CK.
		if (RESET = '1') then
			SAI <= false;
			TXI <= false;
			TXWP <= false;
			ENTCK <= false;
			BOOST <= false;
			tck_divisor <= default_tck_divisor;
			tx_channel <= 0;
			int_period_1 <= (others => '0');
			int_period_2 <= (others => '0');
			int_period_3 <= (others => '0');
			int_period_4 <= (others => '0');
			stimulus_current <= 0;
			df_reg <= (others => '0');
			int_mask <= (others => '0');
			CPRST <= true;
			DACTIVE <= false;
			frequency_low <= default_frequency_low;
			
		-- We use the falling edge of RCK to write to registers and to initiate sensor 
		-- and transmit activity. Some signals we assert only for one CK period, and 
		-- these we assert as false by default.
		elsif falling_edge(CK) then
			CPRST <= false;
			SWRST <= false;
			SAI <= false;
			TXI <= false;
			int_rst <= (others => '0');
			if CPUDS and CPUWR then 
				if (all_bits >= ctrl_bot) and (all_bits <= ctrl_top) then
					case bottom_bits is
						when mmu_scr  => SAI <= true;
						when mmu_xlb  => xmit_bits(7 downto 0) <= cpu_data_out;
						when mmu_xhb  => xmit_bits(15 downto 8) <= cpu_data_out;
						when mmu_xch  => tx_channel <= to_integer(unsigned(cpu_data_out));
						when mmu_xcr  => 
							TXI <= (cpu_data_out(0) = '1');
							TXWP <= (cpu_data_out(1) = '1');
						when mmu_rfc  => frequency_low <= to_integer(unsigned(cpu_data_out));
						when mmu_imsk => int_mask <= cpu_data_out;
						when mmu_irst => int_rst <= cpu_data_out;
						when mmu_dact => DACTIVE <= (cpu_data_out(0) = '1');
						when mmu_stc  => stimulus_current <= to_integer(unsigned(cpu_data_out));
						when mmu_rst  => SWRST <= (cpu_data_out(0) = '1');
						when mmu_etc  => ENTCK <= (cpu_data_out(0) = '1');
						when mmu_tcd  => tck_divisor <= to_integer(unsigned(cpu_data_out));
						when mmu_bcc  => BOOST <= (cpu_data_out(0) = '1');
						when mmu_dfr  => df_reg <= cpu_data_out(3 downto 0);
						when mmu_cpr => CPRST <= true;
						when mmu_i1p => int_period_1(7 downto 0) <= cpu_data_out;
						when mmu_i2p => int_period_2(7 downto 0) <= cpu_data_out;
						when mmu_i3p  => int_period_3(7 downto 0) <= cpu_data_out;
						when mmu_i4p  => int_period_4(7 downto 0) <= cpu_data_out;
					end case;
				end if;
			end if;
		end if;
	end process;
	
	-- The Clock Calibrator counts cycles of TCK for one half-period of RCK after the
	-- assertion of Enable Transmit Clock (ENTCK) and makes them available to the CPU
	-- in the tck_frequency register. If TCK is 5.00 MHz and RCK is 32.768 kHz, 
	-- tck_frequency will be 76 when the counter stops. The counter will hold its 
	-- value until ENTCK is unasserted.
	Clock_Calibrator : process (TCK,ENTCK) is
	variable state, next_state : integer range 0 to 3;
	begin
		if not ENTCK then
			state := 0;
			tck_frequency <= 0;
		elsif rising_edge(TCK) then
			next_state := state;
			if (state = 0) then
				if ENTCK then 
					next_state := 1;
				end if;
				tck_frequency <= 0;
			elsif (state = 1) then
				if (RCK = '1') then 
					next_state := 2;
				end if;
				tck_frequency <= tck_frequency + 1;
			elsif (state = 2) then
				if not ENTCK then 
					next_state := 0;
				end if;
				tck_frequency <= tck_frequency;
			else 
				next_state := 0;
				tck_frequency <= tck_frequency;
			end if;
			state := next_state;
		end if;
	end process;
	
	-- The Boost Controller switches the CPU bewteen RCK and TCK, but makinge 
	-- sure TCK is enabled for two cycles before connecting the CPU clock to
	-- TCK. The CPU must first enable TCK with ENTCK, then assert BOOST. When
	-- switching back to RCK, it must first unassert BOOST, then unassert ENTCK.
	Boost_Controller : process (TCK,ENTCK) is
	variable state, next_state : integer range 0 to 3;
	begin
		if not ENTCK then
			state := 0;
		elsif rising_edge(TCK) then
			case state is
				when 0 =>
					if BOOST then 
						next_state := 1;
					else 
						next_state := 0;
					end if;
				when 1 => next_state := 3;
				when 3 =>
					if (not BOOST) then
						next_state := 2;
					else
						next_state := 3;
					end if;
				when 2 => 
					if (RCK = '0') then
						next_state := 0;
					else
						next_state := 2;
					end if;
			end case;
			state := next_state;
		end if;
		CK <= to_std_logic(((RCK = '1') and (state = 0))
			or ((TCK = '1') and (state = 3))
			or (state = 2));
	end process;

	-- The Interrupt_Controller provides the interrupt signal to the CPU in response to
	-- timer events. By default, at power-up, all interrupts are masked. We can set the
	-- period of each timer by writing to locations in the CPU control space. If we want
	-- the counter to have period N ticks, we write value N-1 to the period registers.
	Interrupt_Controller : process (RCK,CK,RESET) is
	variable counter_1, counter_2 : integer range 0 to 255;
	variable counter_3, counter_4 : integer range 0 to 255;
	variable mcnt : integer range 0 to 31;
	
	begin
		-- Millisecond counter divides 32.768 kHz by 32 to give 1.024 kHz, period
		-- is 0.977 ms, which is close enough to one millisecond for our purposes.
		if falling_edge(RCK) then
			if mcnt = 31 then
				mcnt := 0;
			else
				mcnt := mcnt + 1;
			end if;
		end if;
	
		-- The first of two sixteen-bit delay timers, generating interrupt bit zero
		-- we call it Interrupt Timer One. Counts milliseconds.
		if (int_rst(0) = '1') then
			counter_1 := 0;
		elsif rising_edge(RCK) then
			if (mcnt = 0) then
				if (counter_1 = 0) then
					counter_1 := to_integer(unsigned(int_period_1));
				else
					counter_1 := counter_1 - 1;
				end if;
			else
				counter_1 := counter_1;
			end if;
		end if;

		-- The second of two sixteen-bit delay timers, generating interrupt bit one
		-- we call it Interrupt Timer Two. Counts milliseconds.
		if (int_rst(1) = '1') then
			counter_2 := 0;
		elsif rising_edge(RCK) then
			if (mcnt = 0) then
				if (counter_2 = 0) then
					counter_2 := to_integer(unsigned(int_period_2));
				else
					counter_2 := counter_2 - 1;
				end if;
			else
				counter_2 := counter_2;
			end if;
		end if;
		
		-- The first of two eight-bit repeating timers, generating interrupt bit two
		-- we call it Interrupt Timer Three.
		if rising_edge(RCK) then
			if (counter_3 = 0) then
				counter_3 := to_integer(unsigned(int_period_3));
			else
				counter_3 := counter_3 - 1;
			end if;
		end if;

		-- The second of two eight-bit repeating timers, generating interrupt bit three
		-- we call it Interrupt Timer Four.
		if rising_edge(RCK) then
			if (counter_4 = 0) then
				counter_4 := to_integer(unsigned(int_period_4));
			else
				counter_4 := counter_4 - 1;
			end if;
		end if;

		-- We clear the timer one interrupt and set the the timer one zero 
		-- flag when we assert the int_rst bit zero. The interrupt bit
		-- itself is set when we the counter reaches zero but the zero flag 
		-- is not set.
		if (int_rst(0) = '1') then
			int_bits(0) <= '0';
			INTZ1 <= true;
		elsif rising_edge(RCK) then
			INTZ1 <= (counter_1 = 0);
			if ((counter_1 = 0) and (not INTZ1)) then
				int_bits(0) <= '1';
			end if;
		end if;
			
		-- The timer two interrupt.
		if (int_rst(1) = '1') then
			int_bits(1) <= '0';
			INTZ2 <= true;
		elsif rising_edge(RCK) then
			INTZ2 <= (counter_2 = 0);
			if ((counter_2 = 0) and (not INTZ2)) then
				int_bits(1) <= '1';
			end if;
		end if;
		
		-- The timer three interrupt.
		if (int_rst(2) = '1') then
			int_bits(2) <= '0';
			INTZ3 <= true;
		elsif rising_edge(RCK) then
			INTZ3 <= (counter_3 = 0);
			if ((counter_3 = 0) and (not INTZ3)) then
				int_bits(2) <= '1';
			end if;
		end if;

		-- The timer four interrupt.
		if (int_rst(3) = '1') then
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
		-- set and unmasked, but we synchronize with the CPU
		-- clock.
		if falling_edge(CK) then
			CPUIRQ <= (int_bits and int_mask) /= "00000000";
		end if;
	end process;

	-- The Sensor Controller reads out the eight-bit battery monitoring ADC when it
	-- sees Sensor Access Initiate (SAI). While running, it asserts Sensor Acces Active
	-- (SAA), which the CPU can poll until the access is complete. It runs off the 
	-- Transmit Clock (TCK), so the CPU must enable TCK with ENTCK in order for the 
	-- process to start. The SAI signal will be asserted for one period of CK following
	-- a CPU write to the SAI location. Further writes to the same location will have
	-- no effect until the Sensor Controller returns to its idle state.
	Sensor_Controller : process (TCK,RESET) is
		variable state, next_state : integer range 0 to 31 := 0;
		
 	begin
		-- Upon startup, we make sure we are in the idle state and we are not
		-- requesting a byte access by the Sensor Interface.
		if (RESET = '1') then 
			state := 0;
			
		-- The Sensor Contoller proceeds through states so as initiate a conversion,
		-- read out two zeros, read eight data bits, and load the result into the
		-- sensor register. By default, the state machine increases its state variable
		-- by one, so we state explicitly when the state should do otherwise.
		elsif rising_edge(TCK) then
			next_state := state + 1;
			
			case state is
				when 0 => -- CS unasserted, SCK HI.
					CS <= false; SCK <= '0';
					if not SAI then next_state := 0; end if;
				when 1 => -- Assert CS to start conversion, a zero appears on SDO.
					CS <= true; SCK <= '1';
				when 2 => -- Clock the second zero out of DAC.
					CS <= true; SCK <= '0';
				when 3 => -- Prepare another falling edge on SCK.
					CS <= true; SCK <= '1';
				when 4 => -- Clock D7 out of DAC
					CS <= true; SCK <= '0';
				when 5 => -- Read D7 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(7) <= SDO;			
				when 6 => -- Clock D6 out of DAC
					CS <= true; SCK <= '0';
				when 7 => -- Read D6 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(6) <= SDO;			
				when 8 => -- Clock D5 out of DAC
					CS <= true; SCK <= '0';
				when 9 => -- Read D5 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(5) <= SDO;			
				when 10 => -- Clock D4 out of DAC
					CS <= true; SCK <= '0';
				when 11 => -- Read D4 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(4) <= SDO;			
				when 12 => -- Clock D3 out of DAC
					CS <= true; SCK <= '0';
				when 13 => -- Read D3 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(3) <= SDO;			
				when 14 => -- Clock D2 out of DAC
					CS <= true; SCK <= '0';
				when 15 => -- Read D2 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(2) <= SDO;			
				when 16 => -- Clock D1 out of DAC
					CS <= true; SCK <= '0';
				when 17 => -- Read D1 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(1) <= SDO;			
				when 18 => -- Clock D0 out of DAC
					CS <= true; SCK <= '0';
				when 19 => -- Read D0 into sensor register.
					CS <= true; SCK <= '1';
					sensor_bits_in(0) <= SDO;			
				when 20 => -- Drive SCK LO again
					CS <= true; SCK <= '0';
				when 21 => -- Unassert CS and leave SCK LO.
					CS <= false; SCK <= '0';
					if SAI then next_state := 21; end if;
				when others => next_state := 0;
			end case;
			SAA <= (state /= 0) and (state /= 21);
			state := next_state;
		end if;
		
		-- CS we negate for our active-low chip select output.
		NCS <= to_std_logic(not CS);
	end process;
	
	
-- The Message Transmitter responds to Transmit Initiate (TXI) by turning on the 
-- radio-frequency oscillator, reading sixteen bits from one of the sensors and
-- transmitting the bits. The process runs off TCK, so the CPU must assert ENTCK
-- for the process to run. The TXI signal will be asserted for one period of CK
-- following a CPU write to the TXI location. Further writes to the same location
-- will be ignored until the Message Transmitter returns to its idle state.
	Message_Transmitter : process (TCK) is
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
		variable state, next_state : integer range 0 to 63 := 0; -- Stample Transmit State
		
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
	Frequency_Modulation : process is
	begin
		-- Frequency modulation runs off the 10-MHz FCK clock. This clock is
		-- synchronous with TCK. It presents a rising edge over 10 ns after 
		-- both the rising and falling edges of TCK. Thus, when we see a
		-- rising edge on FCK, the value of TCK and TXB are both established.
		wait until (FCK = '1');
	
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
	end process;
	
-- The Receive Power signal must be synchronized with the RCK clock.
	Synchronize_RP: process is 
	begin
		wait until (RCK = '0');
		RPS <= (RP = '1');
	end process;
	
-- We detect a long enough burst of command power to initiate
-- command reception, and set the ICMD signal.
	Initiate_Command: process is 
		constant endcount : integer := 63;
		variable counter : integer range 0 to endcount := 0;
	begin
		wait until (RCK = '1');
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
	end process;
	
-- We detect a long enough period without command power to 
-- terminate command reception, and set the TCMD signal.
	Terminate_Command: process is 
		constant endcount : integer := 255;
		variable counter : integer range 0 to endcount := 0;
	begin
		wait until (RCK = '1');
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
	end process;
	
-- The Receive Command (RCMD) signal indicates that a command is being 
-- received. We set RCMD when Initiate Command (ICMD) occurs, and we clear
-- RCMD when Terminate Command (TCMD) occurs.
	Receive_Command: process is
	begin
		wait until (RCK = '1');
		if not RCMD then
			RCMD <= ICMD;
		else 
			RCMD <= not TCMD;
		end if;
	end process;

-- We watch for a start bit and receive serial bytes when instructed
-- to do so by the Command Processor with the RBI signal.
	Byte_Receiver: process is
		variable state, next_state : integer range 0 to 63 := 0;
		variable no_stop_bit : boolean := false;
	begin
		wait until (RCK = '1');
		
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
	end process;

-- This process runs all the bits of a command through a sixteen-bit linear 
-- feedback shift register, with local name "crc" for "cyclic redundancy check". 
-- We preset crc to all ones. The final sixteen bits of every command are chosen 
-- so that they reset the crc register to all zeros. If crc is not zero at the 
-- end of a command, there was some error during reception. We use the Command
-- Bit Strobe (CBS) signal to clock crc, because CBS is asserted only when a command 
-- data bit is received, not when we receive a start or stop bit.
	Error_Check : process is
		variable crc, next_crc : std_logic_vector(15 downto 0) := (others => '1');
	begin
		wait until (RCK = '1');
		
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
	Command_Processor: process (RCK, RESET, CPRST) is
		
		-- General-purpose state names for the Command Processor
		constant idle_s : integer := 0;
		constant receive_cmd_s : integer := 1;
		constant store_cmd_s : integer := 2;
		constant check_fifo_s : integer := 3;
		constant check_cmd_s : integer := 4;
		constant complete_s : integer := 5;
		
		-- Variables for the Command Processor
		variable state, next_state : integer range 0 to 7 := 0;
		
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

-- Test Point One appears on P4-1.
	TP1 <= to_std_logic((df_reg(0)='1') or CPA);
	
-- Test Point Two appears on P4-2.
	TP2 <= to_std_logic((df_reg(1)='1') or CMDRDY);
	
-- Test Point Three appears on P4-3 after the programming connector is removed.
	TP3 <= to_std_logic(CPUIRQ);

-- Test point Four appears on P4-4 after the programming connector is removed. 
-- Note that P4-4 is tied LO with 8 kOhm on the programming extension, so if 
-- this output is almost always HI, and the programming extension is still 
-- attached, quiescent current increases by 250 uA.
	TP4 <= to_std_logic(FHI);

end behavior;