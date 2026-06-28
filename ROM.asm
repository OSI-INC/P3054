; A3054 Intraperitoneal Transmitter (IPT) Program
; -----------------------------------------------

; This code runs in an OSR8 microprocessor. We begin with
; declaration of two-byte constants for memory addresses and 
; one-byte constants for parameter values. After that come
; routines to configure the sensors, the interrupt routine
; that does just about everything, the command acknowledge
; routines, and the command interpreter routine. The 
; initialization code comes next, and the main loop. Last
; of all we have included files: the eight-bit multiplier
; routine and I2C write and read routines.

; Configuration Constants.
const version         1 ; The firmwarwe version.
const id_hi        0xAA ; 0-255, no restrictions
const id_lo        0x55 ; 0-255, low nibble cannot be 0x0 or 0xF 
const f_low          14 ; Radio frequency calibration.

; CPU Address Map Boundary Constants. The first kilobytes is
; RAM. The RAM shadows the first 256 bytes, which serve for 
; the control registers. Then we have 256 bytes for each of
; the stack, user program variables, and main program 
; variables. The top kilobyte is the user program itself. 
const ctrl_bot   0x0000 ; Control Registers
const stack_bot  0x0100 ; Main Program Stack
const uvar_bot   0x0200 ; User Program Variables
const mvar_bot   0x0300 ; Main Program Variables
const prog_bot   0x0400 ; User Program Instructions
const prog_top   0x07FF ; Top of Address Range

; Control Register Locations. These are shadowed in RAM.
const mmu_irqb   0x0000 ; Interrupt Request Bits (Read)
const mmu_imsk   0x0001 ; Interrupt Mask Bits (Write/Readback)
const mmu_irst   0x0002 ; Interrupt Reset Bits (Write)
const mmu_acfg   0x0003 ; Amplifier Configuration (Write/Readback)
const mmu_led    0x0004 ; Lamp Control (Write/Readback)
const mmu_rst    0x0005 ; Software Reset (Write)
const mmu_xhb    0x0006 ; Transmit HI Byte (Write/Readback)
const mmu_xlb    0x0007 ; Transmit LO Byte (Write/Readback)
const mmu_xch    0x0008 ; Transmit Channel Number (Write/Readback)
const mmu_xcr    0x0009 ; Transmit Control Register (Write)
const mmu_rfc    0x000A ; Radio Frequency Calibration (Write/Readback)
const mmu_ccr    0x000B ; Clock Control Register (Write/Readback)
const mmu_dfr    0x000C ; Diagnostic Flag Register (Write/Readback)
const mmu_sr     0x000D ; Status Register (Read)
const mmu_cmp    0x000E ; Command Memory Portal (Read)
const mmu_cpr    0x000F ; Command Processor Reset (Write)
const mmu_i3p    0x0010 ; Interrupt Timer Three Period (Write/Readback)
const mmu_i4p    0x0011 ; Interrupt Timer Four Period (Write/Readback)
const mmu_i2c00  0x0012 ; i2c SDA=0 SCL=0 (Write/Readback)
const mmu_i2c01  0x0013 ; i2c SDA=0 SCL=1 (Write/Readback)
const mmu_i2cA0  0x0014 ; i2c SDA=A SCL=0 (Write/Readback)
const mmu_i2cA1  0x0015 ; i2c SDA=A SCL=1 (Write/Readback) 
const mmu_i2cZ0  0x0016 ; i2c SDA=Z SCL=0 (Write/Readback)
const mmu_i2cZ1  0x0017 ; i2c SDA=Z SCL=1 (Write/Readback) 
const mmu_i2cMR  0x0018 ; i2c Most Recent Eight Bits (Read)
const mmu_smcr   0x0019 ; Sample Control Register (Write)
const mmu_saddr  0x001A ; Sample Address Register (Write/Readback)
const mmu_adcdh  0x001B ; ADC Data HI Byte (Read)
const mmu_adcdl  0x001C ; ADC Data LO Byte (Read)
const mmu_accdh  0x001D ; Accumulator HI Byte (Read)
const mmu_accdl  0x001E ; Accumulator LO Byte (Read)
const mmu_msr    0x001F ; Impedance Measurement Control (Write/Readback)

; Variables: Main Program, Command Reception and Response
const Sack_key   0x0300 ; Acknowledgement key
const ccmdb      0x0301 ; Copy of Command Byte

; Variables: Main Program, Sample Transmission
const xmit_ch    0x0303 ; Telemetry Channel Number

; Variables: Main Program, Temperature Measurement
const tmp_chb    0x0304 ; Temperature counter, HI
const tmp_clb    0x0305 ; Temperature counter, LO
const temp_hi    0x0306 ; Saved temperature measurement, HI
const temp_lo    0x0307 ; Saved temperature measurement, LO

; Variables: Main Program, Sample Controller Accumulator
const x1_idx     0x0308 ; X1 Sample Counter
const x2_idx     0x0309 ; X2 Sample Counter
const x3_idx     0x030A ; X3 Sample Counter
const x4_idx     0x030B ; X4 Sample Counter
const x1_txp     0x030C ; X1 Sample Period, multiples of 1/1024 s.
const x2_txp     0x030D ; X1 Sample Period, multiples of 1/1024 s.
const x3_txp     0x030E ; X1 Sample Period, multiples of 1/1024 s.
const x4_txp     0x030F ; X1 Sample Period, multiples of 1/1024 s.
const x1_mult    0x0310 ; X1 Repeat Count, applies during accumulation.
const x2_mult    0x0311 ; X2 Repeat Count, applies during accumulation.
const x3_mult    0x0312 ; X3 Repeat Count, applies during accumulation.
const x4_mult    0x0313 ; X4 Repeat Count, applies during accumulation.

; Variables: Main Program, Non-Volatile Memory
const nvm_cntr   0x0316 ; Counter for NVM transmission

; Variables: Main Program, User Program Control
const UPrun      0x0322 ; Running
const UPinit     0x0323 ; Initialize

; Scratch space for memory transfers.
const scratch    0x0380 ; A 128-byte scratchpad.

; Constants: Status Register Bit Masks
const sr_cmdrdy    0x01 ; Command Ready
const sr_enfck     0x02 ; Enable Fast Clock
const sr_ledon     0x04 ; Lamp On
const sr_mck       0x08 ; Millisecond Clock
const sr_cpa       0x10 ; Command Processor Active
const sr_adcbsy    0x20 ; ADC Controller Busy
const sr_cme       0x40 ; Command Memory Empty
const sr_rck       0x80 ; Current Value of Reference Clock

; Constants: Transmit Control Masks
const tx_txi       0x01 ; Assert transmit initiate
const tx_txwp      0x02 ; Assert transmit warm-up

; Constants: Auxiliary message types.
const at_id           1 ; Identification
const at_ack          2 ; Acknowledgements
const at_batt         3 ; Battery Measurement
const at_conf         4 ; Confirmation
const at_ver          5 ; Version

; Constants: Generic Bit Masks
const bit0_mask    0x01 ; Bit Zero Mask
const bit1_mask    0x02 ; Bit One Mask
const bit2_mask    0x04 ; Bit Two Mask
const bit3_mask    0x08 ; Bit Three Mask
const bit4_mask    0x10 ; Bit Four Mask
const bit5_mask    0x20 ; Bit Five Mask
const bit6_mask    0x40 ; Bit Six Mask
const bit7_mask    0x80 ; Bit Seven Mask
const bit0_clr     0xFE ; Bit Zero Clear
const bit1_clr     0xFD ; Bit One Clear
const bit2_clr     0xFB ; Bit Two Clear
const bit3_clr     0xF7 ; Bit Three Clear
const bit4_clr     0xEF ; Bit Four Clear
const bit5_clr     0xDF ; Bit Five Clear
const bit6_clr     0xBF ; Bit Six Clear
const bit7_clr     0x7F ; Bit Seven Clear

; Constants: Delays, Frequencies, and Periods
const min_tcf        72 ; Minimum TCK periods per half RCK period
const initial_tcd    15 ; Max possible value of TCK divisor
const tx_delay       40 ; Wait time for sample transmission in TCK periods
const wp_delay      255 ; Warm-up delay for auxiliary messages in TCK periods
const num_vars      255 ; Number of variable bytes to clear at start
const uprog_tick    163 ; User program interrupt period minus one
const id_delay       33 ; Identification spacing in TCK periods
const ms_tick        33 ; One millisecond in RCK periods
const sample_period  31 ; Sample period in RCK periods
const adc_rdly       16 ; Clock cycles for ADC readout
const adc_cdly       22 ; Clock cycles for ADC self-calibration
const tmp_period      8 ; Temperature update period in 1/4 s
const lon_ms         20 ; Lamp on time in milliseconds
const loff_ms       200 ; Lamp off time in milliseconds

; Constants: Instruction Codes
const ret_code     0x0A ; Return from subroutine instruction

; Constants: Command Codes
const op_stop         0 ; 0 operands
const op_start        1 ; 8 operands
const op_xon          2 ; 2 operand
const op_xoff         3 ; 0 operands
const op_batt         4 ; 0 operands
const op_id           5 ; 0 operands
const op_pgld         6 ; 1 operands
const op_pgon         7 ; 0 operands
const op_pgoff        8 ; 0 operands
const op_pgrst        9 ; 0 operands
const op_shdn        10 ; 0 operands
const op_ver         11 ; 0 operands
const op_lon         12 ; 0 operands
const op_loff        13 ; 0 operands
const op_zon         14 ; 0 operands
const op_zoff        15 ; 0 operands

; Constants: Non-Volatile Memory. The M24C16 EEPROM provides 
; 2K x 8 of NVM with an I2C interface. We can read 1-2048 
; bytes in one read cycle. We can write 1-16 bytes in one write 
; cycle. The device address consists only of four bits, the lower 
; three bits of the seven-bit I2C address are used to select one 
; of eight 256-byte blocks within the EEPROM.
const nvm_addr     0x50 ; Device address.
const nvm_amask    0x07 ; Mask for bottom three bits.

; Constants: Temperature Sensor. The TMP117 provides sixteen-bit 
; read and write registers to the I2C bus.
const tmp_addr     0x49 ; I2C address
const tmp_treg     0x00 ; Temperature register
const tmp_creg     0x01 ; Configuration register
const tmp_oneh     0x0C ; One-shot measurement, MSB
const tmp_onel     0x00 ; One-shot measurement, LSB
const tmp_fasth    0x00 ; Fast measurement, MSB
const tmp_fastl    0x00 ; Fast measurement, LSB

; Constants: Accelerometer. The BMA423 requires configuration for
; its internal sample rate. We give the internal addresses of 
; registers. When the address is always for a two-byte read, we 
; say so.
const bma_addr     0x18 ; I2C address
const bma_id       0x00 ; Identifier register
const bma_x        0x12 ; ACC_X register, two bytes, little-endian
const bma_y        0x14 ; ACC_Y register, two bytes, little-endian
const bma_z        0x16 ; ACC_Z register, two bytes, little-endian
const bma_time0    0x18 ; SENSOR_TIME_0 register
const bma_time1    0x19 ; SENSOR_TIME_1 register
const bma_time2    0x1A ; SENSOR_TIME_2 register
const bma_temp     0x22 ; TEMPERATURE register
const bma_status   0x2A ; INTERNAL_STATUS register
const bma_aconf    0x40 ; ACC_CONF register
const bma_arange   0x41 ; ACC_RANGE register
const bma_pctrl    0x7D ; PWR_CTRL register
const bma_pconf    0x7C ; PWR_CONF register
const bma_2g       0x00 ; For ACC_RANGE, +-2g
const bma_4g       0x01 ; For ACC_RANGE, +-4g
const bma_8g       0x02 ; For ACC_RANGE, +-8g
const bma_16g      0x03 ; For ACC_RANGE, +-16g
const bma_1hz      0x01 ; For ACC_CONF, 1 Hz, no averaging, no filter
const bma_25hz     0x06 ; For ACC_CONF, 25 Hz, no averaging, no filter
const bma_100hz    0x08 ; For ACC_CONF, 100 Hz, no averaging, no filter
const bma_enable   0x04 ; For PWR_CTRL, enable data acquisition.
const bma_disable  0x00 ; For PWR_CTRL, disable data acquisition.
const bma_pwrsv    0x03 ; For PWR_CONF, power save, fifo self-start.
const bma_sdly       50 ; Startup delay in RCK periods.

; Constants: Sample Controller.
const sm_read      0x01 ; Read an ADC and store
const sm_calib     0x03 ; Read and calibrate an ADC
const sm_add       0x04 ; Add sample to accumulator
const sm_rst       0x08 ; Reset the accumulator
const sm_x1        0x00 ; Base of of X1 sample memory
const sm_x2        0x40 ; Base of X2 sample memory
const sm_x3        0x80 ; Base of X3 sample memory
const sm_x4        0xC0 ; Base of X4 sample memory
const smca_mask    0xC0 ; Channel address mask

; Constants: Mathematical.
const off_16bs     0x80 ; Convert sixteen bit signed to unsigned.


; ------------------------------------------------------------
; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.

start:

jp main
jp interrupt

; ------------------------------------------------------------
; A multi-millisecond delay. Pass the number of milliseconds
; in A. The delay will be between A-1 and A milliseconds. If
; A=0, the delay will be 255-256 ms.

; This routine will run in fast or slow modes. If interrupted, the
; delay may be extended, depending upon how quickly the interrupt
; service completes.

delay_ms:
push F
push A
push B

push A
pop B

ld A,(mmu_dfr)
or A,bit2_mask
ld (mmu_dfr),A

delay_ms_lo:

ld A,(mmu_sr)
and A,sr_mck
jp z,delay_ms_lo

delay_ms_hi:

ld A,(mmu_sr)
and A,sr_mck
jp nz,delay_ms_hi

dec B
jp nz,delay_ms_lo

ld A,(mmu_dfr)
and A,bit2_clr
ld (mmu_dfr),A

pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Write 16N bytes to the non-volatile memory (NVM), where N is 
; 1-128, making a total write size of 16 to 2048 bytes. In IX 
; we pass a pointer to the first byte to be written. In HL we 
; pass the eleven-bit sub-address at which the write should 
; begin. The write must begin on a sixteen-byte boundary, for
; that is the EEPROM page size. The routine clears the bottom
; four bits of the sub-address to ensure writing to a page
; boundary. Upon return, IX points to the location after the 
; last byte written and the sub-address in HL points to the 
; start of the next page in NVM.
;
; Can run in slow or boost mode. If interrupted, the interrupt
; service routine must not call any I2C routine.

nvm_wr:

push F
push A
push B
push C
push D

push H
pop A
and A,nvm_amask
or A,nvm_addr
push A
pop H

push L
pop A
and A,0xF0
push A
pop L

ld A,16
push A
pop C

push A
pop D

nvm_wr_loop:
call i2c_wr
ld A,6
call delay_ms
push C
pop B
push L
pop A
add A,B
push A
pop L
push H
pop A
adc A,0
push A
pop H
dec D
jp nz,nvm_wr_loop

pop D
pop C
pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Calibrate the transmit clock frequency. We take the CPU out
; of boost, turn off the transmit clock, and repeat a cycle of
; setting the transmit clock divisor and running the transmit
; clock to measure its frequency. Eventually we get a divisor
; that provides a transmit period in the range 195-215 ns. We
; leave the transmit clock off at the end.
;
; Assumes the calling program is running in slow mode with
; interrupts disabled.

calibrate_tck:

; Push flags and registers.

push F
push A           
push B           

; Pop registers and return.

pop B           
pop A           
pop F
ret  

; ------------------------------------------------------------
; Perform a one-shot measurement of temperature and shut down
; the sensor afterwards. Current consumption will drop to 
; about 250 nA after the single measurement. Prior to initiating
; the conversion, the routine reads out the temperature and stores 
; in two memory locations.
;
; Can run in slow or boost mode, but interrupts must be disabled.
;

tmp_single:

; Push flags and registers.

push F
push A
push B
push C
push H
push L

; Read out sixteen-bit temperature sensor result. We have our 
; i2c_rd routine read them from the sensor and place them 
; directly into their two locations in the main program memory.
; We have to add an offset to the high byte so as to change the 
; signed integer measurement into an unsigned offset integer for 
; which 32768 is zero. 

ld A,tmp_addr
push A
pop H
ld A,tmp_treg
push A
pop L
ld A,2
push A
pop C
ld IX,temp_hi
call i2c_rd
dec IX
dec IX
ld A,(IX)
add A,off_16bs   
ld (IX),A

; Initiate a conversion, which we do by writing two bytes to
; the sensor's configuration register. We first put these 
; two bytes in the scratchpad so that the i2c routine will 
; read them from main program memory and write them to the
; sensor. We already have the sensor address in H. We load
; the configuration register address into L.

ld A,tmp_creg
push A
pop L
ld IX,scratch
ld A,tmp_oneh
ld (IX),A
inc IX
ld A,tmp_onel
ld (IX),A
dec IX
ld A,2
push A
pop C
call i2c_wr

pop L
pop H
pop C
pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Configure the accelerometer. We write to the power configuration 
; register, then the power control register, then wait for a time 
; greater than 500 us, then set the update rate and measurement range. 
; Beware changing the order of the first two writes and the delay. The 
; accelerometer will deliver zeros if we deviate from the correct 
; sequence.
;
; Can run in slow or boost mode, but interrupts must be disabled.
;

bma_config:

push F
push A
push B
push C
push H
push L

; Establish the I2C address in Register H, where it will remain.

ld A,bma_addr
push A
pop H

; Specify the PWR_CONF register with its sub-address in L.

ld A,bma_pconf
push A
pop L
ld IX,scratch
ld A,bma_pwrsv
ld (IX),A
ld A,1
push A
pop C
call i2c_wr

; Enable or disable data acquisition by writing to the PWR_CTRL 
; register.

ld A,bma_pctrl
push A
pop L
ld A,bma_enable
ld (IX),A
ld A,1
push A
pop C
call i2c_wr

; Wait for the sensor to configure.

ld A,bma_sdly
dly A 

; Configure accelerometer update rate.

ld A,bma_aconf 
push A
pop L
ld A,bma_1hz
ld (IX),A
ld A,1
push A
pop C
call i2c_wr

; Configure for +-16 g range.

ld A,bma_arange
push A
pop L
ld A,bma_16g
ld (IX),A
ld A,1
push A
pop C
call i2c_wr

pop L
pop H
pop C
pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Configure the ADCs, which consists of getting them each to
; perform self-calibration. We must call this configuration before
; any other samples are taken from the ADCs, just after we apply
; power to the IPT. The self-calibration consists of a twenty-four
; bit readout, which is effective before any sample is taken, but
; not effective after the first sample is taken. The ADC Controller
; runs of FCK, so we must have TCK running when we call this routine.
;
; Assumes boost mode and interrupts disabled.
;

adc_calib:

push F
push A

ld A,sm_x1
ld (mmu_saddr),A
ld A,sm_calib
ld (mmu_smcr),A
adc_calib_1:
ld A,(mmu_sr)
and A,sr_adcbsy
jp nz,adc_calib_1

ld A,sm_x2
ld (mmu_saddr),A
ld A,sm_calib
ld (mmu_smcr),A
adc_calib_2:
ld A,(mmu_sr)
and A,sr_adcbsy
jp nz,adc_calib_2

ld A,sm_x3
ld (mmu_saddr),A
ld A,sm_calib
ld (mmu_smcr),A
adc_calib_3:
ld A,(mmu_sr)
and A,sr_adcbsy
jp nz,adc_calib_3

ld A,sm_x4
ld (mmu_saddr),A
ld A,sm_calib
ld (mmu_smcr),A
adc_calib_4:
ld A,(mmu_sr)
and A,sr_adcbsy
jp nz,adc_calib_4

pop A
pop F
ret

; ------------------------------------------------------------
; The interrupt handler. Assumes that it interrupts a program
; running off the slow clock. Boosts as quickly as possible to
; fast clock, executes, then restores the clock. We handle the
; user program interrupt, then stimulus interrupt, and finally
; the transmit interrupt. By this means, when the interrupts
; are coincident, the user program can affect the stimulus and
; the stimulus can affect the transmission. The synchronizing
; signal, for example, will reflect the most recent state of
; the stimulus.

interrupt:

; Push A onto the stack. We are already in boost, provided that
; we got here with an interrupt request.

push A              ; Save A on stack
push F              ; Save the flags onto the stack.

; Set diagnostic flag zero.

ld A,(mmu_dfr)      ; Load the diagnostic flag register.
or A,bit0_mask      ; set bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.

; Push all the registers, even if we don't use them in the interrupt
; code. We want to protect the calling process from the user program.
push B
push C
push D
push E
push H
push L
push IX
push IY

; Handle the user program interrupt, in which we call the user program
; and allow it to execute and return. Because we just pushed all the
; registers, the user program can do what it likes with all the registers
; and flags, with the exception of the interrupt flag, which it must
; handle with care. Right now, the interrupt flag is set, and interrupts
; are disabled. Clearing the interrupt flag could cause the user program
; to be interrupted to execute itself in a recursion that overflows the
; stack.

int_uprog:

ld A,(mmu_irqb)     ; Read the interrupt request bits
and A,bit2_mask     ; and test bit two,
jp z,int_uprog_done ; skip if not uprog interrupt.
ld A,bit2_mask      ; Reset this interrupt
ld (mmu_irst),A     ; with the bit two mask.
call prog_bot       ; Call the user program.
int_uprog_done:

; Handle the transmit interrupt, if it exists. We transmit a synchronizing signal.
; We won't wait for the transmission to complete because we are certain to follow 
; our transmission with at least one RCK period when we move out of boost. 

int_xmit:

ld A,(mmu_irqb)     ; Read the interrupt request bits
and A,bit3_mask     ; and test bit three,
jp z,int_xmit_done  ; skip transmit if not set.

ld A,bit3_mask      ; Reset this interrupt
ld (mmu_irst),A     ; with the bit three mask.

ld A,(xmit_ch)      ; Load A with telemetry channel number
ld (mmu_xch),A      ; and write the transmit channel register.

; We are assigning transmit channel numbers to ADC inputs,
; temperature sensor, NVM, and accelerometer in this
; prototype code, so we can read any of them one at a time.
; The selection will be based upon the lower nibble of the
; channel number, which has range 1-14. Channels 1-8 are for
; the inputs AC/DC. Channel 9 for temperature. Channel 10 for
; NVM, 11 for accelerometer, 12 for the state of the LED,
; 13-14 are for the device identifier.

and A,0x0F
push A
pop B
dec A
and A,0x08
jp z,int_xmit_adc
push B
pop A
sub A,9
jp z,int_xmit_temp
push B
pop A
sub A,10
jp z,int_xmit_nvm
push B
pop A
sub A,11
jp z,int_xmit_acc
push B
pop A
sub A,12
jp z,int_xmit_led
jp int_xmit_id

; Transmit a temperature measurement. After that, we decrement
; the two-byte temperature period counter. When it reaches zero,
; update the temperature measurement and reset the counter.

int_xmit_temp:
ld A,(temp_hi)
ld (mmu_xhb),A
ld A,(temp_lo)
ld (mmu_xlb),A
ld A,(tmp_clb)
sub A,1
ld (tmp_clb),A
ld A,(tmp_chb)
sbc A,0
ld (tmp_chb),A
jp p,int_xmit_rdy
ld A,tmp_period
ld (tmp_chb),A
ld A,0
ld (tmp_clb),A
call tmp_single
jp int_xmit_rdy

; Read a sample from an ADC and transmit. We apply AC coupling for
; channel numbers for which the lower nibble is 1-4 and DC coupling
; for lower nibble 5-8. We read ADCs 1-4 for channels 1-4 and again
; ADCs 1-4 for channels 5-8.

int_xmit_adc:

; Compose the sample address out of the channel number and the
; index. Leaving the address of the first sample (index zero) in
; register B.
ld A,(xmit_ch)
dec A
and A,0x03
rrc A
rrc A
push A
pop B
ld A,(x1_idx)
or A,B

; Set the sample address and initiate an ADC read cycle. We don't 
; need to wait for completion because we have enough stuff to do 
; before we change the sample address again. The readout continues 
; in the background.
ld (mmu_saddr),A
ld A,sm_read
ld (mmu_smcr),A

; Decrement the sample index.
ld A,(x1_idx)
dec A
ld (x1_idx),A

; If the sample index is zero or greater, we are not ready to transmit, 
; so we are done.
jp p,int_xmit_done

; We have finished storing samples. Reset the sample index
; to the transmit period minus one. Save the transmit period
; in C for later.
ld A,(x1_txp)
push A
pop C
dec A
ld (x1_idx),A

; Point to the accumulator control register with
; IX and the sample address with IY.
ld IX,mmu_smcr
ld IY,mmu_saddr

; Reset the accumulator. 
ld A,sm_rst
ld (IX),A

; We are going to add sixteen samples to the accumulator
; so as to produce a sum of the correct magnitude. These 
; could be sixteen different samples, sixteen copies of
; a single sample, or some other such combination. The 
; sample multiplier tells us how many times to add each 
; sample to the accumulator. We store the sample multiplier
; in register E.
ld A,(x1_mult)
push A
pop E

; The accumulator loop goes through txp samples in the sample 
; memory and adds each of them mult times to the accumulator.
; We begin by moving the first sample address from B, where we
; stored it earlier, into A, and loading it into the sample 
; address register.
int_xmit_acc_loop:
  push B
  pop A
  ld (IY),A
  push E
  pop D
  ld A,sm_add
  int_xmit_mult_loop:
    ld (IX),A
    dec D
  jp nz,int_xmit_mult_loop
  inc B
  dec C
jp nz,int_xmit_acc_loop

; The accumulator now contains our transmit sample, so
; load it into the transmitter.
ld A,(mmu_accdh)
ld (mmu_xhb),A
ld A,(mmu_accdl)
ld (mmu_xlb),A
jp int_xmit_rdy

; Read two bytes from the non-volatile memory.

int_xmit_nvm:
ld A,nvm_addr
push A
pop H
ld A,(nvm_cntr)
add A,2
and A,0x7E
push A
pop L
ld (nvm_cntr),A
ld IX,scratch
ld A,2
push A
pop C
call i2c_rd
dec IX
ld A,(IX)
ld (mmu_xlb),A
dec IX
ld A,(IX)
ld (mmu_xhb),A
jp int_xmit_rdy

; Transmit an accelerometer measurement. The byte
; ordering on the accelerometer is little-endian.

int_xmit_acc:
ld A,bma_addr
push A
pop H
ld A,bma_x
push A
pop L
ld IX,scratch
ld A,2
push A
pop C
call i2c_rd
dec IX
ld A,(IX)
add A,off_16bs   
ld (mmu_xhb),A
dec IX
ld A,(IX)
ld (mmu_xlb),A
jp int_xmit_rdy

; Transmit the state of the LED, and also, hidden
; in the lower byte, the status register.

int_xmit_led:
ld A,(mmu_led)
rrc A
or A,0x40
ld (mmu_xhb),A
ld A,(mmu_sr)
ld (mmu_xlb),A
jp int_xmit_rdy

; Transmit the device identifier.

int_xmit_id:
ld A,id_hi 
ld (mmu_xhb),A
ld A,id_lo 
ld (mmu_xlb),A
jp int_xmit_rdy

; Ready to transmit. The sample bytes and channel number are 
; loaded in their respective registers. Now all we need to 
; do is initiate and wait.

int_xmit_rdy:
ld A,tx_txi         ; Load transmit initiate bit
ld (mmu_xcr),A      ; and write to transmit control register.
ld A,tx_delay       ; Wait for transmit to
dly A               ; complete.
int_xmit_done:

; Restore registers.

int_done:
pop IY
pop IX
pop L
pop H
pop E
pop D
pop C
pop B

; Clear diagnostic flag zero.

ld A,(mmu_dfr)      ; Load the diagnostic flag register.
and A,bit0_clr      ; Clear bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.


; Restore flags and accumulator, return from interrupt.

pop F               ; Restore the flags.
pop A               ; Restore A.
rti                 ; Return from interrupt.

; ------------------------------------------------------------
; Transmit an annoucement, which consists of two auxiliary 
; messages: one with data and another a confirmation immediately
; following. The two messages allow us to receive the announcement
; and identify the device, as well as eliminate noise announcements.
; We pass the auxiliary type in register A and the auxiliary data 
; in register B. 
;
; Assumes boost mode with interrupts disabled.

xmit_annc:

push F
push A
push B
push C

; Move the auxiliary type into register C.

push A
pop C

; Prepare the VCO for message transmission. We must warm it up or
; else its frequency will be wrong at tranmission time.

ld A,tx_txwp        ; Turn on the VCO by writing the 
ld (mmu_xcr),A      ; warm-up bit to the transmit control register.
ld A,wp_delay       ; Wait for a number of TCK periods while 
dly A               ; the VCO warms up.
ld A,0              ; Turn off the VCO and
ld (mmu_xcr),A      ; let the battery recover
ld A,wp_delay       ; before we 
dly A               ; transmit.

; Prepare the auxiliary message. For the channel number, we must 
; set the bottom nibble to 0xF in order to identify this message
; as auxiliary. The top nibble is the top niblle of the device
; identifier's low byte. Within the auxiliary message, we begin with
; the bottom nibble of the device identifier's low byte. We follow
; with a nibble containing the auxiliary type, which has been 
; passed in A. The second byte consists of the data in register B.

ld A,id_lo          ; Load LO byte of identifier into A,
or A,0x0F           ; set lower four bits to one
ld (mmu_xch),A      ; and write the transmit channel register.
push B              ; Transfer the data byte from
pop A               ; B into A
ld (mmu_xlb),A      ; and write to transmit LO register.
push C              ; Transfer auxiliary type
pop B               ; into B.
ld A,id_lo          ; Load LO byte of identifier again.
sla A               ; Shift A 
sla A               ; left
sla A               ; four
sla A               ; times.
or A,B              ; Set the auxiliary type to acknowledgement.
ld (mmu_xhb),A      ; Write to transmit HI register.

; Transmit the message.

ld A,tx_txi         ; Initiate transmission with another write to
ld (mmu_xcr),A      ; control register, which also turns off the warm-up.
ld A,tx_delay       ; Wait for a number of TCK periods while 
dly A               ; the transmit completes.

; Transmit a confirmation to complete the announcement. The auxilliary
; type of a confirmation is always at_conf and the data byte is always
; the high byte of the device identifier.

ld A,id_hi          ; Load HI byte id identifier into A and
ld (mmu_xlb),A      ; write to transmit LO register.
ld A,id_lo          ; Load LO byte of identifier into A,
or A,0x0F           ; set lower four bits to one and
ld (mmu_xch),A      ; write to the transmit channel register.
ld A,id_lo          ; Load LO byte into A again,
sla A               ; shift A 
sla A               ; left
sla A               ; four
sla A               ; times.
or A,at_conf        ; The confirmation type code.
ld (mmu_xhb),A      ; Write to transmit HI register.
ld A,tx_txi         ; Initiate transmission with a write to the transmit
ld (mmu_xcr),A      ; control register.
ld A,tx_delay       ; Wait for confirmation
dly A               ; transmition to complete.

pop C
pop B
pop A
pop F

ret


; ------------------------------------------------------------
; Transmit an acknowledgement. We put the auxiliary type in
; A and the acknowledgement key in B, then call our announcement
; routine.
;
; Assumes boost mode with interrupts disabled because it calls
; the xmit_annc routine that requires boost mode and interrupts
; disabled.


annc_ack:

push F
push A
push B

ld A,(Sack_key)
push A
pop B
ld A,at_ack
call xmit_annc

pop B
pop A
pop F

ret

; ------------------------------------------------------------
; Transmit a battery measurement. Routine mimics an A3041.
;
; Assumes boost mode with interrupts disabled because it calls
; the xmit_annc routine that requires boost mode and interrupts
; disabled.

annc_batt:

push F
push A
push B

ld A,102  
push A 
pop B  
ld A,at_batt 
call xmit_annc 

pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Announce the version number. We use A and B to pass the 
; version message identifier and the version number itself.
;
; Assumes boost mode with interrupts disabled because it calls
; the xmit_annc routine that requires boost mode and interrupts
; disabled.

annc_ver:

push F
push A
push B

ld A,version        ; Put the version number
push A              ; in register
pop B               ; B for procedure call.
ld A,at_ver         ; Load version type in A.
call xmit_annc      ; Transmit annoucement.

pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Announce the device identifier. We wait for time and then
; transmit the identifier and confirmation messages that 
; make up the announcement.
;
; Assumes boost mode with interrupts disabled.

annc_id:

push F
push A
push B
push H
push L

; Delay for id_delay clock cycles multiplied by numeric value 
; of the device id. By this means, each device transmits its
; identifying message at a different time. With id_delay set
; to 2*tx_delay, no two ISTs will collide, but we may have to
; wait 1.4 s for all our answers. Using tx_delay, two ISTs will
; collide only if their IDs are consecutive, and we will wait
; only 0.7 s. 

ld A,id_hi
push A
pop H
ld A,id_lo
push A
pop L
identify_delay:
ld A,id_delay
dly A
push L
pop A
sub A,1
push A
pop L
push H
pop A
sbc A,0
push A
pop H
jp nc,identify_delay

; Prepare A and B for call to xmit_annc.

ld A,id_hi  ; Load HI byte id identifier 
push A              ; into A and
pop B               ; store in B.
ld A,at_id          ; Load the identify type code into A.
call xmit_annc      ; Transmit annoucement.

; Return.

pop L
pop H
pop B
pop A
pop F

ret


; ------------------------------------------------------------
; Read a byte out of the command memory portal and store in a
; variable location, as well as returning it in A. With gcb_dsp
; set, we produce a serial display of the byte on diagnostic flag 
; one. 
;
; Assumes boost mode with interrupts disabled.


get_cmd_byte:

const gcb_dsp 0     ; Set to one for debugging.
const gcb_dly 33    ; Bit period for display.
const gcb_nb  11    ; Total number of bits minus start bit.

push F

ld A,(mmu_cmp)      ; Read from command FIFO portal. 
ld (ccmdb),A        ; Store byte in memory.
ld A,gcb_dsp        ; Check if we should display the byte.
and A,bit0_mask     ; If not, jump to end
jp z,gcb_done       ; of routine.

push B              ; Push the two registers we are 
push C              ; going to use.

ld A,(mmu_dfr)      ; Display the start bit.    
or A,bit1_mask      ; and 
ld (mmu_dfr),A      ; wait for
ld A,gcb_dly        ; proscribed delay.
dly A

ld A,gcb_nb         ; We are going to 
push A              ; transmit this number of bits
pop B               ; plus a stop bit.

ld A,(ccmdb)        ; Make a copy of the command
push A              ; byte in 
pop C               ; register C.

gcb_loop:
and A,bit7_mask     ; Check the most significant bit
jp nz,gcb_hi        ; in the remaining command bits.

ld A,(mmu_dfr)      ; Transmit a zero and wait.
and A,bit1_clr
ld (mmu_dfr),A
ld A,gcb_dly
dly A
jp gcb_sl

gcb_hi:      
ld A,(mmu_dfr)       ; Transmit a one and wait.
or A,bit1_mask
ld (mmu_dfr),A
ld A,gcb_dly
dly A

gcb_sl:
push C               ; Get the remaining bits
pop A                ; and shift them to
sla A                ; the left, bringing a zero
push A               ; in for the least significant
pop C                ; bit, copy into C.
dec B                ; Check bit counter and 
jp nz,gcb_loop       ; repeat if still not zero.

pop C                ; Recover C and
pop B                ; B registers.

gcb_done:
ld A,(ccmdb)         ; Load the command byte into A.

pop F
ret

; ------------------------------------------------------------
; Read out, interpret, and execute comands. Uses the global command
; count variable, stimulus and configuration locations, and starts
; and stops stimuli, transmission, battery measurement and
; acknowledgements. The routine assumes that the user program pointer
; is stored in IY upon entry, and will pass IY back after modification.
; The user program needs to be stored from one command to the next
; because we build the user program in stages, writing chunks of code
; to the program memory that must be contiguous.
;
; Assumes the calling routine is running in slow mode with interrupts
; enabled.

cmd_execute:

; Push the flags onto the stack and disable interrupts. Allowing interrupts
; while we are configuring a stimulus or a transmission is more challenging
; than simply turning them off and making sure everything is set up properly
; before returning from this routine and popping the flags off the stack 
; again, restoring the interrupt flag (I) to its prior state. 

push F              ; Push flags.
seti                ; Disable interrupts.

; Now we push A, turn on the fast clock and go into boost, then push all 
; the remaining registers we plan to use.

push A       
ld A,0x03 
ld (mmu_ccr),A 
push B
push C
push D
push E
push H
push L
push IX

; Check the empty flag and abort if it is set. We don't want to try
; to process an empty command. If the empty flag is not set, we
; expect there to be at least two bytes in the command, for the 
; target identifier.

ld A,(mmu_sr)
and A,sr_cme 
jp nz,cmd_done

; Load the device id into HL and the command target id into DE.

ld A,id_hi
push A
pop H
ld A,id_lo
push A
pop L
call get_cmd_byte
push A
pop D
call get_cmd_byte
push A
pop E

; Check to see if the device id and target id are equal. If so, we will
; process the command. If not, we will check if the target id is the
; wildcard.

push L
pop B
push E
pop A
sub A,B
jp nz,cmd_wildcard_check
push H
pop B
push D
pop A
sub A,B
jp nz,cmd_wildcard_check
jp cmd_id_matched

; See if the target id is the wildcard. If so, we will process the command.
; Otherwise we will abort. 

cmd_wildcard_check:
push E
pop A
sub A,0xFF
jp nz,cmd_done
push D
pop A
sub A,0xFF
jp nz,cmd_done

; The device and target idendifiers match, so we are going to process this
; command.

cmd_id_matched:

; The start of our command byte decoding loop. 

cmd_loop:

; Check the command memory empty flag, and if set, we are done.

ld A,(mmu_sr)
and A,sr_cme 
jp nz,cmd_done

; The command memory is a first-in first-out buffer, so we read a 
; byte and store it in memory and in A with get_cmd_byte. After that, 
; we can get the byte by reading it from location ccmdb (copy of 
; command byte).

call get_cmd_byte

; We acknowledge instructions that start and stop long-lasting
; processes. We store the opcode now, in case we need it.

ld (Sack_key),A

; The lamp-off instruction. All we are going to do is turn off 
; the LED.

check_loff:
ld A,(ccmdb)
sub A,op_loff
jp nz,check_loff_end
ld A,0x00             ; Clear A and
ld (mmu_led),A        ; turn off lamp.
call annc_ack         ; Acknowledge.
jp cmd_loop
check_loff_end:

; The lamp-on instruction. We turn on the LED.

check_lon:
ld A,(ccmdb)
sub A,op_lon
jp nz,check_lon_end
ld A,0x01            ; Load a one for bit zero
ld (mmu_led),A       ; and turn on the LED.
call annc_ack        ; Acknowledge.
jp cmd_loop
check_lon_end:

; The impedance measurement off instruction. We open the impedance
; measurement switch.

check_zoff:
ld A,(ccmdb)
sub A,op_zoff
jp nz,check_zoff_end
ld A,0x00             ; Load a zero for bit zero
ld (mmu_msr),A        ; and open the switch.
call annc_ack         ; Acknowledge the stop.
jp cmd_loop
check_zoff_end:

; The impedance measurement on instruction. We close the impedance
; measurement switch.

check_zon:
ld A,(ccmdb)
sub A,op_zon
jp nz,check_zon_end
ld A,0x01            ; Load a one for bit zero
ld (mmu_msr),A       ; and close the switch.
call annc_ack        ; Acknowledge.
jp cmd_loop
check_zon_end:

; Start data transmission.

check_xon:
ld A,(ccmdb)
sub A,op_xon
jp nz,check_xon_end
call get_cmd_byte    ; Read the telemetry channel number
ld (xmit_ch),A       ; and save in memory.
call get_cmd_byte    ; Read xmit period minus one. 

check_xon_255:
ld A,(ccmdb)
sub A,255
jp nz,check_xon_127
ld A,8
ld (x1_txp),A 
ld A,2
ld (x1_mult),A
jp check_xon_go

check_xon_127:
ld A,(ccmdb)
sub A,127
jp nz,check_xon_63
ld A,4
ld (x1_txp),A 
ld A,4
ld (x1_mult),A
jp check_xon_go

check_xon_63:
ld A,(ccmdb)
sub A,63
jp nz,check_xon_31
ld A,2
ld (x1_txp),A 
ld A,8
ld (x1_mult),A
jp check_xon_go

check_xon_31:
ld A,1
ld (x1_txp),A 
ld A,16
ld (x1_mult),A

check_xon_go:
ld A,(xmit_ch)
dec A
and A,bit2_mask
srl A
srl A
ld (mmu_acfg),A
ld A,(x1_txp)        ; Load the transmit period,
dec A                ; decrement,
ld (x1_idx),A        ; and write to the sample index.
ld A,sample_period   ; Load the sample period,
ld (mmu_i4p),A       ; write interrupt timer four.
ld A,(mmu_imsk)      ; Enable interrupt timer four
or A,bit3_mask       ; with bit three of interrupt
ld (mmu_imsk),A      ; mask.
call annc_ack        ; Acknowledge xon.
check_xon_end:

; Stop data transmission.

check_xoff:
ld A,(ccmdb)
sub A,op_xoff
jp nz,check_xoff_end
ld A,0               ; Disable
ld (mmu_i4p),A       ; timer interrupt.
ld A,(mmu_imsk)      ; Mask timer interrupt
and A,bit3_clr       ; with bit three of
ld (mmu_imsk),A      ; interrupt mask.
call annc_ack        ; Acknowledge xoff.
check_xoff_end:

; Battery voltage measurement request instruction. This instruction
; takes no parameters. We call the battery measurement routine
; immediately, which will take about fifty microseconds. Battery 
; measurements are their own acknowledgement, so we do not transmit
; an acknowledgement.

check_battery:
ld A,(ccmdb)
sub A,op_batt
jp nz,check_battery_end
call annc_batt       
jp cmd_loop
check_battery_end:

; Identification request instruction. This instruction takes no
; operands. We call the identification transmission routine, which
; will occupy the CPU for up to 650 ms before transmitting a single
; message that gives the device id to any listeners. The identity 
; broadcast is its own acknowledgement.

check_identify:
ld A,(ccmdb)
sub A,op_id
jp nz,check_identify_end
call annc_id    
jp cmd_loop
check_identify_end:

; Receive user code and load into user program memory. Instruction
; takes one operand: the number of program bytes that follow the
; operand. The bytes will be loaded into the location pointed to 
; by index register IY. If one or more bytes are loaded by this
; instruction into the user program memory, we set the user program
; run flag so as to keep the device powered up to receive more
; user program bytes in future commands. But we disable the user
; program interrupt to make sure that we don't execute a partially-
; loaded program. We do not acknowledge the upload because the
; upload itself does not cause any action.
  
check_pgld:
ld A,(ccmdb)
sub A,op_pgld
jp nz,check_pgld_end
call get_cmd_byte  ; Get the number of program bytes.
add A,0            ; If number of bytes is zero,
jp z,cmd_loop      ; we are done with this instruction.
push A             ; Otherwise, use B to count the
pop B              ; program bytes.
load_prog:        
call get_cmd_byte  ; Read instruction byte from command memory
ld (IY),A          ; and write to program memory.
inc IY             ; Increment memory pointer.
dec B              ; Decrement B, and if not zero, 
jp nz,load_prog    ; read another byte.
ld A,0x01          ; Otherwise, we set the
ld (UPrun),A       ; user program flag to keep the device awake.
ld (UPinit),A      ; Clear the user program initialization flag.
ld A,0             ; Load interrupt three timer with zero
ld (mmu_i3p),A     ; to disable the interrupt.
ld A,(mmu_imsk)    ; Mask interrupt number
and A,bit2_clr     ; three with bit two in the 
ld (mmu_imsk),A    ; interrupt mask.
jp cmd_loop        ; We are done with this instruction.
check_pgld_end:

; Turn on execution of user code by enabling the dedicated user program
; interrupt.

check_pgon:
ld A,(ccmdb)
sub A,op_pgon
jp nz,check_pgon_end
ld A,0x01               ; Set the the user program
ld (UPrun),A            ; run and
ld (UPinit),A           ; initialization flags.
ld A,uprog_tick         ; Set interrupt timer three to 
ld (mmu_i3p),A          ; the uprog_tick period
ld A,(mmu_imsk)         ; and enable interrupt timer
or A,bit2_mask          ; three with bit two of 
ld (mmu_imsk),A         ; the interrupt mask.
call annc_ack           ; Acknowledge enable program.
jp cmd_loop
check_pgon_end:

; Turn off execution of user code, disable user code interrupt.

check_pgoff:
ld A,(ccmdb)
sub A,op_pgoff
jp nz,check_pgoff_end
ld A,0x00               ; Clear the user program 
ld (UPrun),A            ; run and
ld (UPinit),A           ; initialization flags.
ld A,0                  ; Load interrupt three timer with zero
ld (mmu_i3p),A          ; to disable the interrupt.
ld A,(mmu_imsk)         ; Mask interrupt timer three
and A,bit2_clr          ; with bit
ld (mmu_imsk),A         ; two of interrupt mask
call annc_ack           ; Acknowledge disable program.
jp cmd_loop
check_pgoff_end:

; Reset the user program pointer to point to the first byte in user
; program memory. We don't acknowledge this instruction.

check_pgrst:
ld A,(ccmdb)
sub A,op_pgrst
jp nz,check_pgrst_end
ld IY,prog_bot          ; Load IY with the base of
jp cmd_loop             ; user program memory.
check_pgrst_end:

; Shut down the device. We acknowledge but otherwise do nothing.

check_shdn:
ld A,(ccmdb)
sub A,op_shdn
jp nz,check_shdn_end 
call annc_ack        ; Acknowledge shutdown command.
jp cmd_loop
check_shdn_end:

; Version request instruction. This instruction takes no
; operands. We call the version transmit routine.

check_ver:
ld A,(ccmdb)
sub A,op_ver
jp nz,check_ver_end
call annc_ver 
jp cmd_loop
check_ver_end:

; If we get here, the opcode is not valid, so abandon the command.

jp cmd_done     

; Now that we are done with command processing, we reset the command 
; processor.

cmd_done:
ld A,0x01
ld (mmu_cpr),A

; Restore most registers, but not IY, which contains the user program pointer.

pop IX
pop L
pop H
pop E
pop D
pop C
pop B

; Turn of fast clock and move out of boost. Pop the flag register off
; the stack, restores the previous value of the interrupt flag.

ld A,0x00           ; Clear bits zero and one,
ld (mmu_ccr),A      ; Disable TCK and move out of boost.

pop A               
pop F               
ret

; -----------------------------------------------------------------
; The main program. We begin by initializing the device, which
; includes initializing the stack pointer, variables, and interrupts.
; The main program uses IY to store the user program pointer.

main:

; Disable interruupts.
seti

; Initialize the stack pointer.
ld HL,stack_bot
ld SP,HL

; Turn on the lamp and wait. This is the first start-up flash.

ld A,0x01
ld (mmu_led),A
ld A,lon_ms
call delay_ms

; Initialize variable locations to zero.

ld IX,mvar_bot
ld A,num_vars
push A
pop B
ld A,0
main_var_init_loop:
ld (IX),A
inc IX
dec B
jp nz,main_var_init_loop

; Initialize certain variables to values other than zero.

ld A,id_lo         ; Set the primary channel number to the
ld (xmit_ch),A     ; low byte of the device identifier.

; Turn off the lamp. This is the end of the first start-up
; flash.

ld A,0x00
ld (mmu_led),A
ld A,loff_ms
call delay_ms

; Configure control space registers.

ld A,0             ; Prepare zeros to write.
ld (mmu_led),A     ; Turn off Lamp.
ld (mmu_acfg),A    ; Select AC coupling.
ld (mmu_msr),A     ; Turn off impedance measurement.
ld (mmu_dfr),A     ; Clear diagnostic flags to zero.
ld (mmu_i3p),A     ; Disable interrupt timer three.
ld (mmu_i4p),A     ; Disable interrupt timer four.
ld (mmu_imsk),A    ; Mask all interrupts.
ld (x1_idx),A      ; Clear the X1 sample index.
ld (x2_idx),A      ; Clear the X2 sample index.
ld (x3_idx),A      ; Clear the X3 sample index.
ld (x4_idx),A      ; Clear the X4 sample index.
ld (x1_txp),A      ; Clear the X1 sample period.
ld (x2_txp),A      ; Clear the X2 sample period.
ld (x3_txp),A      ; Clear the X3 sample period.
ld (x4_txp),A      ; Clear the X4 sample period.
ld (x1_mult),A     ; Clear the X1 multiplier.
ld (x2_mult),A     ; Clear the X2 multiplier.
ld (x3_mult),A     ; Clear the X3 multiplier.
ld (x4_mult),A     ; Clear the X4 multiplier.
ld A,0xFF          ; Prepare ones to write.
ld (mmu_irst),A    ; Reset all interrupts.
ld A,f_low         ; Write the radio frequency
ld (mmu_rfc),A     ; calibration to the firmware.

; Configure user programming. 

ld IY,prog_bot     ; The main loop uses IY for the user program pointer.
ld A,ret_code      ; Put a return opcode at first byte
ld (IY),A          ; in user program, in case of enable.

; Turn on the lamp. This is the start of the second start-up flash.

ld A,0x01
ld (mmu_led),A
ld A,lon_ms
call delay_ms

; Calibrate the transmit clock.

call calibrate_tck

; Configure the accelerometer (BMA423) and thermometer (TMP117). 

call bma_config
call tmp_single

; Set the period with which we will read out the thermometer, in
; units of interrupt periods.

ld A,tmp_period
ld (tmp_chb),A
ld A,0
ld (tmp_clb),A

; Turn off the lamp. This is the end of the second start-up
; flash.

ld A,0x00
ld (mmu_led),A
ld A,loff_ms
call delay_ms

; Put the CPU into boost mode and call the ADC calibration routine.
; Afterwards, move out of boost. We have to perform this calibration
; in boost mode because the SPI interface works only with TCK.

ld A,0x03 
ld (mmu_ccr),A 
call adc_calib
ld A,0x00
ld (mmu_ccr),A 

; Turn on the lamp. This is the start of the third start-up flash.

ld A,0x01
ld (mmu_led),A
ld A,lon_ms
call delay_ms

; Write a pattern to the scratch area.

ld IX,scratch
ld A,64
push A
pop C
ld A,0
nvm_scratch_init:
ld (IX),A
inc IX
ld (IX),A
inc IX
add A,4
dec C
jp nz,nvm_scratch_init

; Write the scratch area to the NVM.

ld IX,scratch
ld A,0
push A
pop L
push A
pop H
ld A,128
call nvm_wr

; Turn off the lamp. This is the end of the third start-up flash.

ld A,0x00
ld (mmu_led),A

; Enable interrupts.

clri

; ---------------------------------------------------------------
; The main event loop.

main_loop:

; Deal with any pending commands.

ld A,(mmu_sr)       ; Fetch status register.
and A,sr_cmdrdy     ; Check the command ready bit.
jp z,main_nocmd     ; Jump if it's clear,
call cmd_execute    ; execute command if it's set.
main_nocmd:

jp main_loop

; ------------------------------------------------------------
; Include files. These must follow the main and interrupt jumps.

include "../../OSR8/Mult_8.asm"
include "../../OSR8/Sub_8N.asm"
include "../../OSR8/Add_8N.asm"
include "../../OSR8/Copy_8N.asm"
include "../../OSR8/Left_8N.asm"
include "../../OSR8/Right_8N.asm"
include "../../OSR8/Mult_16.asm"
include "../../OSR8/Sqrt_32.asm"
include "I2C.asm"

