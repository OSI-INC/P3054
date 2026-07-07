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
const rf_low         14 ; Radio frequency calibration.

; CPU Address Map Boundaries. The first 256-byte block is  
; the control register space. This is shadowed by RAM so
; that all write-only registers can be read back. The next
; block is the program stack. After that we have the scratch-
; pad block and the program variable block. The total memory
; size is 1 KByte.

const ctrl_bot   0x0000 ; Control Registers
const stack_bot  0x0100 ; Program Stack
const scr_bot    0x0200 ; Scratch-Pad
const var_bot    0x0300 ; Program Variables
const mem_top    0x03FF ; Top of Address Map

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
const mmu_i0p    0x0011 ; Interrupt Zero Period (Write/Readback)
const mmu_i2c00  0x0012 ; i2c SDA=0 SCL=0 (Write/Readback)
const mmu_i2c01  0x0013 ; i2c SDA=0 SCL=1 (Write/Readback)
const mmu_i2cA0  0x0014 ; i2c SDA=A SCL=0 (Write/Readback)
const mmu_i2cA1  0x0015 ; i2c SDA=A SCL=1 (Write/Readback) 
const mmu_i2cZ0  0x0016 ; i2c SDA=Z SCL=0 (Write/Readback)
const mmu_i2cZ1  0x0017 ; i2c SDA=Z SCL=1 (Write/Readback) 
const mmu_i2cMR  0x0018 ; i2c Most Recent Eight Bits (Read)
const mmu_smcr   0x0019 ; Sample Control Register (Write)
const mmu_x1cfg  0x001A ; X1 Configuration (Write/Readback)
const mmu_x2cfg  0x001B ; X2 Configuration (Write/Readback)
const mmu_x3cfg  0x001C ; X3 Configuration (Write/Readback)
const mmu_x4cfg  0x001D ; X4 Configuration (Write/Readback)
const mmu_smemh  0x001E ; ADC Data HI Byte (Read)
const mmu_smeml  0x001F ; ADC Data LO Byte (Read)

; The variable space will be entirely written on start-up by a
; a block read from the NVM. Before copying, however, we check
; to see if the first two bytes match our password.

const key_hb     0x0300 
const key_lb     0x0301

; Variables: Amplifier and sampling configuration. For each 
; of the four biopotential inputs, we have a sample index,
; transmit period, sample storage base address in the sample
; memory, a sample control command that includes the number
; of time the samples must be shifted left before storage,
; and a transmit channel number. The ordering of these in
; memory matches the order in which we use them in the 
; interrupt routine. We dedicate sixteen bytes of variable
; space to each of the four inputs so that we can step 
; through the inputs by adding sixteen to an index register
; and find in memory all the information we need to read,
; shift, and store a sample.

const x1_xpd     0x0310 ; Transmit Period
const x1_idx     0x0311 ; Sample Index
const x1_xch     0x0312 ; Transmit Channel Number
const x2_xpd     0x0313 ; Transmit Period
const x2_idx     0x0314 ; Sample Index
const x2_xch     0x0315 ; Transmit Channel Number
const x3_xpd     0x0316 ; Transmit Period
const x3_idx     0x0317 ; Sample Index
const x3_xch     0x0318 ; Transmit Channel Number
const x4_xpd     0x0319 ; Transmit Period
const x4_idx     0x031A ; Sample Index
const x4_xch     0x031B ; Transmit Channel Number

const dc_in      0x031F ; DC-Coupled Inputs

; Variables: Configuration of the TMP117 temperature sensor. 
; The period is in multiples of 250 ms. We have a two-byte
; timer that is incremented on every interrupt. When its hi
; byte equals the period, we read thesensor and initiate 
; another conversion.

const temp_xch   0x0350 ; Transmit Channel 
const temp_xpd   0x0351 ; Transmit Period
const temp_idx   0x0352 ; Index
const temp_mpd   0x0353 ; Measurement Period
const temp_mth   0x0354 ; Measurement Timer, HI
const temp_mtl   0x0355 ; Measurement Timer, LO
const temp_svh   0x0356 ; Saved, HI
const temp_svl   0x0357 ; Saved, LO

; Variables: Configuration of BMA423 accelerometer. The state 
; is a code for enabled or disabled. The rate is a code for the 
; update frequency. The range is a code for the acceleration 
; dynamic range.

const acc_xch    0x0360 ; Transmit Channel
const acc_xpd    0x0361 ; Transmit Period
const acc_idx    0x0362 ; Index
const bma_state  0x0363 ; Enable or Disable
const bma_rate   0x0364 ; Update Rate
const bma_range  0x0365 ; Dynamic Range

; Variables: NVM readout and transmission. Each transmit
; sample is a new byte read from the NVM, in which the top
; byte is the byte read and the bottom byte is the bottom
; eight bits of the address.

const nvm_xch    0x0370 ; Transmit Channel
const nvm_xpd    0x0371 ; Transmit Period
const nvm_idx    0x0372 ; Index
const nvm_xah    0x0373 ; Transmit Address, HI
const nvm_xal    0x0374 ; Transmit Address, LO

; Variables: Temporary storage locations for particular
; purposes, available globally.

const sack_key   0x03F0 ; Acknowledgement key
const ccmdb      0x03F1 ; Copy of Command Byte

; Constants: Non-volatile memory password.

const nvm_pssh     0xA3
const nvm_pssl     0x62

; Constants: Status Register Bit Masks

const sr_cmdrdy    0x01 ; Command Ready
const sr_rp        0x02 ; Receive Power
const sr_mck       0x04 ; Millisecond Clock
const sr_txa       0x08 ; Transmit Active
const sr_cpa       0x10 ; Command Processor Active
const sr_scbsy     0x20 ; Sample Controller Busy
const sr_cme       0x40 ; Command Memory Empty
const sr_rck       0x80 ; Current Value of Reference Clock

; Constants: Transmit Control Masks

const tx_txi       0x01 ; Assert transmit initiate
const tx_txwp      0x02 ; Assert transmit warm-up

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
const int_period     31 ; Interrupt period in RCK periods
const lon_ms         20 ; Lamp on time in milliseconds
const loff_ms       200 ; Lamp off time in milliseconds
const spd_500ms       2 ; 1/2 s in 256 * interrupt period
const spd_1000ms      4 ; 1 s in 256 * interrupt period
const spd_2000ms      8 ; 2 s in 256 * interrupt period

; Constants: Auxiliary message types.

const at_id           1 ; Identification
const at_ack          2 ; Acknowledgements
const at_batt         3 ; Battery Measurement
const at_conf         4 ; Confirmation
const at_ver          5 ; Version

; Constants: Command Codes

const op_id           5 ;
const op_ver         11 ; 
const op_ton         16 ; 
const op_toff        17 ;
const op_zon         18 ; 
const op_zoff        19 ;
const op_nvmwr       20 ;
const op_lon         30 ;
const op_loff        31 ; 

; Constants: Non-Volatile Memory. The M24C16 EEPROM provides 
; 2K x 8 of NVM with an I2C interface. We can read 1-2048 
; bytes in one read cycle. We can write 16 bytes in one write 
; cycle. The device address consists only of four bits, the lower 
; three bits of the seven-bit I2C address are used to select one 
; of eight 256-byte blocks within the EEPROM. We dedicate these
; blocks to various functions.

const nvm_addr     0x50 ; Device address.
const nvm_amask    0x07 ; Mask for bottom three bits.
const nvm_calib    0x00 ; Calibration block, bytes 0x000-0x0FF.
const nvm_config   0x01 ; Configuration block, bytes 0x100-0x1FF.
const nvm_notes    0x02 ; User note blocks, bytes 0x200-0x3FF.
const nvm_history  0x04 ; Usage history blocks, bytes 0x400-0x7FF.

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
const bma_sdly        2 ; Startup delay in milliseconds

; Constants: for use with the Sample Controller.

const adc_shift0   0x00 ; No Shift Left
const adc_shift1   0x01 ; One Shift Left
const adc_shift2   0x02 ; Two Shifts Left
const adc_shift3   0x03 ; Three Shifts Left
const adc_shift4   0x04 ; Four Shifts Left
const adc_x4ss     0x0C ; Four Shifts Left, Single Sample
const acc_rst      0x04 ; Reset the Accumulator
const sm_wrcpu     0x08 ; Write to Sample Memory
const sm_x1        0x00 ; X1 Sample Memory Location
const sm_x2        0x01 ; X2 Sample Memory Location
const sm_x3        0x02 ; X3 Sample Memory Location
const sm_x4        0x03 ; X4 Sample Memory Location

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

pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Write 16N bytes to the non-volatile memory (NVM), where N is 
; 1-128, making a total write size of 16 to 2048 bytes. In IX 
; we pass a pointer to the first byte to be written. In A we
; pass N. In HL we pass the eleven-bit sub-address at which the 
; write should begin. The write must begin on a sixteen-byte 
; boundary, for that is the EEPROM page size. The routine clears 
; the bottom four bits of the sub-address to ensure writing to a 
; page boundary. Upon return, IX points to the location after the 
; last byte written and the sub-address in HL points to the start 
; of the next page in NVM.
;
; Can run in slow or boost mode. If interrupted, the interrupt
; service routine must not call any I2C routine.

nvm_wr:

push F
push A
push B
push C
push D

push A
pop D

ld A,16
push A
pop C

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
ld IX,temp_svh
call i2c_rd
dec IX
dec IX
ld A,(IX)
add A,off_16bs   
ld (IX),A

; Initiate a conversion, which we do by writing two bytes to
; the sensor's configuration register. We first put these 
; two bytes in the scratch-pad so that the i2c routine will 
; read them from main program memory and write them to the
; sensor. We already have the sensor address in H. We load
; the configuration register address into L.

ld A,tmp_creg
push A
pop L
ld IX,scr_bot
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
ld IX,scr_bot
ld A,bma_pwrsv
ld (IX),A
ld A,1
push A
pop C
call i2c_wr

; Enable or disable data acquisition by writing to PWR_CTRL 
; register.

ld A,bma_pctrl
push A
pop L
ld A,(bma_state)
ld (IX),A
ld A,1
push A
pop C
call i2c_wr

; Wait for the sensor to configure.

ld A,bma_sdly
call delay_ms 

; Configure update rate by writing to ACC_CONF.

ld A,bma_aconf 
push A
pop L
ld A,(bma_rate)
ld (IX),A
ld A,1
push A
pop C
call i2c_wr

; Configure range by writing to ACC_RANGE.

ld A,bma_arange
push A
pop L
ld A,(bma_range)
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

; Handle the transmit interrupt.

int_xmit:

ld A,(mmu_irqb)     ; Read the interrupt request bits
and A,bit0_mask     ; and test bit zero,
jp z,int_xmit_done  ; skip transmit if not set.

ld A,bit0_mask      ; Reset this interrupt
ld (mmu_irst),A     ; with the bit zero mask.

; Transmit a temperature measurement. After that, we decrement
; the two-byte temperature period counter. When it reaches zero,
; we update the temperature measurement and reset the counter.

int_xmit_temp:
ld A,(temp_xpd)
add A,0
jp z,int_xmit_temp_done
ld A,(temp_idx)
add A,0
dec A
ld (temp_idx),A
jp nz,int_xmit_temp_done
ld A,(temp_xpd)
ld (temp_idx),A
ld A,(temp_xch)
ld (mmu_xch),A
ld A,(temp_svh)
ld (mmu_xhb),A
ld A,(temp_svl)
ld (mmu_xlb),A
ld A,tx_txi
ld (mmu_xcr),A
ld A,tx_delay 
dly A
ld A,(temp_mtl)
sub A,1
ld (temp_mtl),A
ld A,(temp_mth)
sbc A,0
ld (temp_mth),A
jp p,int_xmit_temp_done
ld A,(temp_mpd)
ld (temp_mth),A
ld A,0
ld (temp_mtl),A
call tmp_single
int_xmit_temp_done:

; Transmit an accelerometer measurement. The byte ordering
; on the accelerometer is little-endian.

int_xmit_acc:
ld A,(acc_xpd)
add A,0
jp z,int_xmit_acc_done
ld A,(acc_idx)
dec A
ld (acc_idx),A
jp nz,int_xmit_acc_done
ld A,(acc_xpd)
ld (acc_idx),A
ld A,bma_addr
push A
pop H
ld A,bma_x
push A
pop L
ld IX,scr_bot
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
ld A,(acc_xch)
ld (mmu_xch),A
ld A,tx_txi 
ld (mmu_xcr),A
ld A,tx_delay 
dly A
int_xmit_acc_done:

; Transmit a byte read from the non-volatile memory (NVM). 
; We transmit bytes read consecutively. We put the byte 
; value in the top eight transmit sample bits. We put the 
; bottom eight bits of the NVM address in the bottom eight 
; transmit sample bits.

int_xmit_nvm:
ld A,(nvm_xpd)
add A,0
jp z,int_xmit_nvm_done
ld A,(nvm_idx)
dec A
ld (nvm_idx),A
jp nz,int_xmit_nvm_done
ld A,(nvm_xpd)
ld (nvm_idx),A
ld A,(nvm_xal)
add A,1
ld (nvm_xal),A
push A
pop L
ld A,(nvm_xah)
adc A,0
and A,0x07
ld (nvm_xah),A
or A,nvm_addr
push A
pop H
ld IX,scr_bot
ld A,1
push A
pop C
call i2c_rd
dec IX
ld A,(IX)
ld (mmu_xhb),A
push L
pop A
ld (mmu_xlb),A
ld A,(nvm_xch)
ld (mmu_xch),A
ld A,tx_txi 
ld (mmu_xcr),A
ld A,tx_delay 
dly A
int_xmit_nvm_done:

; Transmit accumulated X1-X4 samples.

int_xmit_x:

; The first input is X1. We store the address of its transmit
; period in IX. We will be incrementing and decrementing IX
; to access the sample control variables. We will keep in B
; the sample memory address of this input.
 
ld IX,x1_xpd
ld A,sm_x1
push A
pop B

; This loop assumes IX contains the address of one of the
; transmit period for one of X1-X4 and also that (adc_idx) 
; contains a the sample address of the same input.

int_xmit_x_loop:

; If the transmit period is zero, the input is disabled, so 
; move to the next input. Note that IX is pointing at this
; input's transmit period.

ld A,(IX)
add A,0
jp z,int_xmit_x_next

; Decrement the sample index. If it is not yet zero after the
; decrement, we are not ready to transmit. Decrement IX so it
; points to the transmit period and move on to the next input.

inc IX ; Sample Index
ld A,(IX)
dec A
ld (IX),A
dec IX ; Transmit Period
jp nz,int_xmit_x_next

; Set the sample index equal to the transmit period.

ld A,(IX)
inc IX ; Sample Index
ld (IX),A

; Select this input's sample in the sample memory.

push B
pop A
ld (mmu_smcr),A

; Make sure the transmitter is not busy.

int_xmit_x_txw:
ld A,(mmu_sr)
and A,sr_txa
jp nz,int_xmit_x_txw

; Read the accumulated sample provided by the sample memory
; and write to the sample transmitter.

ld A,(mmu_smemh)
ld (mmu_xhb),A
ld A,(mmu_smeml)
ld (mmu_xlb),A

; Load the telemetry channel number into the sample transmitter.

inc IX ; Telemetry Channel
ld A,(IX)
ld (mmu_xch),A

; Transmit the message.

ld A,tx_txi 
ld (mmu_xcr),A

; Reset the Sample Accumulator so that its output is zero, then
; store that zero in the sample memory, so we are ready to start
; accumulating the next sample. We must construct the two commands
; that reset and store by combining the sample memory address,
; which is in B, and the correct control bits.

ld A,acc_rst
or A,B
ld (mmu_smcr),A
ld A,sm_wrcpu
or A,B
ld (mmu_smcr),A

; Done with this input, time to move on to the next one, but 
; first move IX back to point to this channels transmit period.

dec IX ; Sample Index
dec IX ; Sample Period

int_xmit_x_next:

; Check the sample address. If it is equal to address of the final
; input we are done with all of them.

ld A,sm_x4
sub A,B
jp z,int_xmit_x_done

; Prepare for the next input. We increment the sample address
; and move IX from this input's period to the next input's period.

inc B
inc IX
inc IX
inc IX
jp int_xmit_x_loop

; Done with ADC readout and transmission.

int_xmit_x_done:

; Done with transmit interrupt. Make sure we are not transmitting before
; we leave.

int_xmit_done_txw:
ld A,(mmu_sr)
and A,sr_txa
jp nz,int_xmit_done_txw

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

ld A,(sack_key)
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

ld A,id_hi          ; Load HI byte id identifier 
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
; acknowledgements.
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

; Now we push A, turn on the fast clock and go into boost.

push A       
ld A,0x03 
ld (mmu_ccr),A 

; Push all the other registers. We reserve the right to use any
; and all of them in this routine.

push B
push C
push D
push E
push H
push L
push IX
push IY

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

; The command memory is a first-in first-out buffer. We use our
; get_cmd_byte routine to bring the next byte into A and also to
; store that same byte in location ccmdb (copy of command byte).

call get_cmd_byte

; We acknowledge instructions that start and stop long-lasting
; processes. We store the opcode now, in case we need it.

ld (sack_key),A

; The lamp-off instruction. Turn off the LED.

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
; measurement switch so that the unipolar input ground is equal to
; the amplifier zero-volt potential.

check_zoff:
ld A,(ccmdb)
sub A,op_zoff
jp nz,check_zoff_end
ld A,(mmu_acfg)
and A,bit1_clr
ld (mmu_acfg),A
call annc_ack 
jp cmd_loop
check_zoff_end:

; The impedance measurement on instruction. We close the impedance
; measurement switch so that the unipolar input ground is shifted
; downwards with respect to the amplifier zero-volt potential.

check_zon:
ld A,(ccmdb)
sub A,op_zon
jp nz,check_zon_end
ld A,(mmu_acfg)
or A,bit1_mask
ld (mmu_acfg),A       
call annc_ack 
jp cmd_loop
check_zon_end:

; Start telemetry protocol. We un-mask interrupt one, which
; starts the Telemetry Manager, which in turn will generate
; the number-one interrupt.

check_ton:
ld A,(ccmdb)
sub A,op_ton
jp nz,check_ton_end
ld A,(mmu_imsk)      
or A,bit0_mask
ld (mmu_imsk),A
call annc_ack
check_ton_end:

; Stop telemetry protocol. Mask interrupt one, which stops
; the Telemetry Manager, so interrupt one will no longer
; be generated.

check_toff:
ld A,(ccmdb)
sub A,op_toff
jp nz,check_toff_end
ld A,(mmu_imsk)    
and A,bit0_clr     
ld (mmu_imsk),A    
call annc_ack  
check_toff_end:

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

; Write bytes to the non-volatile memory at a particular address.
  
check_nvmwr:
ld A,(ccmdb)
sub A,op_nvmwr
jp nz,check_nvmwr_end
call get_cmd_byte  ; Get the number of program bytes.
push A             ; and move
pop B              ; to B.
call get_cmd_byte  ; Read the upper address byte.
push A             ; and move
pop H              ; to H.
call get_cmd_byte  ; Read the lower address byte.
push A             ; and move
pop L              ; to L.
push B             ; Bring back 
pop A              ; the number of bytes.
add A,0            ; If this number is zero,
jp z,cmd_loop      ; we are done with this instruction.
push A             ; Make a copy of the number of
pop E              ; bytes for later.
ld IX,scr_bot      ; Point IX to the scratch-pad.
load_prog:        
call get_cmd_byte  ; Read instruction byte from command memory
ld (IX),A          ; and write to the scratch-pad.
inc IX             ; Increment memory pointer.
dec B              ; Decrement B, and if not zero, 
jp nz,load_prog    ; read another byte.
ld IX,scr_bot      ; Preapare to write the
push E             ; values to the NVM.
pop A              ; We divide the number of
srl A              ; bytes by sixteen using
srl A              ; shift-right logical to get the
srl A              ; number of sixteen-byte
srl A              ; pages to be written.
call nvm_wr        ; Call the write routine and
call annc_ack      ; acknowledge.
jp cmd_loop        ; We are done with this instruction.
check_nvmwr_end:

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

; Restore registers.

pop IY
pop IX
pop L
pop H
pop E
pop D
pop C
pop B

; Turn of fast clock and move out of boost. 

ld A,0x00           ; Clear bits zero and one,
ld (mmu_ccr),A      ; Disable TCK and move out of boost.

; Pop A and F off
; the stack. When we restore F, we restore the previous value of the
; interrupt flag.
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

; Turn on the lamp and wait. This is the start-up flash.

ld A,0x01
ld (mmu_led),A
ld A,lon_ms
call delay_ms

; Move into boost to speed up initialization.

ld A,0x03 
ld (mmu_ccr),A 

; Configure control space registers.

ld A,0             ; Prepare zeros to write.
ld (mmu_acfg),A    ; Select AC coupling and Xoff.
ld (mmu_dfr),A     ; Clear diagnostic flags
ld (mmu_imsk),A    ; Mask all interrupts.
ld A,0xFF          ; Prepare ones to write.
ld (mmu_irst),A    ; Reset all interrupts.
ld A,rf_low        ; Write the radio frequency
ld (mmu_rfc),A     ; calibration to the firmware.
ld (mmu_i2cZ1),A   ; Set I2C's SDA to Z.

; Configure analog inputs.

ld A,1
ld (x1_xpd),A
ld (x1_idx),A
ld A,adc_shift3
ld (mmu_x1cfg),A

ld A,2
ld (x2_xpd),A
ld (x2_idx),A
ld A,adc_shift2
ld (mmu_x2cfg),A

ld A,4
ld (x3_xpd),A
ld (x3_idx),A
ld A,adc_shift1
ld (mmu_x3cfg),A

ld A,8
ld (x4_xpd),A
ld (x4_idx),A
ld A,adc_x4ss
ld (mmu_x4cfg),A

ld A,69
ld (x1_xch),A  
ld A,70
ld (x2_xch),A
ld A,71
ld (x3_xch),A
ld A,72
ld (x4_xch),A

ld A,0x01
ld (dc_in),A
ld (mmu_acfg),A

ld A,2
ld (mmu_i0p),A

; Configure the TMP117 temperature sensor and its telemetry
; channel.

ld A,9
ld (temp_xch),A
ld A,0
ld (temp_xpd),A
ld (temp_idx),A
ld A,0
ld (temp_svh),A
ld (temp_svl),A
ld A,spd_1000ms
ld (temp_mpd),A
ld (temp_mth),A
ld A,0
ld (temp_mtl),A
call tmp_single

; Configure the BMA423 accelerometer and its telemetry
; channel.

ld A,14
ld (acc_xch),A
ld A,0
ld (acc_xpd),A
ld (acc_idx),A
ld A,bma_disable
ld (bma_state),A
ld A,bma_1hz
ld (bma_rate),A
ld A,bma_2g
ld (bma_range),A
call bma_config

; Configure the non-volatile memory telemetry channel.

ld A,13
ld (nvm_xch),A
ld A,0
ld (nvm_xpd),A
ld (nvm_idx),A
ld A,0
ld (nvm_xah),A
ld (nvm_xal),A

; Turn off the lamp and move out of boost. This is the end of the start-up flash.

ld A,0x00
ld (mmu_led),A
ld A,0x00
ld (mmu_ccr),A 

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

