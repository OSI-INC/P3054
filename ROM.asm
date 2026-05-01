; A3054 Intraperitoneal Transmitter (IPT) Program
; -----------------------------------------------

; This code runs in an OSR8 microprocessor.

; Configuration Constants.
const version          1 ; The firmwarwe version.
const identifier_hi 0xAA ; 0-255, no restrictions
const identifier_lo 0x55 ; 0-255, low nibble cannot be 0x0 or 0xF 
const frequency_low   24 ; Radio frequency calibration.

; CPU Address Map Boundary Constants
const mvar_bot  0x0000 ; Bottom of Main Variable Space
const stack_bot 0x0100 ; Bottom of Program Stack
const uvar_bot  0x0200 ; Bottom of User Variable Space
const ctrl_bot  0x0400 ; Bottom Control Register Space
const prog_bot  0x0800 ; Bottom of User Program Memory
const prog_top  0x0FFF ; Top of User Program Memory

; Address Map Locations
const mmu_irqb  0x0400 ; Interrupt Request Bits (Read)
const mmu_imsk  0x0401 ; Interrupt Mask Bits (Read/Write)
const mmu_irst  0x0402 ; Interrupt Reset Bits (Write)
const mmu_acfg  0x0403 ; Amplifier Configuration (Write)
const mmu_stc   0x0404 ; Stimulus Current (Write)
const mmu_rst   0x0405 ; System Reset (Write)
const mmu_xhb   0x0406 ; Transmit HI Byte (Write)
const mmu_xlb   0x0407 ; Transmit LO Byte (Write)
const mmu_xch   0x0408 ; Transmit Channel Number (Write)
const mmu_xcr   0x0409 ; Transmit Control Register (Write)
const mmu_rfc   0x040A ; Radio Frequency Calibration (Write)
const mmu_ccr   0x040B ; Clock Control Register (Write)
const mmu_dfr   0x040C ; Diagnostic Flag Register (Read/Write)
const mmu_sr    0x040D ; Status Register (Read)
const mmu_cmp   0x040E ; Command Memory Portal (Read)
const mmu_cpr   0x040F ; Command Processor Reset (Write)
const mmu_i1ph  0x0410 ; Interrupt Timer One Period HI (Write)
const mmu_i1pl  0x0411 ; Interrupt Timer One Period LO (Write)
const mmu_i2ph  0x0412 ; Interrupt Timer Two Period HI (Write)
const mmu_i2pl  0x0413 ; Interrupt Timer Two Period LO (Write)
const mmu_i3p   0x0414 ; Interrupt Timer Three Period (Write)
const mmu_i4p   0x0415 ; Interrupt Timer Four Period (Write)
const mmu_i2c00 0x0416 ; i2c SDA=0 SCL=0 (Write)
const mmu_i2c01 0x0417 ; i2c SDA=0 SCL=1 (Write)
const mmu_i2cA0 0x0418 ; i2c SDA=A SCL=0 (Write)
const mmu_i2cA1 0x0419 ; i2c SDA=A SCL=1 (Write) 
const mmu_i2cZ0 0x041A ; i2c SDA=Z SCL=0 (Write)
const mmu_i2cZ1 0x041B ; i2c SDA=Z SCL=1 (Write) 
const mmu_i2cMR 0x041C ; i2c Most Recent Eight Bits (Read)
const mmu_adc 0x041D ; SPI Control Register (Write)
const mmu_spidh 0x041E ; SPI Data MSB (Read)
const mmu_spidl 0x041F ; SPI Data LSB (Read)

; Status Bit Masks, for use with status register
const sr_cmdrdy  0x01 ; Command Ready Flag
const sr_entck   0x02 ; Transmit Clock Enabled
const sr_saa     0x04 ; Sensor Access Active Flag
const sr_txa     0x08 ; Transmit Active Flag
const sr_cpa     0x10 ; Command Processor Active
const sr_boost   0x20 ; Boost Flag
const sr_cme     0x40 ; Command Memory Empty
const sr_rck     0x80 ; RCK State

; Transmit Control Masks, for use with tansmit control register
const tx_txi     0x01 ; Assert transmit initiate
const tx_txwp    0x02 ; Assert transmit warm-up

; Auxiliary message types.
const at_id         1 ; Identification
const at_ack        2 ; Acknowledgements
const at_batt       3 ; Battery Measurement
const at_conf       4 ; Confirmation
const at_ver        5 ; Version

; Bit Masks
const bit0_mask  0x01 ; Bit Zero Mask
const bit1_mask  0x02 ; Bit One Mask
const bit2_mask  0x04 ; Bit Two Mask
const bit3_mask  0x08 ; Bit Three Mask
const bit4_mask  0x10 ; Bit Four Mask
const bit5_mask  0x20 ; Bit Five Mask
const bit6_mask  0x40 ; Bit Six Mask
const bit7_mask  0x80 ; Bit Seven Mask
const bit0_clr   0xFE ; Bit Zero Clear
const bit1_clr   0xFD ; Bit One Clear
const bit2_clr   0xFB ; Bit Two Clear
const bit3_clr   0xF7 ; Bit Three Clear
const bit4_clr   0xEF ; Bit Four Clear
const bit5_clr   0xDF ; Bit Five Clear
const bit6_clr   0xBF ; Bit Six Clear
const bit7_clr   0x7F ; Bit Seven Clear

; Timing Constants.
const min_tcf       72  ; Minimum TCK periods per half RCK period
const tx_delay      40  ; Wait time for sample transmission, TCK periods
const sa_delay      30  ; Wait time for sensor access, TCK periods
const wp_delay     255  ; Warm-up delay for auxiliary messages
const num_vars      64  ; Number of vars to clear at start
const initial_tcd   15  ; Max possible value of TCK divisor
const uprog_tick   163  ; User program interrupt period minus one
const id_delay      33  ; To pad id delay to 50 TCK periods
const min_int_p     25  ; Minimum transmit period
const shdn_rst     250  ; Shutdown counter reset value.

; Stimulus Control Variables
const Scurrent    0x0000 ; Stimulus Current
const Spulse1     0x0001 ; Pulse Length, HI
const Spulse0     0x0002 ; Pulse Length, LO
const Sint1       0x0003 ; Interval Length, HI
const Sint0       0x0004 ; Interval Length, LO
const Slen1       0x0005 ; Stimulus Length, HI
const Slen0       0x0006 ; Stimulus Length, LO
const Srand       0x0007 ; Random pulse timing
const Srun        0x0008 ; Run stimulus
const Spulse      0x0009 ; Stimulus Pulse Run Flag
const Sack_key    0x000A ; Acknowledgement key
const Sdly1       0x000B ; Stimulus Delay Byte One
const Sdly0       0x000C ; Stimulus Delay Byte Zero
const Sdelay      0x000D ; Stimulus Delay Run Flag
const Smaxdly1    0x000E ; Max Delay, HI
const Smaxdly0    0x000F ; Max Delay, LO

; Command Decode Variables
const ccmdb       0x0016 ; Copy of Command Byte

; Shutdown counter.
const shdncnt1    0x0019 ; Counter Byte One
const shdncnt0    0x001A ; Counter Byte Zero

; Random Number Variables
const rand_1      0x0020 ; Random Number Byte One
const rand_0      0x0021 ; Random Number Byte Zero

; User Program Control Variables
const UPrun       0x0022 ; Running
const UPinit      0x0023 ; Initialize

; Transmission Control Variables
const xmit_p      0x0028 ; Transmit Period
const xmit_ch     0x0029 ; Telemetry Channel Number

; User Program Constants
const ret_code      0x0A ; Return from subroutine instruction

; Operation Codes
const op_stop        0 ; 0 operands
const op_start       1 ; 8 operands
const op_xon         2 ; 2 operand
const op_xoff        3 ; 0 operands
const op_batt        4 ; 0 operands
const op_id          5 ; 0 operands
const op_pgld        6 ; 1 operand, variable data
const op_pgon        7 ; 0 operands
const op_pgoff       8 ; 0 operands
const op_pgrst       9 ; 0 operands
const op_shdn       10 ; 0 operands
const op_ver        11 ; 0 operands

; Synchronization.
const synch_nostim  32 ; 
const synch_stim    96 ;

; Non-Volatile Memory Constants. The M24C16 EEPROM provides 2K x 8 
; of NVM with an I2C interface. We can read 1-2048 bytes in one
; read cycle. We can write 1-16 bytes in one write cycle. The device
; address consists only of four bits, the lower three bits of the
; seven-biit I2C address are used to select one of eight 256-byte
; blocks within the EEPROM.
const nvm_addr       0x50 ; I2C address, top four bits only.
const nvm_hmask      0x0F ; 

; TMP117 Temperature Sensor Constants. The tmp provides
; sixteen-bit read and write registers to the I2C bus.
const tmp_addr   0x49 ; I2C address
const tmp_treg   0x00 ; Temperature register
const tmp_creg   0x01 ; Configuration register
const tmp_oneh   0x0C ; For Configuration, one-shot measurement, MSB
const tmp_onel   0x00 ; For Configuration, one-shot measurement, LSB
const tmp_fasth  0x00 ; For Configuration, fast measurement, MSB
const tmp_fastl  0x00 ; For Configuration, fast measurement, LSB

; BMA423 Accelerometer Addresses and Configuration Values. We give
; the internal addresses of registers. When the address is always 
; for a two-byte read, we say so.
const bma_addr     0x18 ; I2C address
const bma_id       0x00 ; Identifier register
const bma_x        0x12 ; ACC_X register, two bytes
const bma_y        0x14 ; ACC_Y register, two bytes
const bma_z        0x16 ; ACC_Z register, two bytes
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
const bma_25hz     0x06 ; For ACC_CONF, 25 Hz, no averaging, no filter
const bma_100hz    0x08 ; For ACC_CONF, 100 Hz, no averaging, no filter
const bma_enable   0x04 ; For PWR_CTRL, enable data acquisition.
const bma_pwrsv    0x03 ; For PWR_CONF, power save, fifo self-start.
const bma_sdly       50 ; Startup delay in RCK periods.

; ADS7052 Analog to Digital Converter Constants.
const ads_rd1      0x34 ; Read ADC1
const ads_rd2      0x35 ; Read ADC2
const ads_rd3      0x36 ; Read ADC3
const ads_rd4      0x37 ; Read ADC4
const ads_cal1     0x0C ; Calibrate ADC1
const ads_cal2     0x0D ; Calibrate ADC2
const ads_cal3     0x0E ; Calibrate ADC3
const ads_cal4     0x0F ; Calibrate ADC4
const ads_rdly       16 ; ADC Read Delay
const ads_cdly       22 ; ADC Calib Delay

; Random Number Generator.
const rand_taps   0xB4 ; Determines which taps to XOR.

; Math constants.
const off_16bs    0x80 ; Convert sixteen bit signed to unsigned.


; ------------------------------------------------------------
; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.

start:

jp main
jp interrupt

; ------------------------------------------------------------
; Include files. These must follow the main and interrupt jumps.

include "I2C.asm"
          

; ------------------------------------------------------------
; Calibrate the transmit clock frequency. We take the CPU out
; of boost, turn off the transmit clock, and repeat a cycle of
; setting the transmit clock divisor and running the transmit
; clock to measure its frequency. Eventually we get a diviso
; that provides a transmit period in the range 195-215 ns. We
; leave the transmit clock off at the end.

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
; about 250 nA after the single measurement.

tmp_single:

; Push flags and registers.

push F
push A
push B
push C
push H
push L

ld A,tmp_addr
push A
pop H
ld A,tmp_creg
push A
pop L
ld A,tmp_oneh
push A
pop C
ld A,tmp_onel
push A
pop B
call i2c_wr16

pop L
pop H
pop C
pop B
pop A
pop F
ret

; ------------------------------------------------------------
; Configure the temperature sensor for fast temperature and
; continuous temperature measurements.

tmp_fast:

; Push flags and registers.

push F
push A
push B
push C
push H
push L

ld A,tmp_addr
push A
pop H
ld A,tmp_creg
push A
pop L
ld A,tmp_fasth
push A
pop C
ld A,tmp_fastl
push A
pop B
call i2c_wr16

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
; accelerometer will deliver zeros if we deviate from the correct sequence.

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

; Specify the PWR_CONF register with its sub-address in L and 
; a value in C. We are configuring for power-save mode.

ld A,bma_pconf
push A
pop L
ld A,bma_pwrsv
push A
pop C
call i2c_wr8

; Enable data acquisition by writing to the PWR_CTRL register.

ld A,bma_pctrl
push A
pop L
ld A,bma_enable
push A
pop C
call i2c_wr8

; Wait for the sensor to configure.

ld A,bma_sdly
dly A 

; Configure the accelerometer for 100 Hz update.

ld A,bma_aconf 
push A
pop L
ld A,bma_100hz
push A
pop C
call i2c_wr8

; Configure for +-16 g range.

ld A,bma_arange
push A
pop L
ld A,bma_16g
push A
pop C
call i2c_wr8

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
; not effective after the first sample is taken.

ads_calib:

push F
push A

ld A,ads_cal1
ld (mmu_adc),A
ld A,ads_cdly
ld A,ads_cal2
ld (mmu_adc),A
ld A,ads_cdly
ld A,ads_cal3
ld (mmu_adc),A
ld A,ads_cdly
ld A,ads_cal4
ld (mmu_adc),A
ld A,ads_cdly

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

; Handle the stimulus interval interrupt. We decrement the stimulus
; interval counter. We start a delay or a pulse using interrupt timer
; two. We set and clear the stimulus delay, pulse, and run flags. The
; first interval of a stimulus should start immediately after reception
; of command, so the initial value of the interrupt period is one. We
; correct this value in the handler.

int_sii:

ld A,(mmu_irqb)     ; Read the interrupt request bits
and A,bit0_mask     ; and test bit zero.
jp z,int_sii_done   ; If not set, skip this interrupt.

ld A,(Srun)         ; Check the Srun flag. 
add A,0             ; If it's been cleared,
jp z,int_sii_done   ; we do nothing.

ld A,(Sint1)        ; Set the stimulus interval delay to
ld (mmu_i1ph),A     ; the value specified by the most 
ld A,(Sint0)        ; recent command, or adapt to value written
ld (mmu_i1pl),A     ; by user program to interval length locations.
ld A,(Slen0)        ; Load LO byte of stimulus length counter.
sub A,1             ; decrement
ld (Slen0),A        ; save,
ld A,(Slen1)        ; Load HI byte,
sbc A,0             ; apply carry bit
ld (Slen1),A        ; and save.
jp nc,int_sii_do    ; If >=0, start a delay or a pulse.

ld A,0              ; If <0, 
ld (mmu_stc),A      ; turn off the stimulus current.
ld (Srun),A         ; Clear the Srun,
ld (Spulse),A       ; Spulse,
ld (Sdelay),A       ; and Sdelay flags.
ld (mmu_i1ph),A     ; Disable the interrupt
ld (mmu_i1pl),A     ; timer.
ld A,(mmu_imsk)     ; Mask the the 
and A,bit0_clr      ; timer
ld (mmu_imsk),A     ; interrupt.
ld A,op_stop        ; Transmit a stimulus stop
ld (Sack_key),A     ; acknowledgement
call annc_ack       ; to mark stimulus end.
jp int_sii_rst

int_sii_do:
ld A,1              ; Set the
ld (Spulse),A       ; pulse flag.
ld A,0              ; Clear the delay
ld (Sdelay),A       ; flag.
ld A,(Scurrent)     ; Load stimulus current and
ld (mmu_stc),A      ; turn on the stimulus.
ld A,(Spulse1)      ; Set interrupt timer 
ld (mmu_i2ph),A     ; two period to the
ld A,(Spulse0)      ; pulse
ld (mmu_i2pl),A     ; length.
ld A,(mmu_imsk)     ; Unmask 
or A,bit1_mask      ; Timer Two
ld (mmu_imsk),A     ; interrupt.
ld A,bit1_mask      ; Reset
ld (mmu_irst),A     ; Timer Two
jp int_sii_rst

int_sii_rst:
ld A,bit0_mask      ; Reset interrupt timer one, which clears
ld (mmu_irst),A     ; its interrupt bit and reloads its timer.

int_sii_done:

; Handle the delay and pulse interrupt, which is generated by 
; Timer Two. The Sdelay flag tells us if the interrupt marks
; the end of a delay or the end of a pulse. We either start
; a pulse or end a pulse.

int_sdp:

ld A,(mmu_irqb)     ; Read the interrupt request bits
and A,bit1_mask     ; and test bit one,
jp z,int_sdp_done   ; skip if not delay and pulse interrupt.
ld A,(Sdelay)       ; Check delay flag
add A,0             ; and if set, end delay and start pulse
jp z,int_sdp_pulse  ; otherwise end pulse.

int_sdp_delay:
ld A,(Scurrent)     ; Turn on the 
ld (mmu_stc),A      ; stimulus current.
ld A,(Spulse1)      ; Load timer two
ld (mmu_i2ph),A     ; with the
ld A,(Spulse0)      ; pulse 
ld (mmu_i2pl),A     ; length.
ld A,0              ; Clear the
ld (Sdelay),A       ; delay flag.
ld A,1              ; Set the
ld (Spulse),A       ; pulse flag.
jp int_sdp_rst

int_sdp_pulse:
ld A,0              ; Stop the
ld (mmu_stc),A      ; stimulus pulse.
ld (Spulse),A       ; Clear pulse flag.
ld (Sdelay),A       ; And the delay flag.
ld (mmu_i2ph),A     ; Disable the timer two
ld (mmu_i2pl),A     ; interrupt.
ld A,(mmu_imsk)     ; Mask 
and A,bit1_clr      ; interrupt
ld (mmu_imsk),A     ; two.

int_sdp_rst:
ld A,bit1_mask      ; Reset Timer Two, which loads the
ld (mmu_irst),A     ; the current period into its counter.

int_sdp_done:

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
; temperature sensor, EEPROM, and accelerometer in this
; prototype code, so we can read any of them one at a time.
; The selection will be based upon the lower nibble of the
; channel number, which has range 1-14. Channels 1-8 are for
; the inputs AC/DC. Channel 9 for temperature. Channel 10 for
; EEPROM, 11 for accelerometer, and 12-14 for synch.

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
jp int_xmit_sync

; Transmit a temperature measurement.

int_xmit_temp:
ld A,tmp_addr
push A
pop H
ld A,tmp_treg
push A
pop L
call i2c_rd16
push C
pop A
add A,off_16bs   
ld (mmu_xhb),A
push B
pop A
ld (mmu_xlb),A
jp int_xmit_rdy

; Read a sample from an ADC and transmit. We apply AC coupling for
; channel numbers for which the lower nibble is 1-4 and DC coupling
; for lower nibble 5-8. We read ADCs 1-4 for channels 1-4 and again
; ADCs 1-4 for channels 5-8.

int_xmit_adc:
ld A,(xmit_ch)
dec A
and A,bit2_mask
srl A
srl A
push A
pop B
ld A,(mmu_acfg)
and A,bit0_clr
or A,B
ld (mmu_acfg),A
ld A,(xmit_ch)
dec A
and A,0x03
or A,0x34
ld (mmu_adc),A
ld A,ads_rdly
dly A
ld A,(mmu_spidh)
ld (mmu_xhb),A
ld A,(mmu_spidl)
ld (mmu_xlb),A
jp int_xmit_rdy

; Read a byte from the non-volatile memory.

int_xmit_nvm:
ld A,nvm_addr
push A
pop H
ld A,0x00
push A
pop L
call i2c_rd16
push C
pop A
ld (mmu_xhb),A
push B
pop A
ld (mmu_xlb),A
jp int_xmit_rdy

; Transmit an accelerometer register.

int_xmit_acc:
ld A,bma_addr
push A
pop H
ld A,bma_time0
push A
pop L
call i2c_rd16
push C
pop A
add A,off_16bs   
ld (mmu_xhb),A
push B
pop A
ld (mmu_xlb),A
jp int_xmit_rdy

; Transmit synchronizing signal.

int_xmit_sync:
ld A,0              ; Load A with zero
ld (mmu_xlb),A      ; write to transmit LO register.
ld A,(Srun)         ; Load A with Srun
add A,0             ; check value
jp nz,int_xmit_stim ; jump if set.

ld A,synch_nostim   ; Load A with synch_nostim and
ld (mmu_xhb),A      ; write to transmit HI register.
jp int_xmit_rdy   

int_xmit_stim:
ld A,(Spulse)        ; Load A with Spulse
add A,0              ; check value, jump if set.
jp nz,int_xmit_pulse

ld A,synch_stim     ; Load A with synch_stim and
ld (mmu_xhb),A      ; write to transmit HI register.
jp int_xmit_rdy   

int_xmit_pulse:
ld A,(Scurrent)     ; Load A with Scurrent and
sla A               ; shift left
sla A               ; three times to
sla A               ; multiply by eight
add A,synch_stim    ; then add synch_stim.
ld (mmu_xhb),A      ; Write to transmit HI register.

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
; following. The two messages allow us to receive the annoucement
; and identify the device, as well as eliminate noise announcements.
; We pass the auxiliary type in register A and the auxiliary data 
; in register B. The routine assumes we are running in boost with 
; the interrupts disabled.

xmit_annc:

push F
push A

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

ld A,identifier_lo  ; Load LO byte of identifier into A,
or A,0x0F           ; set lower four bits to one
ld (mmu_xch),A      ; and write the transmit channel register.
push B              ; Transfer the data byte from
pop A               ; B into A
ld (mmu_xlb),A      ; and write to transmit LO register.
pop B               ; Pop auxiliary type into B.
ld A,identifier_lo  ; Load LO byte of identifier again.
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

ld A,identifier_hi  ; Load HI byte id identifier into A and
ld (mmu_xlb),A      ; write to transmit LO register.
ld A,identifier_lo  ; Load LO byte of identifier into A,
or A,0x0F           ; set lower four bits to one and
ld (mmu_xch),A      ; write to the transmit channel register.
ld A,identifier_lo  ; Load LO byte into A again,
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

pop F

ret


; ------------------------------------------------------------
; Transmit an acknowledgement. We put the auxiliary type in
; A and the acknowledgement key in B, then call our auxiliary
; message routine.

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

ld A,identifier_hi
push A
pop H
ld A,identifier_lo
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

ld A,identifier_hi  ; Load HI byte id identifier 
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
; one. We assume interrupts are disabled and the CPU is running 
; on the boost clock. 

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

ld A,identifier_hi
push A
pop H
ld A,identifier_lo
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

; Reset the shutdown counter.

ld A,shdn_rst      ; Load the extinguish counter with
ld (shdncnt1),A    ; the maximum sixteen-bit
ld (shdncnt0),A    ; integer.

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

; The stimulus stop instruction.

check_stop_stim:
ld A,(ccmdb)
sub A,op_stop
jp nz,check_stop_stim_end
ld A,0                ; Clear
ld (Srun),A           ; run flag
ld (mmu_stc),A        ; stop stimulus
ld (mmu_i1ph),A       ; and disable the
ld (mmu_i1pl),A       ; timer one interrupt.
ld A,(mmu_imsk)       ; Mask
and A,bit0_clr        ; interrupt one
ld (mmu_imsk),A       ; to stop stimulus.
call annc_ack         ; Acknowledge the stop.
jp cmd_loop
check_stop_stim_end:

; The stimulus start instruction.

check_start:
ld A,(ccmdb)
sub A,op_start
jp nz,check_start_end
call get_cmd_byte    ; Read stimulus current.
ld (Scurrent),A      ; and store in memory.
call get_cmd_byte    ; Read pulse length byte one
ld (Spulse1),A       ; and store to memory.
call get_cmd_byte    ; Read pulse length byte zero
ld (Spulse0),A       ; and store in memory.
call get_cmd_byte    ; Read interval length byte one,
ld (Sint1),A         ; store in memory.
call get_cmd_byte    ; Read interval length byte zero,
ld (Sint0),A         ; store in memory,
call get_cmd_byte    ; Read stimulus length byte one,
ld (Slen1),A         ; and write to memory.
call get_cmd_byte    ; Read stimulus length byte zero,
ld (Slen0),A         ; and write to memory.
call get_cmd_byte    ; Read randomization byte
ld (Srand),A         ; and write to memory.
ld A,0x01            ; Set the
ld (Srun),A          ; stimulus run flag.
ld A,0               ; Clear the 
ld (Spulse),A        ; pulse and
ld (Sdelay),A        ; delay flags.  
ld (mmu_i1ph),A      ; Set the Timer One period to
ld A,1               ; one, so that our first interval
ld (mmu_i1pl),A      ; will begin within a millisecond.
ld A,bit0_mask       ; Reset Timer One interrupt to load
ld (mmu_irst),A      ; our value of one into its counter.
ld A,(mmu_imsk)      ; Load the interrupt mask and
or A,bit0_mask       ; set bit zero to enable
ld (mmu_imsk),A      ; interrupt timer one.
call annc_ack        ; Acknowledge the start.
jp cmd_loop
check_start_end:

; Start data transmission.

check_xon:
ld A,(ccmdb)
sub A,op_xon
jp nz,check_xon_end
call get_cmd_byte    ; Read the telemetry channel number
ld (xmit_ch),A       ; and save in memory.
call get_cmd_byte    ; Read xmit period minus one. 
sub A,min_int_p      ; Subtract the minimum period.
jp c,check_xon_end   ; If result negative, we ignore.
ld A,(ccmdb)         ; Load the period again,
ld (xmit_p),A        ; save to memory and write to
ld (mmu_i4p),A       ; interrupt timer four period.
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
ld A,0               ; Set the xmit period to zero
ld (xmit_p),A        ; in memory, which acts as a flag.
ld (mmu_i4p),A       ; Disable timer interrupt.
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

; Shut down the device. We do this by clearing the transmit, user program
; and stimulus run flags. We acknowledge the shutdown command. The main loop 
; will then shut down the device for us.

check_shdn:
ld A,(ccmdb)
sub A,op_shdn
jp nz,check_shdn_end 
ld A,0x00            ; Clear the 
ld (xmit_p),A        ; transmit enable,
ld (UPrun),A         ; user program run,
ld (Srun),A          ; and stimulus run flags
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

; Initialize registers.
ld A,0
push A
pop B
push A
pop C
push A
pop D
push A
pop E
push A
pop H
push A
pop L

; Initialize variable locations to zero. This activity also serves
; as a boot-up delay to let the power supply settle before we
; calibrate the transmit clock. We are clearing all flags.

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

ld A,identifier_lo ; Set the primary channel number to the
ld (xmit_ch),A     ; LO byte of the device identifier.
ld (rand_0),A      ; Seed the random number generator
ld A,identifier_hi ; with the LO and HI bytes of the
ld (rand_1),A      ; device identifier.
ld A,shdn_rst      ; Load the extinguish counter with
ld (shdncnt1),A    ; the reset value,
ld (shdncnt0),A    ; ready to decrement.

; Configure control space registers.

ld A,0             ; Make sure the stimulus
ld (mmu_stc),A     ; current is zero.
ld (mmu_acfg),A    ; Unassert MSR and select AC coupling.
ld (mmu_dfr),A     ; Set the diagnostic flags to zero.
ld (mmu_i1ph),A    ; Set all the 
ld (mmu_i1pl),A    ; interrupt timer
ld (mmu_i2ph),A    ; periods to zero,
ld (mmu_i2pl),A    ; which disables
ld (mmu_i3p),A     ; their interrupt
ld (mmu_i4p),A     ; generation.
ld (mmu_imsk),A    ; Mask all interrupts.
ld A,0xFF          ; Load A with ones
ld (mmu_irst),A    ; and reset all interrupts.
ld A,frequency_low ; Write the radio frequency
ld (mmu_rfc),A     ; calibration to the firmware.

; Configure user programming.

ld IY,prog_bot     ; The main loop uses IY for the user program pointer.
ld A,ret_code      ; Put a return opcode at first byte
;ld (IY),A          ; in user program, in case of enable.

; Calibrate the transmit clock and initialize the sensors.
call calibrate_tck
call bma_config
call tmp_single
call ads_calib

; Enable interrupts.

clri

; The main event loop.

main_loop:

; Deal with any pending commands.

ld A,(mmu_sr)       ; Fetch status register.
and A,sr_cmdrdy     ; Check the command ready bit.
jp z,main_nocmd     ; Jump if it's clear,
call cmd_execute    ; execute command if it's set.
main_nocmd:

; If the stimulus flag is set, reset the shutdown counter and call main loop
; again. 

main_check_srun:
ld A,(Srun)
add A,0
jp z,main_shdn_dec
ld A,shdn_rst
ld (shdncnt0),A
ld (shdncnt1),A
jp main_loop

; Decrement the shutdown counter. When negative, we will switch off. The shutdown
; counter will be decremented by the one in every main loop. The cmd_execute 
; routine resets the counter whenever it receives a command directed at this device.

main_shdn_dec:
ld A,(shdncnt0)
sub A,1
ld (shdncnt0),A
ld A,(shdncnt1)
sbc A,0
ld (shdncnt1),A
jp nc,main_check_flags

; We are going to shut down. We announce the shutdown once, when the shutdown counter
; reaches zero. We will keep running the main loop, but because the shutdown counter 
; will start counting down from 0xFFF, it will not reach zero again before the device
; powers off.

main_shdn_ack:
seti
ld A,0x03        
ld (mmu_ccr),A 
ld A,op_shdn
ld (Sack_key),A
call annc_ack
ld A,0x00
ld (mmu_ccr),A
clri
jp main_shdn

; Check to see if we are transmitting our synchronizing signal or perhaps running
; a user program. If so, we continue running.

main_check_flags:
ld A,(xmit_p)
add A,0
jp nz,main_loop
ld A,(UPrun)
add A,0
jp nz,main_loop

; We get here when the device is first powering up, in which case our effort to 
; turn off the device will not take effect because the device is being kept awake
; by a command incoming flag. We keep executing the main loop. We also get here
; when it is time to shut down after commands have been executed or the shutdown 
; counter has run down. We turn off device power, but continue executing the main
; loop until the logic turns off.

main_shdn:
jp main_loop

; ---------------------------------------------------------------


