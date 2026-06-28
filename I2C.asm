; ------------------------------------------------------------
; I2C Write. Write N bytes to an I2C device. The routine assumes 
; the device auto-increments its sub-address after each read. We
; will be writing to consecutive bytes in its internal address 
; space. We pass the device selection address in H, the internal 
; sub-address in L, the number of bytes to be written in C, and 
; a pointer to the data bytes in IX. Register IX will be returned
; pointing to the location after the final byte written. All other
; registers will be returned unchanged.
;
; This routine will run equally well in slow or boost mode. The
; I2C interface is not re-entrant, so any interruption of this
; routine must refrain from calling this or any other I2C routine.

i2c_wr:

; Push F, A, and C so we don't trash them.

push F
push A
push C

; Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; Write seven-bit device address and set the WRITE flag (SAD+W). 
; The device address is in H. We rotate the address left and fill 
; the least significant bit with a zero to indicate that we are 
; going to write a byte after we send the device address. That 
; byte is going to be the sub-address.

push H            ; 1
pop A             ; 2
sla A             ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; Accept slave acknowledgement (SAK). But note that we do not 
; bother to check the slave acknowledgement. We just assume it
; occurs.

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Write the eight-bit sub-address (SUB), which has been passed in 
; Register L, to the slave, which should now be listening after
; being selected by the device address.

push L            ; 1
pop A             ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Write consecutive bytes to the slave. The byte to be written is pointed
; to by IX and the number of bytes remaining to be written is in Register C.
; We assume that C > 0. If not, we will write 256 bytes.

i2c_wr_loop:

ld A,(IX)         ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Increment IX and decrement C. If C is not zero, loop
; back and write another bytes.
inc IX
dec C
jp nz,i2c_wr_loop

; I2C: Stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
     
; Restore A, C, and F.

pop C
pop A
pop F

ret

; ------------------------------------------------------------
; I2C Read. Read one or more bytes from an I2C device. The routine
; assumes the device auto-increments its sub-address after each
; read, so that we will be reading consecutive bytes from its
; internal address space. We pass the device selection address 
; in H, the internal sub-address in L, the number of bytes to be
; read in C, and a pointer to the destination of the bytes in IX. 
; Registers C, H and L are unchanged, but IX will be incremented to
; the location just after the last written byte. All other registers
; remain intact.
;
; This routine will run equally well in slow or boost mode. The
; I2C interface is not re-entrant, so any interruption of this
; routine must refrain from calling this or any other I2C routine.

i2c_rd:

; Push F, A, and C so we don't trash them. They are not used to pass
; arguments into the routine.

push F
push A
push C

; Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; Write seven-bit device address and set the WRITE flag (SAD+W). 
; The device address is in H. We rotate the address left and fill 
; the least significant bit with a zero to indicate that we are 
; going to write a byte after we send the device address. That 
; byte is going to be the sub-address.

push H            ; 1
pop A             ; 2
sla A             ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Write eight-bit sub-address (SUB), which is stored in L.

push L            ; 1
pop A             ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Repeat start code (RS). This code tells the bus that we are
; keeping control and continuing.

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; We write seven-bit device address again, but this time with
; the READ flag (SAD+R) set. We shift the device address left
; and set the least significant bit to one for !WRITE so that
; we have READ.

push H            ; 1
pop A             ; 2
sla A             ; 1
or A,0x01         ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Read consecutive bytes from the slave. The destination of the byte in 
; our process memory is given by IX and the number of bytes remaining to 
; be read is in Register C. We assume that C > 0. If not, we will read 
; 256 bytes. The individual bit reads are done with writes of any value
; to i2cZ0, i2cZ1, and i2cZ0. On a write to i2cZ1 the I2C data line is 
; shifted into the i2C data byte in the firmware. We will this byte
; after eight bits have been read.

i2c_rd_loop:

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Transfer the data byte to A and write to process memory at 
; location IX.

ld A,(mmu_i2cMR)  ; 4
ld (IX),A

; Increment IX and decrement C. If C is zero, jump to NMAK. 

inc IX
dec C
jp z,i2c_rd_nmak

; We still have bytes to read, so ransmit master acknowledgement 
; (MAK) and jump to the beginning of our read loop.

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ0),A  ; 3
jp i2c_rd_loop

; Transmit not master acknowledgement (NMAK).

i2c_rd_nmak:

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Now we are done, so release the bus with a stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
  
; Pop A, C and F.

pop C
pop A
pop F
 
ret
