; ------------------------------------------------------------
; I2C Eight-Bit Write. Write to one eight-bit register 
; location on the sensor. We pass the I2C device address in 
; Register H, the sub-address in Register L, and the eight
; bits to write in Register C.

i2c_wr8:
       
; I2C: Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address and !WRITE flag (SAD+W).
; The device address is in Register H.

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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write eight-bit sub-address (SUB), which has
; been passed in Register L.

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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write the first eight data bits, which we have passed
; in Register C.

push C            ; 1
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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
     
ret


; ------------------------------------------------------------
; I2C Sixteen-Bit Write. Write to one sixteen-bit register 
; location on the sensor. We pass the I2C device address in 
; Register H, the sub-address in Register L, first eight bits
; to write in Register C and final eight bits in Register B.

i2c_wr16:
       
; I2C: Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address and !WRITE flag (SAD+W).
; The device address is in Register H.

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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write eight-bit sub-address (SUB), which has
; been passed in Register L.

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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write the first eight data bits, which we have passed
; in Register C.

push C            ; 1
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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write the next eight data bits, which we have passed
; in Register B.

push B            ; 1
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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
     
ret


; ------------------------------------------------------------
; I2C Sixteen-Bit Read. Read two consecutive bytes from the sensor
; address map. We pass the I2C device address in Register H and the
; sub-address in Register L. The first byte read will be returned
; in C, the second in B.

i2c_rd16:

; I2C: Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address and !WRITE flag (SAD+W). The
; device address is in Register H.

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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write eight-bit sub-address (SUB), which is
; stored in Register L.

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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Repeat start code (RS)

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address again, this time with
; a READ flag (SAD+R).

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

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Read eight data bits from slave (DATA).

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

; Transfer the first data byte to C.

ld A,(mmu_i2cMR)  ; 4
push A            ; 1
pop C             ; 2

; I2C: Transmit master acknowledgement (MAK).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Read eight data bits from slave (DATA).

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

; Transfer the second data byte to B.

ld A,(mmu_i2cMR)  ; 4
push A            ; 1
pop B             ; 2

; I2C: Transmit not master acknowledgement (NMAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
     
ret
