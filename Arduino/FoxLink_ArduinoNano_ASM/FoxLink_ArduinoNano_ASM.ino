void setup() {
    DDRD = 0b00000010;
}

void loop() {
    asm volatile (
        "in r24, %[pind]  \n"
        "lsl r24          \n"
        "andi r24, 0x02   \n"
        "out %[portd], r24 \n"
        :
        : [pind] "I" (_SFR_IO_ADDR(PIND)), [portd] "I" (_SFR_IO_ADDR(PORTD))
        : "r24"
    );
}