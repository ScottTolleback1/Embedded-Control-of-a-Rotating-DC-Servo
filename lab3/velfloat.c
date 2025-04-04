/** 
 * AVR program for control of the DC-servo process, 2024 edition.
 *  
 * User communication via the serial line. Commands:
 *   s: start controller
 *   t: stop controller
 *   r: change sign of reference (+/- 5.0 volt)
 * 
 * To compile for the ATmega8 AVR:
 *   avr-gcc -mmcu=atmega8 -O -g -Wall -o DCservo.elf DCservo.c   
 * 
 * To upload to the ATmega8 AVR:
 *   avr-objcopy -Osrec DCservo.elf DCservo.sr
 *   avrdude -e -p atmega8 -P /dev/ttyACM0 -c avrisp2 -U flash:w:DCservo.sr:a
 * 
 * To compile for the ATmega16 AVR:
 *   avr-gcc -mmcu=atmega16 -O -g -Wall -o DCservo.elf DCservo.c   
 * 
 * To upload to the ATmega16 AVR:
 *   avr-objcopy -Osrec DCservo.elf DCservo.sr
 *   avrdude -e -p atmega16 -P usb -c avrisp2 -U flash:w:DCservo.sr:a
 * 
 * To view the assembler code:
 *   avr-objdump -S DCservo.elf
 * 
 * To open a serial terminal on the PC:
 *   simcom -38400 /dev/ttyS0 
 */

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <inttypes.h>
 #define K 2.6133
 #define Ti 0.4523
 #define B 0.5
 #define h 0.05
 #define KB 1.30665
 #define k1 3.2898
 #define k2 1.7831
 #define kr 1.7832
 #define l1 2.0361
 #define l2 1.2440
 #define l3 3.2096
 
 #define gamma1 0.1122
 #define gamma2 0.0140
 /* Controller parameters and variables (add your own code here) */
 
 int8_t on = 0;                     /* 0=off, 1=on */
 int16_t r = 255;                   /* Reference, corresponds to +5.0 V */
 float u = 0;
 float I = 0;
 /** 
  * Write a character on the serial connection
  */
 static inline void put_char(char ch){
   while ((UCSRA & 0x20) == 0) {}; /* Wait until USART Data Register is empty */
   UDR = ch; /* Write to USART Data Register, sends data via serial cable */
 }
 
 /**
  * Write 10-bit output using the PWM generator
  */
 static inline void writeOutput(int16_t val) {
   val += 512;
   OCR1AH = (uint8_t) (val>>8);
   OCR1AL = (uint8_t) val;
 }
 
 /**
  * Read 10-bit input using the AD converter from channel (0 or 1)
  */
 static inline int16_t readInput(char chan) {
   uint8_t low, high;
   ADMUX = 0xc0 + chan;             /* Specify reference voltage and channel */
   ADCSRA |= 0x40;                  /* Start the conversion */
   while (ADCSRA & 0x40);           /* Wait for conversion to finish */
   low = ADCL;                      /* Read input, low byte first! */
   high = ADCH;                     /* Read input, high byte */
   return ((high<<8) | low) - 512;  /* 10 bit ADC value [-512..511] */ 
 }  
 
 /**
  * Interrupt handler for receiving characters over serial connection
  * Interrupt occurs when data has been received
  */
 ISR(USART_RXC_vect){ 
   switch (UDR) {                   /* USART I/O Data Register */
   case 's':                        /* Start the controller */
     put_char('s');
     on = 1;
     break;
   case 't':                        /* Stop the controller */
     put_char('t');
     on = 0;
     break;
   case 'r':                        /* Change sign of reference */
     put_char('r');
     r = -r;
     break;
   }
 }
 
 /**
  * Interrupt handler for the periodic timer. Interrupts are generated
  * every 10 ms. The control algorithm is executed every 50 ms.
  */
 ISR(TIMER2_COMP_vect){
   static int8_t ctr = 0;
   if (++ctr < 5) return;
   ctr = 0;
   
   float Y = readInput('0');
   if (on) {
     /* Insert your controller code here */
     u = K * B * r - K*Y + I;
     if(u > 511) u = 511;
       
     else if(u< -512) u = -512;
    
     writeOutput(u);
     I = I  + K * h/Ti *(r - Y);
 
   } else {                     
     writeOutput(0);     /* Off */
   }
 }
 
 /**
  * Main program
  */
 int main(){
 
   /* Set port data directions and configure ADC */
   DDRB = 0x02;    /* Enable PWM output for ATmega8 */
   DDRD = 0x20;    /* Enable PWM output for ATmega16 */
   DDRC = 0x30;    /* Enable time measurement pins */
   ADCSRA = 0xc7;  /* ADC enable + start + prescaling */
 
   /* Timer/Counter configuration */
   TCCR1A = 0xf3;  /* Timer 1: OC1A & OC1B 10 bit fast PWM */
   TCCR1B = 0x09;  /* Clock / 1 (i.e. no prescaling) */
 
   TCNT2 = 0x00;   /* Timer 2: Reset counter (periodic timer) */
   TCCR2 = 0x0f;   /* Clock / 1024, clear after compare match (CTC) */
   OCR2 = 144;     /* Set the output compare register, corresponds to ~100 Hz */
 
   /* Configure serial communication */
   /* Set USART Control and Status Registers */
   UCSRA = 0x00;   /* USART: */
   UCSRB = 0x98;   /* USART: RXC enable, Receiver enable, Transmitter enable */
   UCSRC = 0x86;   /* USART: 8bit, no parity, asynchronous */
   /* 12bit USART baud rate register (high and low byte) */
   UBRRH = 0x00;   /* USART: 38400 @ 14.7456MHz */
   UBRRL = 23;     /* USART: 38400 @ 14.7456MHz */
 
   TIMSK = 1<<OCIE2; /* Start periodic timer */
 
   sei();          /* Enable interrupts */
 
   while (1);
 }
 