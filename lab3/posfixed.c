#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define Q 13

#define k1     26944
#define k2     14608 
#define kr     14609
#define l1     16678
#define l2     10184
#define lv     10184 

#define phi11  8142
#define phi12  0
#define phi21  2041 
#define phi22  8192
#define gamma1 919
#define gamma2 115

int16_t v = 0;
int16_t x1 = 0;
int16_t x2 = 0;
int16_t u = 0;
int16_t eps_13 = 0;

 /* Controller parameters and variables (add_13 your own code here) */
 
 int8_t on = 0;                     /* 0=off, 1=on */
 int16_t r = 255;                   /* Reference, corresponds to +5.0 V */
 
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
 static inline int16_t add_13(int32_t x, int32_t y){
    int32_t result = x + y;  
    if (result > INT16_MAX) {
        return INT16_MAX;  
    } else if (result < INT16_MIN) {
        return INT16_MIN;  
    }
    return (int16_t)result;  
}
static inline int16_t sub_13(int32_t x, int32_t y){
    int32_t result =  x - y;  
    if (result > INT16_MAX) {
        return INT16_MAX;  
    } else if (result < INT16_MIN) {
        return INT16_MIN;  
    }
    return (int16_t)result;  
    
}
static inline int16_t mul_13(int32_t x, int32_t y){
    int32_t result = x *y;  
    result = (result + (1 << (Q - 1))) >> Q;
    if (result > INT16_MAX) {
        return INT16_MAX;  
    } else if (result < INT16_MIN) {
        return INT16_MIN;  
    }
    return (int16_t)result;  
    
}

static inline int16_t div_13(int32_t x, int32_t y){
  if (y == 0) return 0;
  int32_t result = x << Q; 
  result = result / y;  
  if (result > INT16_MAX) {
      return INT16_MAX;  
  } else if (result < INT16_MIN) {
      return INT16_MIN;  
  }
  return (int16_t)result; 
}
 
 /**
  * Interrupt handler for the periodic timer. Interrupts are generated
  * every 10 ms. The control algorithm is executed every 50 ms.
  */
 ISR(TIMER2_COMP_vect){
   static int8_t ctr = 0;
   if (++ctr < 5) return;
   ctr = 0;
   int16_t Y = readInput('1');
   int32_t scaled_r = r << Q;   // Scale r to Q3.13 format
   int32_t scaled_Y = Y << Q;
   if (on) {
     /* Insert your controller code here */

     int32_t u_temp = sub_13(
            sub_13(
                sub_13(
                    mul_13(kr, r),
                    mul_13(k1, x1)
                ),
                mul_13(k2, x2)
            ),
            v
        );    
    u = u_temp >> Q;  
    if(u > 511) u = 511;
    
    else if(u< -512) u = -512;
     writeOutput(u);
     int16_t x1_old = x1;
     eps_13 = sub_13(scaled_Y, x2);
     x1 = add_13(
        add_13(
            add_13(
                mul_13(phi11, x1),
                mul_13(phi12, x2)),
            mul_13(gamma1, add_13(u, v))),
        mul_13(l1, eps_13)
    );
     x2 = add_13(
            add_13(
                add_13(
                    mul_13(phi21, x1_old),
                    mul_13(phi22, x2)),
                mul_13(gamma2, add_13(u, v))),
            mul_13(l2, eps_13)
        );
     v = add_13(v, mul_13(eps_13, lv));
     
 
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
 