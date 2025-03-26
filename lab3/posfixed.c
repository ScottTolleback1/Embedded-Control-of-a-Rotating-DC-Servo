#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define Q_SCALE (1 << 13)  

#define K      ((int16_t)(2.6133  * Q_SCALE))  
#define Ti     ((int16_t)(0.4523  * Q_SCALE))  
#define B      ((int16_t)(0.5     * Q_SCALE)) 
#define h      ((int16_t)(0.05    * Q_SCALE))  
#define KB     ((int16_t)(1.30665 * Q_SCALE))  

#define k1     ((int16_t)(3.2898  * Q_SCALE))  
#define k2     ((int16_t)(1.7831  * Q_SCALE))  
#define kr     ((int16_t)(1.7832  * Q_SCALE))  
#define l1     ((int16_t)(2.0361  * Q_SCALE))  
#define l2     ((int16_t)(1.2440  * Q_SCALE)) 
#define lv     ((int16_t)(3.2096  * Q_SCALE))  

#define phi11  ((int16_t)(0.994   * Q_SCALE))  
#define phi12  ((int16_t)(0       * Q_SCALE))  
#define phi21  ((int16_t)(0.2493  * Q_SCALE)) 
#define phi22  ((int16_t)(1       * Q_SCALE))  
#define gamma1 ((int16_t)(0.1122 * Q_SCALE)) 
#define gamma2 ((int16_t)(0.0140 * Q_SCALE)) 

int16_t v = 0;
int16_t x1 = 0;
int16_t x2 = 0;
int16_t u = 0;
int16_t eps = 0;

 /* Controller parameters and variables (add your own code here) */
 
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
 static inline int16_t add(int16_t x, int16_t y){
    int32_t result = (int32_t)x + y;  
    if (result > INT16_MAX) {
        return INT16_MAX;  
    } else if (result < INT16_MIN) {
        return INT16_MIN;  
    }
    return (int16_t)result;  
}
static inline int16_t sub(int16_t x, int16_t y){
    int32_t result = (int32_t)x - y;  
    if (result > INT16_MAX) {
        return INT16_MAX;  
    } else if (result < INT16_MIN) {
        return INT16_MIN;  
    }
    return (int16_t)result;  
    
}
static inline int16_t mul(int16_t x, int16_t y){
    int32_t result = (int32_t)x *y;  
    result = result >> Q;
    if (result > INT16_MAX) {
        return INT16_MAX;  
    } else if (result < INT16_MIN) {
        return INT16_MIN;  
    }
    return (int16_t)result;  
    
}
static inline int16_t div(int16_t x, int16_t y){
    if(y == 0) return 0;
    int32_t result = (int32_t)x << Q; 
    result = result / (int32_t)y; 
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
   int32_t scaled_r = r << 13;   // Scale r to Q3.13 format
   int32_t scaled_Y = Y << 13;
   if (on) {
     /* Insert your controller code here */

     int32_t u_temp = sub(
            sub(
                sub(
                    mul(kr, scaled_r),
                    mul(k1, x1)
                ),
                mul(k2, x2)
            ),
            v
        );    
    u = u_temp >> 13;  
    if(u > 511) u = 511;
    
    else if(u< -512) u = -512
     writeOutput(u);
     eps = sub(scaled_Y, x2);
     x1 = add(
        add(
            add(
                mul(phi11, x1),
                mul(phi12, x2)),
            mul(gamma1, add(u, v))),
        mul(l1, eps)
    );
     x2 = add(
            add(
                add(
                    mul(phi21, x1),
                    mul(phi22, x2)),
                mul(gamma2, add(u, v))),
            mul(l2, eps)
        );
     v = add(v, lv);
     
 
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
 