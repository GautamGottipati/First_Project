/*  
 * Team Id: EYRC<3921> 
 * Author List: G.Gautam,G.RajiniKanth,G.Tarun,K.Ritesh
 * Filename: Collector bot ATMEGA2560 
 * Theme: Collector bot implementation
 * Functions: servo1_pin_config,motion_pin_config ,port_init(),timer1_init,velocity
 * Global Variables: NUMBER_OF_FIELDS,fieldIndex,sign
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include<UART.h>
#include<math.h>
const int Number_Of_Fields=3;
int fieldIndex=0;
char values[Number_Of_Fields];
int sign=1;

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
  DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
  PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
  DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
  PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}


//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
 DDRK = DDRK | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTK = PORTK & 0xF0; //set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRE = 0x18;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTE = 0x18; //PD4 and PD5 pins are for velocity control using PWM
}

//Function to Initialize PORTs
void port_init()
{
 servo1_pin_config ();
 servo2_pin_config();
 motion_pin_config();
}

//Initialize the ports
//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
  TCCR1B = 0x00; //stop
  TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
  TCNT1L = 0x01;  //Counter low value to which OCR1xH value is to be compared with
  OCR1AH = 0x03;  //Output compare Register high value for servo 1
  OCR1AL = 0xFF;  //Output Compare Register low Value For servo 1
  OCR1BH = 0x03;  //Output compare Register high value for servo 2
  OCR1BL = 0xFF;  //Output Compare Register low Value For servo 2
  
  ICR1H  = 0x03;  
  ICR1L  = 0xFF;
  TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
          For Overriding normal port functionality to OCRnA outputs.
        {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
  TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Timer3 is configured for constant frequency and variable duty cyclr
//TIMER3 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer3_init(void)
{
 TCCR3B = 0x00; //stop
 TCNT3H = 0xFF; //higher byte constant frequency value of PWM cycle 
 TCNT3L = 0x01; //lower byte constant frequency value of PWM cycle 
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR3H  = 0x00;
 ICR3L  = 0xFF;
 TCCR3A = 0xA9;//0xA1 
 TCCR3B = 0x0D; //start Timer
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortKRestore = 0;

 Direction &= 0x0F;       // removing upper nibbel as it is not needed
 PortKRestore = PORTK;      // reading the PORTB's original status
 PortKRestore &= 0xF0;      // setting lower direction nibbel to 0
 PortKRestore |= Direction;   // adding lower nibbel for direction command and restoring the PORTB status
 PORTK = PortKRestore;      // setting the command to the port
}

void forward (void) //both wheels forward
{
  motion_set(0x0A);
}

void backward (void) //both wheels forward
{
  motion_set(0x05);
}

void softright(void)
{
  motion_set(0x06);
}

void left(void)
{
  motion_set(0x02);
}

void softleft(void)
{
  motion_set(0x09);
}
void right(void)
{
  motion_set(0x08);
}

void hard_stop (void)    //hard stop (Stop suddenly)
{
  motion_set(0x00);
}


//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR3AH = 0x00;
 OCR3AL = left_motor;     // duty cycle 'ON' period of PWM out for Left motor 
 OCR3BH = 0x00;
 OCR3BL = right_motor;    // duty cycle 'ON' period of PWM out for Right motor 
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 uart0_init();
 port_init();
 timer1_init();
 timer3_init();
 sei(); //Enables the global interrupts
}


void servo_1(unsigned char degrees)  
{
  float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
  OCR1AH = 0x00;
  OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
  float PositionTiltServo = 0;
  PositionTiltServo = ((float)degrees / 1.86) + 35.0;
  OCR1BH = 0x00;
  OCR1BL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
  OCR1AH = 0x03; 
  OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
  OCR1BH = 0x03;
  OCR1BL = 0xFF; //Servo 2 off
}

void object_grip (void) // grip the object
{
 servo_2 (35);
 _delay_ms(1000);
}

void object_ungrip (void) // relase the object
{
 servo_2 (90);
 _delay_ms(1000);

}

void arm_lift (void)
{
 servo_1 (40);
 _delay_ms(1000);
}

void arm_down (void)
{
 servo_1 (90);
 _delay_ms(1000);
}


//Main Function
int main(void)
{
 unsigned char ch;
 
 init_devices();
 arm_down();
object_ungrip(); 
// char phi;

 //velocity (v_des, v_des); //Set robot velocity here. Smaller the value lesser will be the velocity
          //Try different valuse between 0 to 255
 forward(); 
 while(1)
 {
  ch=uart_rx();//Takes the ASCII values that is sent by the transmitter 
  if(ch=='u')
  {
    arm_lift();
  }
  else if(ch=='d')
  {
    arm_down();
  }
  else if(ch=='g')
  {
    object_grip();
  }
  else if(ch=='l')
  {
    object_ungrip();
  }
  else if(ch>=48 && ch<=57)
  {
    values[fieldIndex]=(values[fieldIndex]*10)+(ch-'0');
  }
  else if(ch==44)
  {
    values[fieldIndex]=values[fieldIndex]*sign;
    fieldIndex = fieldIndex+1;
    sign=1;
  }
  else if(ch=='-')
  {
    sign=-1;
  }
  else
  {
    values[fieldIndex]=values[fieldIndex]*sign;
    w_l=values[0];
    w_r=values[1];
    int l=w_l;
    int r=w_r;
    w_l=abs(w_l);
    w_r=abs(w_r);
    velocity(w_l,w_r);
    
    if(l>=0 && l>=0)
    {
      forward();
    }
    if(l<0 && r>0)
    {
      softleft();
      
    }
    if(l>0 && r<0)
    {
      softright();
    }
   
    if(l<=0 && r<=0)
    {
      backward();
    }
    
    
    for(int i=0;i<min(Number_Of_Fields,fieldIndex+1);i++)
    {
      values[i]=0;
    }
    fieldIndex=0;
    sign=1;
  }
  
  
  
  
  
  
  
  
 }
  
  
 

}//finalcode
/*  
 *   if (w_l>=0 && w_l>=0)
    {
      velocity(w_l,w_r);
      forward();
    }
     if (w_l<=-100 && w_r>=0)
    {
      velocity(abs(w_l),abs(w_r));
      softleft();
    }
    if (w_l>=0 && w_r<=0)
    {
      velocity(abs(w_l),abs(w_r));
      softright();
    }
    else
    {
      velocity(abs(w_l),abs(w_r));
      backward();
    }
  Use this code segment to graduali increace robot velocity in sawtooth pattern, comment the bottom code segment
  
  forward(); //both wheels forward
  for(i = 0;i<255;i++)
    {
  velocity (i, i); //Set robot velocity here. More value will result in more velocity
  
  _delay_ms(100);
    }
 */      
 /*phi= uart_rx();
 phi=float(phi);
 */
/* w_des=-0.5*phi;
 v_r=v_des+(wheel_seperation/2)*w_des;
 v_l=v_des-(wheel_seperation/2)*w_des;
 w_r=v_r/wheel_radius;
 w_l=v_l/wheel_radius;
*/
//________________
// _delay_ms(2000);
  
 //velocity(0,0); 
/*
  hard_stop();            
  _delay_ms(500);

  back(); //both wheels backward            
  _delay_ms(1000);

  hard_stop();            
  _delay_ms(500);
  
  left(); //Left wheel backward, Right wheel forward
  _delay_ms(500);
  
  hard_stop();            
  _delay_ms(500);
  
  right(); //Left wheel forward, Right wheel backward
  _delay_ms(500);

  hard_stop();            
  _delay_ms(500);

  soft_left(); //Left wheel stationary, Right wheel forward
  _delay_ms(500);
  
  hard_stop();            
  _delay_ms(500);

  soft_right(); //Left wheel forward, Right wheel is stationary
  _delay_ms(500);

  hard_stop();            
  _delay_ms(500);

  soft_left_2(); //Left wheel backward, right wheel stationary
  _delay_ms(500);

  hard_stop();            
  _delay_ms(500);

  soft_right_2(); //Left wheel stationary, Right wheel backward
  _delay_ms(500);
  while(1)
 {
   w_l=w_l*10;
   w_l=uart_rx();
   if(w_l==0)
      break;
   w_l=(w_l-'0');
   _delay_ms(500);

*/
  //velocity(0,0);            
  //_delay_ms(500);

