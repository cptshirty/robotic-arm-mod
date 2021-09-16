#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

/*
This is a source code for 5 DOF robotic hand with one extra axis for gripper/some accesory,
 that uses a stepper motor, most of this code was written before I had any Idea wham am I doing, so please dont judge the coding style too harshly.

*/


#define MIN_STEP_DURATION 50 // max speed 57,44 deg/s not used rn
#define MICROSTEPS 16 //MICROSTEPS set on the driver
#define GEAR_RATIO_PLANETARY 38.4 //transfer ratio of used planetary gearboxes
#define GEAR_RATIO_STRAIN 38 //transfer ratio of new strain wave gearboxes
#define STEPS_PER_3_DEGREES 1024 // 3 degrees is arbitrarily chosen constant, because its a whole number with the 38,4 gear reduction
#define DEFAULT_SPEED 5 // deg/s
#define NUMBER_OF_JOINTS 6
#define PROGPIN A0
#define GRIPPER_RATIO 108
#define GRIPPER_STEPS_PER_120TH 107
#define CS 10 //chip select pin
#define HELP " \n _______ \n J- |joint A- |angle S- |speed \n M | start move \n G0 -- | mount SD card \n G1 - | change directory \n G2 - | \\
jump one directory up \n G3 - | show files in directory \n G4- | select file \n G5 | execute file \n G6 | delay (ms) \n G7 | end of program \n G8- | \\
 Jump to place in code \n G9 - code mark to jump \n G10 -home position \n H | help \n _________"

volatile uint32_t half_mikrosec = 0;

ISR(TIMER1_OVF_vect) //interrupt service routine of hardware timer in the MCU
{
half_mikrosec += UINT16_MAX; //the timer is 16 bit
}

class joints
{
  public:
    float currentangle = 0;
    unsigned long steps;
    int konst = 0;
    float ratio = 0;
    volatile uint8_t *port = 0; //output port at which the pin is located
    // add soft limits for movement
    // unsigned long maxsteps;
    float angle = 0; //target angle
    uint16_t pause; //pause between steps
    uint32_t lastmicros = 0;
    uint8_t step_pin; //pin for step
    uint8_t dir_pin; //pin for direction change
    //float accel = 70; //acceleration deg/s-2 not used, it was done throught software but the Atmega chip is not powerfull enought
      float targetspeed  = 25;
    void calcPause(float speeds, int konst) //calculate time needed between steps
    {
      if (konst ==107)
        pause = (3000000 / (konst * speeds*10));
      else
        pause = (3000000 / (konst * speeds));
     // pause = pause;
      pause = round(pause);
     // Serial.println(F("pause is "));
     // Serial.println(pause);
    };
    void init(uint8_t stepe, uint8_t dir,volatile uint8_t *porte, float rati = GEAR_RATIO_PLANETARY,int kons = STEPS_PER_3_DEGREES) //initialize 
    {
      step_pin = stepe;
      dir_pin = dir;
      ratio = rati;
      konst = kons;
      port = porte;
      *(port-1) |=step_pin;
      *(port-1) |=dir_pin;
      calcPause(targetspeed,konst);
    };
};

joints joint[NUMBER_OF_JOINTS];
bool start = 0;
bool prog = 1;
bool executing = 0;
uint32_t jump_adress_storage[10] = {0,0,0,0,0,0,0,0,0,0};
SdFat sd;
SdFile root;
SdFile file;
uint8_t movingjoints[] = {255,255,255,255,255,255};
uint8_t num_movingjoints = 0;

//setup all the important registers and start serial communication
void setup() 
{
  joint[0].init(4,8,&PORTD,GRIPPER_RATIO,GRIPPER_STEPS_PER_120TH);
  joint[1].init(16,32,&PORTD);
  joint[2].init(64,128,&PORTD);
  joint[3].init(1,2,&PORTB);
  joint[4].init(2,4,&PORTC);
  joint[5].init(8,16,&PORTC);
  pinMode(PROGPIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,0);
  Serial.begin(9600);
  SD_mount();
  TCCR1A = 0;
  TCCR1B = 0b00000010;
  TIMSK1 |= (1<<TOV1);
}


void stepe(float angle);
void sendstep();
void evaluation(String command);


void loop()
{
  if (!start)
  {
    prog = digitalRead(PROGPIN);
    if(!prog && executing ==0) // execute demo program
    {
      Serial.println(F("Executin of demo started"));
      file.open("demo.txt",O_READ);     
      executing = 1;
    };
    String cmd = "";
    if (Serial.available() > 0)
    {
      cmd = Serial.readStringUntil('\n');
      if (cmd[cmd.length()-1]== '\r')
        cmd = cmd.substring(0,cmd.length()-1);
    }
    else if(executing)
    {
      char k = file.read();
      while (k!= '\n') //weird read from sdcard function, but it works
      {
        if (k!='\r')
          cmd += k;
        k = file.read();
        if (k <0)
          {
          Serial.println(F("end of file reached"));
          executing = 0;
          cmd ="G7";
          break;
          }
      };
    };
    if (cmd.length()>0){
      Serial.println(cmd);
      evaluation(cmd);  
    };
  };

  if (start)
  {
    uint8_t check = 0;
    uint32_t time = half_mikros();
    for(uint8_t i = 0;i<num_movingjoints;i++)
     {
      movejoint(movingjoints[i],time);
      if(joint[movingjoints[i]].angle ==joint[movingjoints[i]].currentangle)
        check++;
    };
    if (check == num_movingjoints)
    {
      start = 0;
    for(uint8_t i =0;i<NUMBER_OF_JOINTS;i++)
      movingjoints[i]=255;
      num_movingjoints = 0;
    };

  };

}

void SD_mount()
{   
    if (!sd.begin(CS, SD_SCK_MHZ(50))) 
      sd.initErrorPrint();
    if (!root.open("/")) 
    {  sd.errorPrint(F("open root failed"));
      Serial.println();
    }
    else
    {
    Serial.println(F("Initialization succesfull!"));
    };
}
void movejoint(uint8_t k,uint32_t time)
{
    if ( (time - joint[k].lastmicros) >= joint[k].pause)
    {
      stepe(k);
      joint[k].lastmicros = time;
    };
}

//prints the content of current directory
void printDirectory() {
  while (file.openNext(sd.vwd(), O_RDONLY)) {
  
  if (file.isHidden())
    {  
      file.close();
      continue;
    };
    file.printFileSize(&Serial);
    Serial.write(' ');
    file.printModifyDateTime(&Serial);
    Serial.write(' ');
    file.printName(&Serial);
    if (file.isDir()) {
      // Indicate a directory.
      Serial.write('/');
    }
    Serial.println();
    file.close();
  };
  if (root.getError()) {
    Serial.println(F("openNext failed"));
  } else {
    Serial.println(F("_________"));
  }
  sd.vwd()->rewind();
}
void sendstep(uint8_t s)
{
  write_one(joint[s].step_pin,joint[s].port);
  delayMicroseconds(1);
  write_zero(joint[s].step_pin,joint[s].port);
}
void stepe(uint8_t s)
{
  if (joint[s].steps > 0)
  {
    sendstep(s);
    joint[s].steps--;
  }
  else
  {
  joint[s].currentangle = joint[s].angle;
  };
}
float toNumber(String a) //my function to convert characters to float, it could probably be done waaay easier
{
  float cislo = 0;
  int point = a.length();
  bool minus = 0;
  for (int i = a.length(); i >= 0; i--)
  {
    if ((a[i] >= 48) && (a[i] <= 57))
      a[i] -= 48;
    else if ((a[i] == 46) || (a[i] == 44))
      point = i;
    else if (a[i] == 45)
      minus = 1;
  };
  for (int i = a.length(); i >= 0; i--)
  {
    if ((a[i] >= 0) && (a[i] <= 9))
    {
      (i > point) ? cislo += (a[i] * pow(10, (point - i))) : cislo += (a[i] * pow(10, point - (i + 1)));
    };
  };
  if (minus)
    cislo = -cislo;
  return cislo;
}
void evaluation(String cmd) // decode the text in command
{
  switch(cmd[0])
    {
      case 'J':
        joint_evaluation(cmd);
        break;
      case 'G':
        g_evaluation(cmd);
          break;
      case 'H':
        Serial.println(F(HELP));
      case 'M':
        start = 1;
      default:
        break;
    }
}
void g_evaluation(String command)
{
  int delka = command.length();
  int temp = 0;
  int G_len = 0;
  for (int i = 0; i < delka; i++)
  {
    if (','<=command[i] && command[i]<='9')
    {
      uint8_t y = 0;
      while(','<=command[i+y] && command[i+y]<='9') 
        {
          y++;
        };
        String number = command.substring(i,i+y);
        i += y - 1;
        G_len = number.length()+2;
        temp = toNumber(number);
        break;
    };
  };
  static char fldr[50] ="/";
  static uint8_t fldr_pointer = 1;
  static uint8_t jump_adress_storage_pointer =0;
    String tada ="";
    int target =0;
  switch(temp)
    {
      case 0:
        SD_mount();
        break;

      case 1:
        if (fldr[fldr_pointer-1] != '/')
        {
          fldr[fldr_pointer]='/';
          fldr_pointer++;
        };
        for (int i = fldr_pointer;i<delka+fldr_pointer-G_len;i++)
          fldr[i] = command[i-fldr_pointer+G_len];
        fldr_pointer += delka-G_len;
        Serial.print(F("changing directory to: "));        
        Serial.println(fldr);
        sd.chdir(fldr);
        break;
      case 2:
      
      /* for (int i = fldr_pointer-1;i>=0;i--)
        {
          if(fldr[i] != '/')
            fldr[i] = '\0';
          else
          {
            if(i>0)
              fldr[i] = '\0';
            Serial.print(F("fldr is: "));
            Serial.println(fldr);
            fldr_pointer = i+1;
            break;
          };          
        };*/
        dir_up(&fldr_pointer,fldr);
        sd.chdir(fldr);
        break;
      case 3:
        printDirectory();
        break;
      case 4:
      if (fldr[fldr_pointer-1] != '/')
        {
          fldr[fldr_pointer]='/';
          fldr_pointer++;
        };
        for (int i = fldr_pointer;i<delka+fldr_pointer-G_len;i++)
          fldr[i] = command[i-fldr_pointer+G_len];
        fldr_pointer += delka-G_len;
        Serial.print(F("Selected file is: "));
        Serial.println(fldr);
        break;
      case 5:
        jump_adress_storage_pointer = 0;
        file.open(fldr,O_READ);
        executing = 1;
        break;
      case 6:
          Serial.print("waiting for: ");
          tada = command.substring(G_len,delka);
          target = toNumber(tada);
          Serial.println(target);
          delay(target);
        break;
      case 7:
        Serial.print(F("file closed"));
        file.close();
        dir_up(&fldr_pointer,fldr);
        executing = 0;
        break;
       case 8: 
          tada = command.substring(G_len);
          target = toNumber(tada);
          file.seekCur(jump_adress_storage[target]-file.curPosition());
        break;
       case 9:
        jump_adress_storage[jump_adress_storage_pointer]= file.curPosition();
        jump_adress_storage_pointer++;
        break;
      case 10:
        for (uint8_t i=0;i<NUMBER_OF_JOINTS;i++)
        { 
          joint[i].angle=0;
          movingjoints[num_movingjoints]= i;
          num_movingjoints++;
          angle_calc(i);
        };
        sort_joints();
        start = 1;
        Serial.println(F("Homing"));
        break;
      default:
        break;
    };

}
void joint_evaluation(String command) //plaintext data parser, should be improved
{
  uint8_t join;
  static float sp = DEFAULT_SPEED;
  int delka = command.length();
  char character = 0;
  float temp;
  uint8_t gotinfo = 0;
  for (int i = 0; i < delka; i++)
  {
    if (command[i] == 'J' || command[i] == 'A' || command[i] == 'S')
    {
      character = command[i];
      gotinfo++;
    }
    else if ((command[i] >= 44) && (command[i] <= 57))
    {
      uint8_t y = 0;
      while(44<=command[i+y] && command[i+y]<=57) 
      {
        y++;
      };
      String number = command.substring(i,i+y);
      i += y - 1;

      temp = toNumber(number);
      gotinfo++; 
    }



    if (gotinfo >= 2)
    {
      gotinfo = 0;
      if (character == 'J') join = temp;
      else if (character == 'A')
        {
          joint[join].angle = temp;
          movingjoints[num_movingjoints] = join;
          num_movingjoints++;
          angle_calc(join);
        }
      else if (character == 'S')sp = temp;
    };
  }; 
      joint[join].targetspeed = sp;
      joint[join].calcPause(joint[join].targetspeed,joint[join].konst);
      Serial.print(F("Joint: "));
      Serial.print(join);
      Serial.print(F(" will move to angle: "));
      Serial.print(joint[join].angle, 3);
      Serial.print(F(" ° with speed of: "));
      Serial.print(joint[join].targetspeed);
      Serial.println(F(" °/s"));
      sort_joints();
      Serial.println("sorted");
      Serial.println(movingjoints[0]);
    
}
void write_one(uint8_t pin, volatile uint8_t *port)
{
    *port |= pin;
}
void write_zero(uint8_t pin,volatile uint8_t *port)
{
    *port &= ~pin;
}
void angle_calc(uint8_t join)
{
    float angle = (joint[join].angle - joint[join].currentangle);
    if(angle < 0)
        write_one(joint[join].dir_pin,joint[join].port);
    else
        write_zero(joint[join].dir_pin,joint[join].port);
    angle = abs(angle);
    angle = joint[join].ratio * angle * 100 * MICROSTEPS / 180;
    joint[join].steps = round(angle);
}
void dir_up(uint8_t *pntr, char folder[50])
{ 
for (int i = (*pntr)-1;i>=0;i--)
        {
          if(folder[i] != '/')
            folder[i] = '\0';
          else
          {
            if(i>0)
            folder[i] = '\0';
            Serial.print(F("fldr is: "));
            Serial.println(folder);
            *pntr = i+1;
            break;
          };          
        };  
}
void sort_joints() //sort the joins, for debugging purposes
{
  uint8_t sorted = 0;
  while(sorted != num_movingjoints)
  {
    sorted = 0;
    for(uint8_t i =0;i<num_movingjoints;i++)
    { 
      if(movingjoints[i]==0xFF || i==num_movingjoints-1)
      {
      sorted++;
      break;
      };
      if (joint[movingjoints[i]].pause<=joint[movingjoints[i+1]].pause)
        sorted++;
      else
      {
        uint8_t helper = movingjoints[i];
        movingjoints[i]= movingjoints[i+1];
        movingjoints[i+1]=helper;
      };
    };
  };
}
uint32_t half_mikros() // this returns timer value, with precision of half a microsecond, it works, but it should probalby be reworked
{
SREG &= ~(1<<7);
half_mikrosec += (TCNT1H<<8)| TCNT1L;
TCNT1L = 0;
TCNT1H = 0;
SREG |= 1<<7;
return half_mikrosec;
}
