/*  Remote GNI-r3 rotator controller code for Arduino
    ========================================================
    Ver. 2.40 - HRD bugs new fix ang degree character added
    Intended Primary for RAU/RAK rotators and Arduino Nano board
    Works with 2 relays or TI DRV8871 motor driver
    For information about the TI DRV88871, see
    http://www.ti.com/product/DRV8871

    For information on the AlfaSpid protocol, see
    http://alfaradio.ca/downloads/program_info/Program_format-Komunicacji-2005-08-10.pdf

    The project is based on implementation of Calvin Li, see
    https://gist.github.com/calvinli/589a98242759e96634c0

    The schematic may be found in http://hf5l.pl/en/gni-r3-antenna-rotor-controller/

    Important notes
    -----------------
    - Intended for azimuth rotation only
    - Software limit switches are set at compile-time.
    - Position data is saved after reaching the target or STOP (pushbutton or command).

    Copyright (c) 2017/8/9 SP5GNI

    Redistribution and use in source and binary forms, with
    or without modification, are permitted provided that the
    following conditions are met:

    1. Redistributions of source code must retain the above
       copyright notice, this list of conditions and the
       following disclaimer.
    2. Redistributions in binary form must reproduce the
       above copyright notice, this list of conditions and
       the following disclaimer in the documentation and/or
       other materials provided with the distribution.
    3. Commercial use of this software, and assiciated
       schematics and layouts is not allowed without written
       permission from Miroslaw Sadowski SP5GNI.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
    CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
    WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
    USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
    USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
    OF SUCH DAMAGE
*/


/* A safety timeout that kills the motor
   if the sense line doesn't register any pulse
   for a certain amount of time.
   RAU rotator turn sped is 3-4°/s,
   Pulse every 0.275 sec (for 13.8V supply).
*/
#define MOTOR_TIMEOUT_MS 400

/* Software limit switch boundaries
   +360° offset should be substracted!
*/
#define AZ_MIN 180 // LOW Limit settable! -180 to 0 (360 MUST be added, as AZ_MIN is shifted +360)
#define AZ_MAX 900 // HIGH Limit settable! 360 to 540 (360 MUST be added, as AZ_MAX is shifted +360)
#define ROT_MIN 5  // Dead zone settable! from 1 to 30 degrees
/* real rotation in the range 360 +/-180 deg
  (-180 to +540 deg ) max
   It may be changed for example to 360 +/-90 deg
*/

#include <EEPROM.h>

/*  LCD display
    LCM1602 module
    SCL -> A5
    SDA -> A4
*/
#include <Wire.h>   // standard library
#include <LiquidCrystal_I2C.h> // download from: https://bitbucket.org/fmalpartida

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// Setting address for 0x27 (PCF8574T) or 0x3F (PSC8574AT)

#define AZ_ADDR 10
#define ANT_ADDR 12
#define EEPROM_MAGIC_ADDR 14
#define EEPROM_MAGIC_VAL 0x19

/* Declarations */
void brake_az();
void forward_az();
void reverse_az();
void check_motors();
void az_pulse_monitor();

/* Global variables */
char serial_char_in; // byte data
char command[13] = {}; //array 13 bytes
char response[12] = {}; // array 12 bytes for Rot2Prog
char msg[16]; // LCD text array

/* These are strictly positive (due to +360° offset) */
unsigned int azPos; // <0; 360) from EEPROM
unsigned int azAnt; // for example <180; 900>,  shifted, real position is <-180; +540>
unsigned int azMoveTo; // <360; 720) shifted, from PC - AlfaSpid protocole

/* 2-byte variable -32768 do 32768 */
int rotate; // should be always <-180; +180>
int azAntReal; // real position of the rotator <-180; +540>

/* the time when the last pulse was detected,
   or backlight was switched on.
   Counted in ms from the start of this program.
*/
volatile unsigned long azLastMoved = 0;

boolean azForward = false;
boolean forward = true;
boolean azReverse = false;

void setup()

{
  pinMode(4, OUTPUT); // signal green LED
  pinMode(7, OUTPUT); // IN1 output for motor
  pinMode(8, OUTPUT); // IN2 output for motor
  pinMode(9, INPUT);  // Forward pushbutton
  pinMode(10, INPUT); // Reverse pushbutton
  pinMode(11, INPUT); // STOP pushbutton

  interrupts();  //enable INT

  /* stop motor if moving */
  brake_az();

  /* Read positions data from EEPROM, if it's there */
  if ((EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VAL) && (digitalRead(11) == LOW)) {
    EEPROM.get(AZ_ADDR, azPos);
    EEPROM.get(ANT_ADDR, azAnt);

  } else {

    /* Reset of the antenna position
       to 180° South. May be changed to another value, i.e. 0° North.
       Keep STOP pushbutton pressed when applying the power or reset
    */
    azPos = 180; // south; may be changed to 0 (north)
    azAnt = 360 + azPos;
    EEPROM.put(ANT_ADDR, azAnt);
    EEPROM.put(AZ_ADDR, azPos);
    delay(100);
    digitalWrite(4, LOW);   // turn on the signal LED
  }

  /* Display stored values of the azimuth
    and real antenna position */

  lcd.begin(16, 2); // Inicjalize LCD 2x16

  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0); // Set cursor

  /* Display welcome message.May be changed for example to "Azimuth: SP5GNI" etc. (15 characters alltogether max.) */
  snprintf(msg, 17, "Azymut:  Hello!");
  lcd.print(msg);
  delay(1000);
  lcd.setCursor(0, 1);
  snprintf(msg, 17, "Antena:"); //po polsku. May be changed to "Antenna:" (8 characters max)
  lcd.print(msg);

  azMoveTo = azAnt;

  Serial.begin(1200); // data rate

  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);

  attachInterrupt(0, az_pulse_monitor, RISING);

  /* Create custom LCD character
  */

  byte degChar3[8] = {
    0b01100,
    0b10010,
    0b10010,
    0b01100,
    0b00000,
    0b00000,
    0b00000,
    0b00000
  };
  lcd.createChar(3, degChar3);

}

void loop() //                                         ***     START    ***

{

  /* Move forward manually, right hand pushbutton is on */
  if ((digitalRead(9) == HIGH) && (azAnt < AZ_MAX))
  {
    brake_az();
    lcd.backlight();
    delay(500);
    forward_az();
    azForward = true;
    azReverse = false;
    azMoveTo = 721; // fake for manual operation

  }

  /* Move reverse manually, left pushbutton on */
  else if ((digitalRead(10) == HIGH) && (azAnt > AZ_MIN))
  {
    brake_az();
    lcd.backlight();
    delay(500);
    reverse_az();
    azForward = false;
    azReverse = true;
    azMoveTo = 721;

  }

  /* Check if motor is rotating without control */

  check_motors();

  /* Check and switch-off  backlight if ther was no activity 3 minutes = 180000 msec
     The value 180000 in [ms] may be changed according to preferences!
  */
  if ((millis() - azLastMoved) > 180000) {
    lcd.noBacklight();   // turn the backlight off
    digitalWrite(4, LOW);   // turn the LED on
    delay(300);
  }

  /* check for STOP pushbutton */
  if (digitalRead(11) == HIGH) {
    lcd.backlight();
    azLastMoved = millis();
    azForward = false;
    azReverse = false;
  }


  /* Display the azimuth
     and the real antenna position */
  azPos = azAnt % 360;
  lcd.setCursor(8, 0); // Set cursor
  snprintf(msg, 9, "%u\3    ", azPos); // deg character
  lcd.print(msg);
  azAntReal = azAnt - 360;
  lcd.setCursor(8, 1);
  snprintf(msg, 9, "%i\3    ", azAntReal);
  lcd.print(msg);


  /* Read-in AlfaSpid command */
  if (Serial.readBytes(command, 13) == 0) {
    return;
  }
  /* start byte is always 0x57 and end byte is 0x20
    if this is not the case, throw out the command */
  if (command[0] != 0x57 || command[12] != 0x20) {
    while (Serial.available()) {
      Serial.read();
    }
    return;
  }

  /*  Proper command is stored in the buffer
      Check for STOP command.
      STOP command returns the position stopped at.
  */
  if (command[11] == 0x0F) {
    brake_az();
    lcd.backlight();
    azLastMoved = millis();
    digitalWrite(4, LOW);   // turn the LED on
    delay(300);
    EEPROM.put(ANT_ADDR, azAnt);
    delay(100);
    azPos = azAnt % 360;
    delay(100);
    EEPROM.put(AZ_ADDR, azPos);


    // azAnt changed to ((azAnt % 360) + 360)

    response[0]  = 0x57;
    response[1]  = ((azAnt % 360) + 360) / 100;
    response[2]  = (((azAnt % 360) + 360) % 100) / 10;
    response[3]  = ((azAnt % 360) + 360) % 10;
    response[4]  = 0x00;
    response[5]  = 0x01;
    response[6]  = 0x03;
    response[7]  = 0x06;
    response[8]  = 0x00;
    response[9]  = 0x00;
    response[10]  = 0x00;
    response[11]  = 0x20;

    Serial.write(response, 12);

    return;
  }


  /* Check for STATUS command
     These can come at any time,
     even while the rotator is moving.
     They are expected to always give the current
     instantaneous position of the rotator.
  */
  if (command[11] == 0x1F) {

    /* Write-out AlfaSpid output.
    */

    response[0]  = 0x57;
    response[1]  = ((azAnt % 360) + 360) / 100;
    response[2]  = (((azAnt % 360) + 360) % 100) / 10;
    response[3]  = ((azAnt % 360) + 360) % 10;
    response[4]  = 0x00;
    response[5]  = 0x01;
    response[6]  = 0x03;
    response[7]  = 0x06;
    response[8]  = 0x00;
    response[9]  = 0x00;
    response[10]  = 0x00;
    response[11]  = 0x20;

    delay(50); // dodane bug w HRD 6.6.x 2019

    Serial.write(response, 12);
    return;
  }

  /* THE MOST IMPORTANT! Check for SET command
     azMoveTo: Azimuth value sent by the software (DXView etc.)
     There is no response to SET commands.
  */
  if (command[11] == 0x2F) {
    /* azMoveTo - the azimuth value required by the program
       <360; 720) shifted from PC - AlfaSpid protocole
       There is no response to SET commands.
    */
    azMoveTo = (command[1] - 0x30) * 100 +
               (command[2] - 0x30) * 10  +
               (command[3] - 0x30);


    /* Added to fix HRD Rotator bug
    */
    if (azMoveTo <= 359) {
      azMoveTo = azMoveTo + 360;
    }
    if (azMoveTo >= 720) {
      azMoveTo = azMoveTo - 360;
    }

    digitalWrite(4, HIGH); // switch-off LED
    lcd.backlight();
    delay(500);

    /* rotate: Should be finally <-180; +180>
       The shortest way to the target.
       Shows the value and direction of rotation
    */
    rotate = (azMoveTo - 360) - (azAnt % 360); // Range (-359; 359)

    /* Do not rotate for small changes less than ROT_MIN */
    if ((rotate < ROT_MIN) && (rotate > - ROT_MIN)) {
      return;
    }

    /* correction for optimum rotation.
       rotation direction may be changed
    */
    if (rotate > 180) {
      rotate = rotate - 360;
    }

    if (rotate < -180) {
      rotate = rotate + 360;
    }

    /* software limit switch ensure positions are kept within range */
    if (azAnt + rotate > AZ_MAX) {
      rotate = rotate - 360;
    }
    else if (azAnt + rotate < AZ_MIN) {
      rotate = rotate + 360;
    }
    /* end of SET command reading and processing
    */
  }

  command[11] = 0x00;

  azForward = (rotate > 0);
  azReverse = (rotate < 0);

  /* start moving */
  if (azForward) {
    if (forward == false) {
      brake_az();
      delay(500); // stop when changing the direction
    }
    forward = true;
    forward_az();

  } else if ((azAnt % 360) != (azMoveTo - 360))
  {
    if (forward) {
      brake_az();
      delay(500);
    }
    forward = false;
    reverse_az();
  }

  /* End of loop */
  return;

}

void brake_az() {
  digitalWrite(7, LOW);   // IN1 output SET 0
  digitalWrite(8, LOW);   // IN2 SET 0

}

void forward_az() {
  azLastMoved = millis();
  digitalWrite(7, HIGH);   // IN1 SET 1
  digitalWrite(8, LOW);   // IN2 SET 0

}
void reverse_az() {
  azLastMoved = millis();
  digitalWrite(7, LOW);   // IN1 SET 0
  digitalWrite(8, HIGH);   // IN2 SET 1
}

/* Check if motor is rotating without control */
void check_motors() {
  if (azLastMoved > 0) {
    if (azLastMoved < millis() - MOTOR_TIMEOUT_MS) {
      brake_az();
    }
  }
}

/* IMPORTANT ISR for azimuth control */
void az_pulse_monitor()

/* These ISRs will stop the motor when
   it reach the desired position.

   We trigger on rising edge because the
   sense line is active-low, so the ISR
   will run only after the pulse is completed.
*/
{
  {
    digitalWrite(4, LOW); // LED on
    if (azForward) {
      ++azAnt;
    } else if (azReverse) {
      --azAnt;
    }

    /* The time when the last pulse was detected
       counted in ms from the start of this program.
       "millis" is the time from the start of the program
    */
    azLastMoved = millis();
    azPos = azMoveTo - 360;

  }

  /* software limit switch ensure rotation is kept within range */
  if ((azAnt >= AZ_MAX) || (azAnt <= AZ_MIN)) {
    brake_az();
    EEPROM.put(ANT_ADDR, azAnt);
    azPos = azAnt % 360;
    EEPROM.put(AZ_ADDR, azPos);
    digitalWrite(4, LOW);
    azForward = false;
    azReverse = false;
    return;
  }

  /* check for STOP pushbutton */
  if (digitalRead(11) == HIGH) {
    brake_az();
    delay(300);
    EEPROM.put(ANT_ADDR, azAnt);
    delay(100);
    azPos = azAnt % 360;
    delay(100);
    EEPROM.put(AZ_ADDR, azPos);
    azForward = false;
    azReverse = false;
    return;
  }


  if (azAnt % 360 == azPos)   {
    brake_az();
    EEPROM.put(AZ_ADDR, azPos);
    EEPROM.put(ANT_ADDR, azAnt);
    azForward = false;
    azReverse = false;
  }

  delay(20);
  digitalWrite(4, HIGH);  // LED off

}


