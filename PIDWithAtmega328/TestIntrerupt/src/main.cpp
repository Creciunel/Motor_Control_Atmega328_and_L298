#include <Arduino.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

#define EN_PIN 6
#define L_PIN 5
#define R_PIN 7
#define S_PIN 3
#define ENCBTN 11
#define ENCHA 10

double Setpoint, Input, Output, SET;
int lastCount, counts;
volatile int count;
double filtVal;
double k;
unsigned long lastTime, time;
long Time1, Time2, Time3 = 0;
byte lastFlag;
byte dirFlag;
String pwm;

double Kp = 2, Ki = 0.1, Kd = 0.01;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void initTimers()
{
  TCCR0A = (1 << WGM01); // set ctc bit
  OCR0A = 255;
  TIMSK0 = (1 << OCIE0A);
  sei();
  TCCR0B = (1 << CS01) | (1 << CS00); // start at 64 prescaler
}

void ISRmSpeed()
{
  count++;
}

void initFreq()
{
  // Пины D5 и D6 - 4 кГц
  TCCR0B = 0b00000010; // x8
  TCCR0A = 0b00000001; // phase correct
}

void setup()
{
  // Init
  k = 0.7;
  time = 0;
  lastTime = 0;
  count = 0;
  lastCount = 0;
  Setpoint = 0;
  Time2 = 0;
  lastFlag = 0;
  dirFlag = 0;
  SET = eeprom_read_byte(0);

  // initTimers();

  // MODE
  pinMode(EN_PIN, OUTPUT);
  pinMode(L_PIN, OUTPUT);
  pinMode(R_PIN, OUTPUT);
  pinMode(S_PIN, INPUT);
  pinMode(ENCBTN, INPUT);
  pinMode(ENCHA, INPUT);
  // set
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(L_PIN, HIGH);
  digitalWrite(R_PIN, LOW);

  attachInterrupt(1, ISRmSpeed, FALLING); // Level to low
  // Serial fr test
  Serial.begin(115200);
  Serial.setTimeout(5);
  Serial.println("Filtred,SET");
  // Serial.println("Speed");
  myPID.SetMode(AUTOMATIC);
  // lcd
  lcd.begin(16, 2, 8);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("kp: ");
  lcd.setCursor(9, 0);
  lcd.print("Kd: ");
  lcd.setCursor(5, 1);
  lcd.print("Ki: ");

  // initFreq();
}
// Median filter for 3 number
int median(double newVal)
{
  static double buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3)
    count = 0;
  return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
}

String setSpeed(double speed)
{
  speed = speed / (33) * (255); // maping
  speed = constrain(speed, 0, 255);

  digitalWrite(L_PIN, HIGH);
  digitalWrite(R_PIN, LOW);
  analogWrite(EN_PIN, (int)speed);

  return (String)speed;
}

void encoderSpeed()
{
  if (millis() - lastTime >= 1000)
  {
    noInterrupts();
    count /= 20;
    count -= lastCount;
    counts = count;
    interrupts();

    // counts /= 3;
    filtVal += ((counts)-filtVal) * k;
    Input = median(filtVal);

    lcd.setCursor(0, 0);
    lcd.print((int)Input);

    myPID.Compute();
    setSpeed(Output);

    Serial.println(Input);
    Serial.print(" ");
    Serial.println(SET);

    lastCount = counts;
    lastTime = millis();
  }
}

void encoderSet()
{
  byte flag = digitalRead(ENCHA);
  if (flag != lastFlag)
  {
    if (SET == NAN)
    {
      SET = 0;
    }
    if (dirFlag)
    {
      SET--;
      eeprom_write_byte(0, SET);
    }
    else
    {
      SET++;
      eeprom_write_byte(0, SET);
    }
    lastFlag = flag;
  }
  if (!digitalRead(ENCBTN))
  {
    if (dirFlag)
    {
      dirFlag = 0;
    }
    else
    {
      dirFlag = 1;
    }
  }
}

void loop()
{
  while (Serial.available() > 1)
  {
    char key = Serial.read();
    float val = Serial.parseFloat();

    switch (key)
    {
    case 'p':
      Kp = val;
      break;
    case 'i':
      Ki = val;
      break;
    case 'd':
      Kd = val;
      break;
    default:
      lcd.setCursor(3, 0);
      lcd.print("err");
      break;
    }

    myPID.SetTunings(Kp, Ki, Kd);

    lcd.setCursor(3, 0);
    lcd.print(myPID.GetKp());
    lcd.setCursor(12, 0);
    lcd.print(myPID.GetKd());
    lcd.setCursor(8, 1);
    lcd.print(myPID.GetKi());
  }
  Serial.println((int)Input);

  encoderSet();
  Setpoint = 50;
  encoderSpeed();
}