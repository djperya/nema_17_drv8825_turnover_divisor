// Подключение библиотек
#include <AccelStepper.h> 
#include <GyverOLED.h>
#include <EncButton.h>

#define OLED_SOFT_BUFFER_64 // Буфер на стороне МК
GyverOLED<SSD1306_128x64> oled; // Обьект дисплея

EncButton<EB_TICK, 2, 3, 9> enc; // энкодер с кнопкой <A, B, KEY>
const int RUN_BUTTON_PIN = 12; // дополнительная кнопка на пине 12

int buzzerPin = 11; //Пищалка

#define STEP_PIN 7 // Подключение шагового двигателя к пинам
#define DIR_PIN 6 // Подключение шагового двигателя к пинам

int microstepping ;

#define MS1_PIN 4  // Подключите к пину MS1 на DRV8825
#define MS2_PIN 5  // Подключите к пину MS2 на DRV8825
#define MS3_PIN 8  // Подключите к пину MS3 на DRV8825

// Функция-обработчик прерывания для дополнительной кнопки
bool previousRunState = false;

// Создание объекта шагового двигателя
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

//bool isMoving = false;

// Флаг, указывающий, запущен ли двигатель
volatile bool isRunning = false;
//Меню
#define ITEMS 12

const char i0[] PROGMEM = "  Steps      :";
const char i1[] PROGMEM = "  Speed      :";
const char i2[] PROGMEM = "  MicroStep  :";
const char i3[] PROGMEM = "  Step       :";
const char i4[] PROGMEM = "  Accel      :";
const char i5[] PROGMEM = "  Parameter 5:";
const char i6[] PROGMEM = "  Parameter 6:";
const char i7[] PROGMEM = "  Parameter 7:";
const char i8[] PROGMEM = "  Parameter 8:";
const char i9[] PROGMEM = "  Parameter 9:";
const char i10[] PROGMEM = "  Parameter 10:";
const char i11[] PROGMEM = "  Parameter 11:";

const char* const names[] PROGMEM = {
  i0, i1, i2, i3,
  i4, i5, i6, i7,
  i8, i9, i10, i11
};

uint16_t data[ITEMS];

int step = 2; 
bool flag = true;
bool allowChange = false;
bool menuChange = false;

void setup() {
   Serial.begin(9600);
   enc.setHoldTimeout(8000); // установка таймаута удержания кнопки
   pinMode (buzzerPin, OUTPUT); //BUZZER
   oled.init(); // инициализация экрана
   oled.setContrast(255);  //Яркость экрана
   attachInterrupt(0, isr, CHANGE);
   attachInterrupt(1, isr, CHANGE);
  
   data[0] = 2; //делитель вращения
   data[1] = 300; //скорость вращения
   data[2] = 32; //режим дробления шага 
   data[3] = 200; //шаги на 1 оборот
   data[4] = 500; //Ускорение
  
   setMicrostepping(data[2]);
   pinMode(RUN_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RUN_BUTTON_PIN), runButtonISR, FALLING);
  
// Установка пинов MS1, MS2 и MS3 как выходов
pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
// Установка максимальной скорости и ускорения
  stepper.setMaxSpeed(5000);
  Serial.print("Max Speed set to: ");
  Serial.println(data[1]);
  stepper.setAcceleration(data[4]);
  
   // Направление движения
  stepper.setSpeed(data[1]); // Установите положительное значение для движения вперед

}
void runButtonISR() {
  isRunning = !isRunning;
}
//Переключерие режимов микрошага 1, 2, 4, 8, 16, 32
void setMicrostepping(uint16_t microsteps) {
  microstepping = microsteps;
  if (microsteps == 1) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
    Serial.println("Microstepping: 1");
  } else if (microsteps == 2) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
    Serial.println("Microstepping: 2");
  } else if (microsteps == 4) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
    Serial.println("Microstepping: 4");
  } else if (microsteps == 8) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
    Serial.println("Microstepping: 8");
  } else if (microsteps == 16) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, HIGH);
    Serial.println("Microstepping: 16");
  } else if (microsteps == 32) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, HIGH);
    Serial.println("Microstepping: 32");
  } else if (microsteps == 64) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, HIGH);
    Serial.println("Microstepping: 64");
  }
}

void isr() {
  enc.tickISR(); // специальный тикер прерывания
}

void loop() {
  
  enc.tick();  // опрос энкодера происходит здесь

  int buttonState = digitalRead(RUN_BUTTON_PIN);
  static int lastButtonState = HIGH;

  if (buttonState == LOW && lastButtonState == HIGH) {
    Serial.println("!");
    int stepsToMove = data[3];
    stepper.moveTo((stepsToMove * data[2]) / data[0]);
     Serial.println("move");
     isRunning = true; 
     Serial.println("run"); // Установите флаг isRunning в true
    while (stepper.distanceToGo() != 0) {
    stepper.run();
    
    
    }
     isRunning = false;
     Serial.println("stop"); // Сбросьте флаг isRunning в false
    stepper.stop();
    stepper.setCurrentPosition(0);  // Сброс позиции для следующего оборота
  }
   lastButtonState = buttonState;
    

  if (enc.held()) {
  tone (buzzerPin, 1100);
  delay (50);
  noTone (buzzerPin);
  delay (100);
    menuChange = !menuChange;
    Serial.println("!menuChange");
  }

  if (menuChange) {
    static int8_t pointer = 0;
    if (enc.left() || enc.leftH()) {
      if (flag) {
        pointer = constrain(pointer - 1, 0, ITEMS - 1);
         } else {
    if (pointer == 2) {
      // Ограничиваем изменение значений data[2]
      if (data[pointer] == 32) {
        data[pointer] = 16;
      } else if (data[pointer] == 16) {
        data[pointer] = 8;
      } else if (data[pointer] == 8) {
        data[pointer] = 4;
      } else if (data[pointer] == 4) {
        data[pointer] = 2;
      }else if (data[pointer] == 2) {
        data[pointer] = 1;
      }else if (data[pointer] == 1) {
        data[pointer] = 32;
      }
      setMicrostepping(data[pointer]);
      } else if (pointer == 1) {
                // Изменение для "Speed" кратно 100
                data[pointer] = constrain(data[pointer] - 100, 0, 65535);     
      } else {
        data[pointer]++;
      }
    }
    }
    if (enc.right() || enc.rightH()) {
      if (flag) {
        pointer = constrain(pointer + 1, 0, ITEMS - 1);
         } else {
    if (pointer == 2) {
      // Ограничиваем изменение значений data[2]
      if (data[pointer] == 1) {
        data[pointer] = 2;
      } else if (data[pointer] == 2) {
        data[pointer] = 4;
      } else if (data[pointer] == 4) {
        data[pointer] = 8;
      } else if (data[pointer] == 8) {
        data[pointer] = 16;
      }else if (data[pointer] == 16) {
        data[pointer] = 32;
      }else if (data[pointer] == 32) {
        data[pointer] = 1;
      }
      setMicrostepping(data[pointer]);
      } else if (pointer == 1) {
                // Изменение для "Speed" кратно 100
                data[pointer] = constrain(data[pointer] + 100, 0, 65535);  
      } else {
        data[pointer]--;
      }
    }
    }
    if (enc.click()) {
      flag = !flag;
    }
    oled.clear();
    oled.home();
    for (uint16_t i = 0; i < 8; i++) {
      printMenuItem(pointer > 7 ? i + (pointer - 7) : i);
    }
    for (uint16_t i = 0; i < 8; i++) {
      oled.setCursor(85, i);
      oled.print(data[pointer > 7 ? i + (pointer - 7) : i]);
    }
    printPointer(constrain(pointer, 0, 7));
    oled.update();
  } else {
    if (enc.press()) {
      tone (buzzerPin, 800);
  delay (100);
  noTone (buzzerPin);
  delay (100);
      allowChange = !allowChange;
    }
    if (allowChange) {
      if (enc.right()) {
        data[0] = data[0] ++; //пункт меню 1 который отображаеться на главном экране
        step++;
        data[0] = step;
      } else if (enc.left() && step > 2) {
        data[0] = data[0] --;
        step--;
        data[0] = step;
      }
    }
    oled.clear();
    oled.setScale(1);
    oled.setCursor(40, 1);
    oled.print("Делитель");
    oled.setCursor(40, 3);
    oled.setScale(4);
    oled.print(data[0]);

    if (allowChange) {
      oled.setCursor(0, 16);
      oled.setScale(1);
      oled.print("Change: Allowed");
    } else {
      oled.setCursor(0, 16);
      oled.setScale(1);
      oled.print("Change: Blocked");
    }
    oled.update();
  }
}

void printPointer(uint16_t pointer) {
  if (flag) {
    oled.setCursor(0, pointer);
    oled.setScale(1);
    oled.print(">");
  } else {
    oled.setCursor(120, pointer);
    oled.setScale(1);
    oled.print("<");
  }
}

void printMenuItem(uint16_t num) {
  char buffer[21];
  uint16_t ptr = pgm_read_word(&(names[num]));
  uint16_t i = 0;
  do {
    buffer[i] = (char)(pgm_read_byte(ptr++));
  } while (buffer[i++] != NULL);
oled.setScale(1);
  oled.println(buffer);
}
