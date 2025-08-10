#include <USB.h>
#include <USBHIDMouse.h>
#include <math.h> 

USBHIDMouse Mouse;  

//// БАЗОВЫЕ ПАРАМЕТРЫ
#define VRX_PIN 3 // X-ось (аналоговый вход)
#define VRY_PIN 2 // Y-ось (аналоговый вход)
#define SW_PIN  17 // Кнопка джойстика (цифровой вход)

//// БЫСТРОДЕЙСТВИЕ
#define POOLING_RATE 100  // Частота опроса в секунду
#define TICK_DELAY 1000/POOLING_RATE // Задержка между опросами сенсоров
#define SECONDS_TO_LOW_RATE 30  // Время до перехода в энергосберегающий режим

static unsigned long last_mouse_move_time = 0;

//// КАЛИБРОВКА ДЖОЙСТИКА
#define JOYSTICK_ROTATION_ANGLE_DEGREES -60  // Угол поворота джойстика

// Предварительно вычисляем синус и косинус угла (в радианах)
const float angle_rad = JOYSTICK_ROTATION_ANGLE_DEGREES * PI / 180.0;
const float cos_angle = cos(angle_rad);
const float sin_angle = sin(angle_rad);

// Определяем центры
int centerX = 1936;
int centerY = 1823;

// Задаем максимальную магнитуду для направлений
float MAX_MAG_UP    = 2727;   
float MAX_MAG_DOWN  = 2202;  
float MAX_MAG_LEFT  = 2400;  
float MAX_MAG_RIGHT = 2620;

// Мертвые зоны
#define DEADZONE 50 

#define DEADZONE_UP DEADZONE + 0
#define DEADZONE_DOWN DEADZONE + 0
#define DEADZONE_LEFT DEADZONE + 0
#define DEADZONE_RIGHT DEADZONE + 0

// СГЛАЖИВАНИЕ
#define BUFFER_SIZE 5 // ТОЛЬКО нечетные

int rawXBuffer[BUFFER_SIZE] = {0};
int rawYBuffer[BUFFER_SIZE] = {0};
int bufferIndex = 0;

//// СКОРОСТЬ КУРСОРА
#define MOUSE_DEFAULT_SPEED 1  // Базовая скорость мыши
#define MOUSE_MAX_SPEED 22  // Максимальная скорость мыши

#define SPEED_CURVE_COEF 1.25  // Чем выше, тем "вальяжнее" начало

struct Coordinates {
  int x;
  int y;
};

struct Shift {
  float magnitude;
  float angle;
};

int median(int* buffer) {
  // Сортируем буфер
  int sortedBuffer[BUFFER_SIZE];

  memcpy(sortedBuffer, buffer, sizeof(sortedBuffer));

  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    for (int j = i + 1; j < BUFFER_SIZE; j++) {
      if (sortedBuffer[i] > sortedBuffer[j]) {
        // Меняем местами
        int temp = sortedBuffer[i];
        sortedBuffer[i] = sortedBuffer[j];
        sortedBuffer[j] = temp;
      }
    }
  }

  // Возвращаем центральное значение
  return sortedBuffer[BUFFER_SIZE / 2];
}

int deadzone_filter(int paramX, int paramY) {
  // paramX — верх (+) и низ (-)
  // paramY — лево (-) и право (+)

  int param = 0;
  int filter_deadzone = DEADZONE;

  if (paramX != 0) {
    param = paramX;

    if (paramX > 0) {
      filter_deadzone = DEADZONE_UP;
    } else {
      filter_deadzone = DEADZONE_DOWN;
    }
  }

  if (paramY != 0) {
    param = paramY;

    if (paramY > 0) {
      filter_deadzone = DEADZONE_RIGHT;
    } else {
      filter_deadzone = DEADZONE_LEFT;
    }
  }

  int filtered = 0;

  if (abs(param) > filter_deadzone) {
    filtered = param;
  }
    
  return filtered;
}

void calibrateJoystick(int &centerX, int &centerY, int samples = 500) {
  long sumX = 0;
  long sumY = 0;

  delay(300);

  Serial.println("Stick calibration, don't touch!");

  for (int i = 0; i < samples; i++) {
    sumX += analogRead(VRX_PIN);
    sumY += analogRead(VRY_PIN);
    delay(10);  // Небольшая задержка для стабилизации
  }

  centerX = sumX / samples;
  centerY = sumY / samples;

  Serial.print("Center X: ");
  Serial.println(centerX);
  Serial.print("Center Y: ");
  Serial.println(centerY);
}

void calibrateJoystickMaxStrength(int duration_ms = 10000) {
  Serial.println("Move stick to all extremes for calibration...");
  delay(1000);

  float max_up = 0;
  float max_down = 0;
  float max_left = 0;
  float max_right = 0;

  unsigned long start_time = millis();

  while (millis() - start_time < duration_ms) {
    int x = analogRead(VRX_PIN);
    int y = analogRead(VRY_PIN);

    int correctedX = x - centerX;
    int correctedY = y - centerY;

    // Поворот координат
    float rotatedX = correctedX * cos_angle - correctedY * sin_angle;
    float rotatedY = correctedX * sin_angle + correctedY * cos_angle;

    float magnitude = sqrt(rotatedX * rotatedX + rotatedY * rotatedY);

    float angle = atan2(rotatedY, rotatedX);

    if (angle >= -PI/4 && angle < PI/4) {
      if (magnitude > max_up) max_up = magnitude;
    } else if (angle >= PI/4 && angle < 3*PI/4) {
      if (magnitude > max_right) max_right = magnitude;
    } else if (angle >= -3*PI/4 && angle < -PI/4) {
      if (magnitude > max_left) max_left = magnitude;
    } else {
      if (magnitude > max_down) max_down = magnitude;
    }

    delay(5);
  }

  // Записываем
  MAX_MAG_UP = max_up;
  MAX_MAG_DOWN = max_down;
  MAX_MAG_LEFT = max_left;
  MAX_MAG_RIGHT = max_right;

  Serial.println("Calibration complete:");
  Serial.print("MAX_MAG_UP: "); Serial.println(MAX_MAG_UP);
  Serial.print("MAX_MAG_DOWN: "); Serial.println(MAX_MAG_DOWN);
  Serial.print("MAX_MAG_LEFT: "); Serial.println(MAX_MAG_LEFT);
  Serial.print("MAX_MAG_RIGHT: "); Serial.println(MAX_MAG_RIGHT);
}

int need_calibration = 1;

void calibration() {
  if (need_calibration == 1) {
    digitalWrite(15, HIGH);
    delay(500); 

    calibrateJoystick(centerX, centerY);

    delay(500); 
    digitalWrite(15, LOW);
    delay(500); 
    digitalWrite(15, HIGH);
    calibrateJoystickMaxStrength();
    digitalWrite(15, LOW);

    need_calibration = 0;
  } 
}

void process_button() {
  int btn = digitalRead(SW_PIN);

  if (btn == LOW) {
    Mouse.click(MOUSE_LEFT);
  }
}

Coordinates get_raw_coordinates() {
  Coordinates result;

  int raw_x = analogRead(VRX_PIN);
  int raw_y = analogRead(VRY_PIN);

  // Корректировка по смещение от центра
  result.x = raw_x - centerX;
  result.y = raw_y - centerY;

  return result;
}

Coordinates get_buffered_coordinates(Coordinates coordinates) {
  Coordinates result;

  // Записываем в буфер
  rawXBuffer[bufferIndex] = coordinates.x;
  rawYBuffer[bufferIndex] = coordinates.y;

  // Обновляем индекс (по кругу)
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  // Усредняем показатели
  result.x = median(rawXBuffer);
  result.y = median(rawYBuffer);

  return result;
}

Coordinates get_filtered_coordinates(Coordinates coordinates) {
  Coordinates result;

  result.x = deadzone_filter(coordinates.x, 0);
  result.y = deadzone_filter(0, coordinates.y);

  return result;
}

Coordinates get_normalized_coordinates(Coordinates coordinates, Shift shift) {
  Coordinates result;

  result.x = round((float)coordinates.x / shift.magnitude);
  result.y = round((float)coordinates.y / shift.magnitude);

  return result;
}

Coordinates get_rotated_coordinates(Coordinates coordinates) {
  Coordinates result;

  result.x = coordinates.x * cos_angle - coordinates.y * sin_angle;
  result.y = coordinates.x * sin_angle + coordinates.y * cos_angle;

  return result;
}  

Shift get_shift(Coordinates coordinates) {
  Shift result;

  float x = coordinates.x;
  float y = coordinates.y;

  result.magnitude = sqrtf(x * x + y * y);
  result.angle = atan2f(y, x);

  return result;
}

void delay_next_tick() {
  if (last_mouse_move_time > SECONDS_TO_LOW_RATE * 1000 * 1000 ) {
    delay(TICK_DELAY*3);
  } else {
    delay(TICK_DELAY);
  }
}

void setup() {
  pinMode(15, OUTPUT); // На WeMos S2 Mini встроенный LED на GPIO15

  USB.begin();
  Mouse.begin();

  Serial.begin(9600);

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);

  analogReadResolution(12); // Диапазон 0–4095 для ESP32-S2
}

void loop() {
    calibration(); // Запускаем калибровку если требуется

    process_button(); // Обрабатываем нажатие кнопки

    Coordinates coordinates = get_raw_coordinates(); // Получаем «сырые» координаты
    coordinates = get_buffered_coordinates(coordinates); // Сглаживаем координаты через буфер
    coordinates = get_filtered_coordinates(coordinates); // Фильтруем по мертвой зоне

    // Вычисляем силу и угол наклона джойстика
    Shift shift = get_shift(coordinates);

    float normX = 0, normY = 0;

    if (shift.magnitude > 0) {
      // Нормализуем координаты по силе направления
      float normX = coordinates.x / shift.magnitude;
      float normY = coordinates.y / shift.magnitude;

      // Поворачиваем координаты на нужный угол
      float rotatedX = normX * cos_angle - normY * sin_angle;
      float rotatedY = normX * sin_angle + normY * cos_angle;

      // Вычисляем ускорение относительно угла наклона джойстика
      float max_magnitude = 1.0;

      // Определяем сектор
      if (shift.angle >= -PI/4 && shift.angle < PI/4) {
        max_magnitude = MAX_MAG_UP;
      } else if (shift.angle >= PI/4 && shift.angle < 3*PI/4) {
        max_magnitude = MAX_MAG_RIGHT;
      } else if (shift.angle >= -3*PI/4 && shift.angle < -PI/4) {
        max_magnitude = MAX_MAG_LEFT;
      } else {
        max_magnitude = MAX_MAG_DOWN;
      }

      //Находим скорость мыши
      float strength = constrain(shift.magnitude / max_magnitude, 0.0, 1.0);
      float curved_strength = powf(strength, SPEED_CURVE_COEF);

      int mouse_speed = round(MOUSE_DEFAULT_SPEED + MOUSE_MAX_SPEED * curved_strength); 

      // Вычисляем само смещение мышки и двигаем курсор
      int deltaX = rotatedX * mouse_speed;
      int deltaY = rotatedY * mouse_speed;

      Mouse.move(deltaX, deltaY);

      last_mouse_move_time = 0;
    } else {
        static unsigned long last_check = 0;
        unsigned long now = millis();

        if (last_check == 0) {
          last_check = now;  // Первичная инициализация
        }

        last_mouse_move_time += (now - last_check);
        last_check = now;
    }

    delay_next_tick();
} 