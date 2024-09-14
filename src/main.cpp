#include <Arduino.h>     // Библиотека для работы с платформой Arduino
#include <Wire.h>        // Библиотека для работы с шиной I2C
#include <WiFi.h>        // Библиотека для работы с Wi-Fi
#include <ArduinoOTA.h>  // Библиотека для поддержки обновления прошивки по воздуху (OTA)
#include <GyverPID.h>    // Библиотека для реализации PID-регулятора
#include <GyverPortal.h> // Библиотека для создания веб-интерфейса
#include <GyverNTP.h>    // Библиотека для синхронизации времени через NTP
#include <SPI.h>         // Библиотека для работы с шиной SPI
#include <BME280I2C.h>   // Библиотека для работы с датчиком BME280 по I2C
#include <TimerMs.h>     // Библиотека для работы с таймерами

const char *ssid = "";          // Название Wi-Fi сети
const char *password = ""; // Пароль для подключения к Wi-Fi

const int thermistorPin = 33;           // Пин подключения термистора
const float referenceVoltage = 3.3;     // Опорное напряжение для измерений
const float referenceResistor = 100000; // Сопротивление резистора, подключенного к термистору
const float beta = 3950;                // Бета-коэффициент термистора
const float nominalTemperature = 25;    // Номинальная температура для расчета
const float nominalResistance = 100000; // Сопротивление термистора при номинальной температуре

// Структура для хранения показаний датчиков
struct Sensor
{
  float tempNTC; // Температура от термистора NTC
  float temp;    // Температура от датчика BME280
  float hum;     // Влажность от датчика BME280
  float pres;    // Давление от датчика BME280
};

// Структура для управления PWM (импульсной модуляцией) нагревателя и вентилятора
struct PWM
{
  int cooler = 255; // ШИМ для вентилятора (макс. скорость)
  int heater = 0;   // ШИМ для нагревателя
};

// Класс для описания свойств филамента (материала)
class Filament
{
public:
  std::string name; // Название филамента (например, PLA, PETG, TPU)
  int temperature;  // Температура для сушки филамента
  int time;         // Время сушки филамента в минутах

  // Конструктор класса Filament
  Filament(std::string _name, int _temperature, int _time)
  {
    name = _name;
    temperature = _temperature;
    time = _time;
  }
};

// Класс Dryer для управления сушилкой филамента
class Dryer
{
public:
  // Поля класса Dryer
  bool powerOn;        // Включена ли сушилка
  bool isOTA;          // Режим обновления прошивки по воздуху (OTA)
  bool isWIFIconn;     // Есть ли подключение к Wi-Fi
  int modeControl;     // Режим работы сушилки (0 - Динамический, 1 - Ночной, 2 - Ручной)
  int typeFilament;    // Индекс выбранного типа филамента
  int targetTemp;      // Целевая температура для сушки
  int updateGraphics;  // Время обновления графики
  int maxTempTen;      // Максимальная температура тэна
  int maxNightTempTen; // Максимальная температура тэна в ночном режиме

  PWM pwm;                         // Объект для управления нагревателем и вентилятором через PWM
  std::vector<Filament> filaments; // Список доступных филаментов
  Sensor sensor;                   // Объект для хранения показаний датчиков
  TimerMs tmr;                     // Таймер для управления временем сушки

  // Конструктор класса Dryer
  Dryer()
  {
    powerOn = false;
    isOTA = false;
    isWIFIconn = false;
    modeControl = 0;       // Динамический режим по умолчанию
    typeFilament = 0;      // Тип филамента по умолчанию
    targetTemp = 60;       // Целевая температура по умолчанию
    updateGraphics = 60;   // Время обновления графики (секунды)
    maxTempTen = 350;      // Максимальная температура тэна
    maxNightTempTen = 100; // Максимальная температура тэна в ночном режиме
  }

  // Метод для добавления нового филамента в список
  void addFilament(std::string name, int temperature, int time)
  {
    Filament newFilament(name, temperature, time);
    filaments.push_back(newFilament);
  }

  // Метод для получения всех названий филаментов в виде строки через запятую
  std::string getAllFilamentNames()
  {
    std::string names = "";
    for (size_t i = 0; i < filaments.size(); i++)
    {
      names += filaments[i].name;
      if (i < filaments.size() - 1)
      {
        names += ","; // Добавляем запятую между именами филаментов
      }
    }
    return names;
  }

  // Метод для получения филамента по индексу
  Filament getFilamentByIndex(int index)
  {
    if (index >= 0 && index < filaments.size())
    {
      return filaments[index]; // Возвращаем филамент по индексу
    }
    else
    {
      return Filament("Unknown", 0, 0); // Возвращаем филамент по умолчанию, если индекс неверен
    }
  }

  // Метод для выбора режима работы сушилки
  void selectMode(GyverPID &Temp, int index = 0)
  {
    modeControl = index;  // Выбор режима
    if (modeControl == 1) // Ночной режим
    {
      Temp.setLimits(0, maxNightTempTen); // Устанавливаем ограничение для температуры тэна
    }
    else
    {
      Temp.setLimits(0, maxTempTen); // Обычный режим
    }
  }

  // Метод для выбора типа филамента по индексу
  void selectType(int index = 0)
  {
    typeFilament = index;
    Filament filament = getFilamentByIndex(typeFilament);
    tmr.setTime(filament.time * 60 * 1000); // Устанавливаем время сушки в миллисекундах
    targetTemp = filament.temperature;      // Устанавливаем целевую температуру сушки
    tmr.setTimerMode();                     // Перевод таймера в режим работы
  }

  // Метод для запуска сушки
  void start()
  {
    if (tmr.timeLeft() > 0) // Если время сушки не закончилось
    {
      tmr.resume(); // Продолжаем работу таймера
    }
    else
    {
      tmr.start();  // Запускаем таймер
      selectType(typeFilament); // Выбираем тип филамента
    }
  }

  // Метод для остановки сушки
  void stop()
  {
    powerOn = false;  // Выключаем сушилку
    ledcWrite(1, 0);  // Отключаем нагреватель
    if (tmr.active()) // Если таймер активен
    {
      tmr.stop(); // Останавливаем таймер
    }
  }

  // Метод для обновления состояния таймера и проверки завершения сушки
  void tick()
  {
    if (powerOn) // Если сушилка включена
    {
      if (tmr.tick()) // Проверяем, истекло ли время
      {
        stop(); // Останавливаем сушилку
      }
    }
  }
};

GyverPortal ui;
#define PLOT_SIZE 255
int16_t arr[2][PLOT_SIZE];
uint32_t dates[PLOT_SIZE];
const char *names[] = {"Температура", "Влажность"};

BME280I2C bme;
GyverNTP ntp(8);

GyverPID heater(20, 1, 0.8, 500);
GyverPID Temp(500, 2, 2, 500);
Dryer dryer;
GPtime valTime;

void getTime();
void getBME();
void OTA();
void connectToWifi();

float GetTempC()
{
  int adcValue = 0;
  for (int i = 0; i < 100; i++)
  {
    adcValue = adcValue + analogRead(thermistorPin); // Чтение значения АЦП
  }
  adcValue = adcValue / 100;
  float voltage = (adcValue * referenceVoltage) / 4095.0;                          // Рассчитываем напряжение
  float resistance = (voltage * referenceResistor) / (referenceVoltage - voltage); // Рассчитываем сопротивление термистора с обновленной конфигурацией

  // Рассчитать температуру, используя уравнение бета-параметра
  float tempK = 1 / (((log(resistance / nominalResistance)) / beta) + (1 / (nominalTemperature + 273.15)));

  float tempC = tempK - 273.15; // Получаем температуру в градусах Цельсия
  return tempC;
}

void build()
{
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);

  // список имён компонентов на обновление
  GP.UPDATE("power,mode,type,time,manualTemp,heater,heaterLimit,temp,tempBME,humBME");
  // обновление случайным числом
  GP.PAGE_TITLE("Сушилка филамента");
  GP.LABEL("Питание: ");
  GP.SWITCH("power", dryer.powerOn);
  GP.BREAK();
  GP.LABEL_BLOCK("Режим:");
  GP.SELECT("mode", "Динамический,Ночной", dryer.modeControl);
  GP.BREAK();
  GP.LABEL_BLOCK("Филамент:");
  GP.SELECT("type", dryer.getAllFilamentNames().c_str(), dryer.typeFilament);
  GP.BREAK();
  GP.LABEL("Время", "t3");
  GP.TIME("time", valTime);
  GP.BREAK();
  GP.LABEL("Выбор температуры:", "t4");
  GP.SLIDER("manualTemp", dryer.targetTemp, 20, 80, 1);
  GP.BREAK();
  GP.HR();
  GP.LABEL("Температура тэна:", "t3");
  GP.LABEL_BLOCK("sensor.tempNTC", "temp");
  GP.BREAK();
  GP.LABEL("PWM:", "t3");
  GP.LABEL_BLOCK("pwm.heater", "heater");
  GP.LABEL("Лимит:", "t3");
  GP.LABEL_BLOCK("heater.setpoint", "heaterLimit");
  GP.BREAK();
  GP.HR();
  GP.LABEL("Температура:", "t3");
  GP.LABEL_BLOCK("sensor.temp", "tempBME");
  GP.BREAK();
  GP.LABEL("Влажность:", "t3");
  GP.LABEL_BLOCK("sensor.hum", "humBME");
  GP.BREAK();
  GP.PLOT_STOCK_DARK<2, PLOT_SIZE>("plot", names, dates, arr);
  GP.SYSTEM_INFO("0.1");
  GP.BUILD_END();
}
void action()
{
  if (ui.click())
  {
    if (ui.click("power"))
    {
      ui.copyBool("power", dryer.powerOn);
      if (dryer.powerOn)
      {
        dryer.start();
      }
      else
      {
        dryer.stop();
      }
    }
    if (ui.click("manualTemp"))
    {
      ui.copyInt("manualTemp", dryer.targetTemp);
    }
    if (ui.click("time"))
    {
      ui.copyTime("time", valTime);
      dryer.tmr.setTime((valTime.hour * 3600 + valTime.minute * 60 + valTime.second) * 1000);
    }

    if (ui.click("mode"))
    {
      dryer.selectMode(Temp, int(ui.getSelected("mode")));
    }
    if (ui.click("type"))
    {
      dryer.selectType(int(ui.getSelected("type")));
      getTime();
    }
  }
  // было обновление
  if (ui.update())
  {
    if (dryer.powerOn)
    {
      ui.updateTime("time", valTime);
      ui.updateInt("manualTemp", dryer.targetTemp);
    }
    ui.updateBool("power", dryer.powerOn);
    ui.updateFloat("temp", dryer.sensor.tempNTC);
    ui.updateFloat("tempBME", dryer.sensor.temp);
    ui.updateFloat("humBME", dryer.sensor.hum);
    ui.updateInt("mod", dryer.modeControl);
    ui.updateInt("type", dryer.typeFilament);
    ui.updateInt("heater", dryer.pwm.heater);
    ui.updateInt("heaterLimit", heater.setpoint);
  }
}

void setup()
{

  dryer.addFilament("PLA", 53, 120);
  dryer.addFilament("PETG", 65, 240);
  dryer.addFilament("TPU", 50, 180);
  dryer.selectType();
  Wire.begin();
  bme.begin();
  pinMode(thermistorPin, INPUT);
  connectToWifi();
  OTA();
  ntp.begin();
  ntp.updateNow();
  delay(100);
  ntp.updateNow();
  getBME();
  dates[PLOT_SIZE - 1] = GPunix(ntp.year(), ntp.month(), ntp.day(), ntp.hour(), ntp.minute(), ntp.second(), 8);
  GPaddInt(int(dryer.sensor.temp), arr[0], PLOT_SIZE);
  GPaddInt(int(dryer.sensor.hum), arr[1], PLOT_SIZE);
  GPaddUnixS(10, dates, PLOT_SIZE);

  ledcSetup(0, 1000, 8);
  ledcSetup(1, 500, 8);
  ledcAttachPin(19, 0);
  ledcAttachPin(18, 1);

  ledcWrite(0, 0);
  ledcWrite(1, 0);

  // подключаем конструктор и запускаем
  ui.attachBuild(build);
  ui.attach(action);
  ui.start("dryer");

  Temp.setDirection(NORMAL);           // направление регулирования (NORMAL/REVERSE).
  Temp.setLimits(0, dryer.maxTempTen); // пределы температуры тэна
  Temp.setpoint = 60;                  // сообщаем регулятору температуру, которую он должен поддерживать

  heater.setDirection(NORMAL);             // направление регулирования (NORMAL/REVERSE).
  heater.setLimits(0, 255);                // пределы (ставим для 8 битного ШИМ).
  heater.setpoint = Temp.getResultTimer(); // сообщаем регулятору температуру, которую он должен поддерживать
}
void getBME()
{
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(dryer.sensor.pres, dryer.sensor.temp, dryer.sensor.hum, tempUnit, presUnit);
}

void getSensor()
{
  static uint32_t tmr;
  if (millis() - tmr >= 500)
  {
    getBME();
    dryer.sensor.tempNTC = GetTempC();
    tmr = millis();
  }
}
void control()
{
  dryer.tick(); // Обновляем таймер сушки

  if (Temp.setpoint != dryer.targetTemp) // Проверяем, изменилась ли целевая температура корпуса
  {
    Temp.setpoint = dryer.targetTemp; // Если изменилась, устанавливаем новую цель для ПИД-регулятора корпуса
  }

  if (dryer.powerOn) // Если сушилка включена
  {
    Temp.input = dryer.sensor.temp;       // Устанавливаем текущую температуру корпуса в качестве входного значения ПИД-регулятора
    heater.setpoint = Temp.getResultTimer(); // Определяем целевое значение для нагревателя с учетом температуры корпуса

    heater.input = dryer.sensor.tempNTC;        // Передаем текущую температуру нагревателя
    dryer.pwm.heater = heater.getResultTimer(); // Рассчитываем значение ШИМ для управления нагревателем
    ledcWrite(1, dryer.pwm.heater);          // Устанавливаем значение ШИМ на вывод, управляющий нагревателем
  }
  else // Если сушилка выключена
  {
    dryer.pwm.heater = 0;           // Отключаем нагреватель
    ledcWrite(1, dryer.pwm.heater); // Устанавливаем 0 на вывод нагревателя
  }

  if (dryer.sensor.tempNTC > dryer.maxNightTempTen + 10) // Если температура нагревателя превышает допустимую на 10 градусов
  {
    ledcWrite(0, dryer.pwm.cooler); // Включаем охлаждающий вентилятор
  }
  else
  {
    ledcWrite(0, 0); // Иначе выключаем вентилятор
  }
}
void REconn()
{
  if (!dryer.isWIFIconn)
  {

    static uint32_t Wifitimer;
    if (millis() - Wifitimer >= 5* 60 * 1000)
    {
      WiFi.disconnect();
      WiFi.reconnect();
      Wifitimer = millis();
    }
  }
}
void loop()
{
  REconn();
  ArduinoOTA.handle(); // Обработка OTA, если включен режим обновления
  if (!dryer.isOTA)    // Если система не находится в режиме OTA
  {
    getSensor(); // Обновляем показания датчиков
    control();   // Выполняем цикл управления сушилкой

    static uint32_t tmr2;
    if (millis() - tmr2 >= 1000) // Каждую секунду обновляем время
    {
      getTime();       // Получаем текущее время сушки
      tmr2 = millis(); // Сохраняем текущее время
    }

    static uint32_t tmr3;
    if (millis() - tmr3 >= (dryer.updateGraphics * 1000)) // Обновляем графику через заданные интервалы
    {
      GPaddInt(int(dryer.sensor.temp), arr[0], PLOT_SIZE);       // Добавляем значения температуры в график
      GPaddInt(int(dryer.sensor.hum), arr[1], PLOT_SIZE);        // Добавляем значения влажности в график
      GPaddUnixS(int(millis() - tmr3) / 1000, dates, PLOT_SIZE); // Обновляем временные метки на графике
      tmr3 = millis();                                           // Сохраняем текущее время
    }
    ui.tick(); // Обновляем интерфейс
  }
}
void getTime()
{
  uint32_t time = dryer.tmr.timeLeft() / 1000;             // Получаем оставшееся время сушки в секундах
  valTime.set(time / 3600, (time % 3600) / 60, time % 60); // Преобразуем время в часы, минуты и секунды и сохраняем в объект времени
}
void OTA()
{
  ArduinoOTA
      .onStart([]()
               { dryer.isOTA = true; }) // При запуске OTA указываем, что система переходит в режим обновления
      .onEnd([]()
             { dryer.isOTA = false; })                              // По завершении OTA обновления система выходит из режима обновления
      .onProgress([](unsigned int progress, unsigned int total) {}) // Лямбда-функция для отображения прогресса обновления
      .onError([](ota_error_t error) {});                           // Лямбда-функция для обработки ошибок OTA
  ArduinoOTA.begin();                                               // Инициализируем OTA
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    dryer.isWIFIconn = true;
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    dryer.isWIFIconn = false;
    break;
  }
}
void connectToWifi()
{
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);                  // Устанавливаем режим Wi-Fi в режим станции (клиента)
  WiFi.begin(ssid, password);           // Подключаемся к сети Wi-Fi с заданным SSID и паролем
  while (WiFi.status() != WL_CONNECTED) // Ожидаем подключения
  {
    delay(500); // Ждем 500 мс между проверками статуса подключения
  }
}
