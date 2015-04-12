#include <OneWire.h>
#include <LiquidCrystal.h>

OneWire  ds(3);  // термодатчик подключается к пину 8
LiquidCrystal lcd(13, 12, 11, 10, 9, 8); // Указываем, к каким пинам Arduino подключены выводы дисплея: RS, E, DB4, DB5, DB6, DB7
const int nagrev = 5; // пин включения нагрева, поддерживающий ШИМ
const int button1_pin = 6;
const int button2_pin = 7; // пины кнопок установки температуры
int button1_state = 0;
int button2_state = 0;

int temp_treb=40; // требуемая температура

float Kp_baz=3; // базовый коэффициент пропорциональности
float Kp_ispr; // исправленный коэффициент пропорциональности, зависящий от температуры
float Kp_t_koef=1/30; // коэффициент для зависимости коэффициента пропорциональности от температуры

float temp_pov_baz=0.2; // базовая температура повышения относительно требуемой, чтобы держаться на требуемой
float temp_pov_ispr; // исправленная температура повышения, зависящая от требуемой температуры
float temp_pov_koef=1/15; // коэффициент для зависимости температуры повышения от требуемой температуры

float procent; // процент мощности, подаваемой на нагреватель
int ShIM; // коэффициент пересчета процентов в 0...255



void setup(void) {
  lcd.begin(16, 2);
  Serial.begin(9600);
  pinMode(nagrev, OUTPUT);
  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
}


void loop(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float temp_fakt; // фактическая температура
  
  if ( !ds.search(addr)) {
    ds.reset_search();
    return;
  }
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // convert the data to actual temperature
  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  temp_fakt = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(temp_fakt);
  Serial.println(" Celsius, ");
  Serial.print("Treb_temp= ");
  Serial.println(temp_treb);
  
  //вывод текущей температуры на ЖК экран
  lcd.setCursor(0, 0);
  lcd.print("Temp=");
  lcd.print(temp_fakt);
  
  
  // проверка температуры и включение нагрева
  Kp_ispr=Kp_baz*(1 - Kp_t_koef * (temp_treb-30)); // чем выше требуемая температура, тем меньше исправленный коэффициент пропорциональности
  
  temp_pov_ispr=temp_pov_baz*(1 + temp_pov_koef * (temp_treb-30)); // чем выше требуемая температура, тем больше исправленная температура повышения
  
  procent=(temp_treb - temp_fakt + temp_pov_ispr) / Kp_ispr * 100; // процент мощности, подаваемой на нагреватель
  if (procent > 100) {procent=100;}
  if (procent < 0) {procent=0;}
  ShIM = map(procent, 0, 100, 0, 255); 
  analogWrite(nagrev, ShIM);  
  
  
  // проверка кнопок
  button1_state = digitalRead(button1_pin);
  button2_state = digitalRead(button2_pin);
  if (button1_state == LOW) {
    temp_treb=temp_treb+1;
    delay(100);
  }
    else if (button2_state == LOW) {
      temp_treb=temp_treb-1;
      delay(100);
    }
      else {delay(100);}
      
  //вывод нужной температуры на ЖК экран
  lcd.setCursor(0, 1);
  lcd.print("Treb_temp=");
  lcd.print(temp_treb);
}
