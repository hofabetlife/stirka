//error=1   превышение оборотов
//error=2   нет вращения
//error=3   показания датчика температуры за пределами допустимого
//error=4   за 200 секунд не сработал датчик заполнения бака
//error=5   за 180 секунд не слилась вода
//error=6   проблемы с блокировкой двери

//error=9  переполнение бака
//--------------------------------------------------------------

// Порты
#define rele_motor_1 PD5  // реле реверса 1
#define rele_motor_2 PD7  // реле реверса 2
#define temp_pin A1       // вход от датчика температуры
#define sliv_1_pin 11     // клапан залива 1
#define sliv_2_pin 12     // клапан залива 2
#define PWM_motor PD6     // двигатель ШИМ (семистор)
#define drain_pump 9      // сливной насос
#define ten_pin PD3       // ТЭН
#define lock_pin 13       // питание замка
#define full_tank A2      // бак заполнен
#define lock_closed A3    // вход подтверждение закрытия замка
//#define tank_overflow     // переполнение бака
// D8 - прерывание по таймеру (таходатчик)
#define krutila_pin A0  // переключение скорости при отжими (индекатор выбора температуры)

#define btn_start_pin A6  // старт стоп
#define btn1_pin A5       // режим
#define btn2_pin A4       // режим

#define led PD4  //лампа индикация

int btnArray[3] = { 0, 0, 0 };  //состояние кнопок


int state_zamok = 0;    // значение ацп замка должно быть 2.2 В
bool is_start = false;  //был ли запуск?
int maxTraska = 800;    // максимальные обороты при отжиманиях



//---------------для вращения двигателя--------------------------------
//Фазовай регулирова
int32_t dimtime = 18000;
volatile unsigned long time;  // время в микросекундах срабатывания датчика нуля
unsigned long tims;           // переменная показаний времени



//ПИД
int32_t prOb;          //предвар реальн обороты
unsigned long rOb;     // реальные обороты
unsigned int int_tic;  //переменные для подсчёта времени между импульсами.
unsigned long tic;
float dt = 0.1;
unsigned long last_time = 0;
int32_t P;
int32_t I;
int32_t D;
int32_t P_pred;


int val = 0;

long totgm = 0;
long e2;


String StringtoSend;
int etap;
int oetap = 0;        //для этапов отжима
int pusk = 0;         // 1 - цикл запущен, 0 - нет  (при подключенной плате индикации pusk = 0)
int stirka = 0;       // статус стирки
int poloskanije = 0;  // статус полоскания
int sliv = 0;         // статус слива
int otgim = 0;        // статус отжима
int start = 0;


int timeHour = 0;         // время всего цикла стирки (часы) (при подключенной плате индикации равно 0)
int timeMin = 0;          // время всего цикла стирки (минуты) (при подключенной плате индикации равно 0)
int polosk = 2;           // колличество полосканий (при подключенной плате индикации равно 0)
int temperatureUst = 20;  // переменная установленной температуры - при подключенной плате индикации считывается с нее


long interval, ms8;
long ms, mszap, mszaliv, mssls, mssliv, mstime, timerotation;
bool zap = 0;
bool startzaliv = 0;
bool sl = 0;
bool timerun = 0;  //разрешить отсчёт времени

bool pu = 0;
bool otgimsmal = 0;  //короткий отжим
bool slivst = 0;
bool rotation, rotation1;
bool zah;


int t = 0;
int temperature = 20;
int error = 0;  // код ошибки
int tst, tpol, kolotg;
int tpol1 = 0;
int totg = 0;
int izamok = 0;
int zamok;
int tst1 = 0;

int MaxObt = 600;        //обороты при стирке  7200
int MaxObtPolosk = 650;  //обороты при полоскание 9000


uint16_t bak = 0;  //состояние бака
//----------------------------------------------------------------------------------------



void setup() {
  Serial.begin(9600);
  //pinMode(temp_pin, INPUT);       // вход от датчика температуры
  pinMode(sliv_1_pin, OUTPUT);    // клапан залива 1
  pinMode(sliv_2_pin, OUTPUT);    // клапан залива 2
  pinMode(ten_pin, OUTPUT);       // ТЭН
  pinMode(rele_motor_1, OUTPUT);  // реле реверса 1
  pinMode(rele_motor_2, OUTPUT);  // реле реверса 2

  pinMode(lock_pin, OUTPUT);   // питание замка
  pinMode(PWM_motor, OUTPUT);  // двигатель ШИМ

  pinMode(led, OUTPUT);

  pinMode(8, INPUT);            // прерывание по таймеру
  pinMode(lock_closed, INPUT);  // вход подтверждение закрытия замка
  pinMode(drain_pump, OUTPUT);  // сливной насос

  attachInterrupt(0, zero_crosss_int, RISING);  // прерывание по пину 2 до этого RISING

  //настройка 16 бит таймера-счётчика 1
  TCCR1B = 0;
  TCCR1A = 0;
  TCNT1 = 0;
  TIMSK1 = (1 << ICIE1) | (1 << TOIE1);                              //создавать прерывание от сигнала на пине ICP1
  TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS10) | (1 << CS11);  //div 1*/

  obnulenie();

  digitalWrite(sliv_1_pin, HIGH);
  digitalWrite(sliv_1_pin, HIGH);
  digitalWrite(lock_pin, HIGH);
  digitalWrite(ten_pin, LOW);
}

// время цикла стирки = время полученное с платы индикации - (3мин* кол-во полосканий) - 8 мин отжим - (3 мин * кол - во промежуточных отжимов)
// кол-во промежуточных отжимов = стирка + (кол-во полосканий -1)

//-------------------ПИД РЕГУЛЯТОР-----------------------------
void PID_motor(int speed, float Kp = 5, float ki = 2, float kd = 0.3) {
  //speed = 2000;
  if (millis() - last_time > (dt * 1000)) {
    last_time = millis();
    P = speed - prOb;
    I = I + P * dt;
    D = (P - P_pred) / dt;
    P_pred = P;
    dimtime = Kp * P + ki * I + kd * D;
    if (dimtime > 18000) dimtime = 18000;
    if (dimtime < 0) dimtime = 0;
  }
}
//-------------------------------------------------------------

int val_temp = 0;
bool height = true;  //наростает
//-------------------ОСНОВНОЙ ЦИКЛ-----------------------------
void loop() {
  //плавный рост
  if (otgim == 0) {
    if (val_temp > 0 && height) {
      val = val + 2;
      if (val > val_temp) val = val_temp;
    } else if (height == false && val_temp == 0) {
      val = val - 10;
      if (val < val_temp) val = val_temp;
    }
  }

  time = micros();                       // считываем время, прошедшее с момента запуска программы
  if (time >= (tims + 18000 - dimtime))  //18000-dimtime
  {                                      //если время больше или равно времени срабатывания нуля + время задержки
    digitalWrite(PWM_motor, HIGH);       // открываем симистор
    delayMicroseconds(10);               // задержка 10 микросекунд (для 60Hz = 8.33)
    digitalWrite(PWM_motor, LOW);        // выключаем сигнал на симистор.
  }

  bak = 0.85 * bak + 0.15 * analogRead(full_tank);


  PID_motor(val);

  check_button();  // проверка кнопОчек

  temperature_sensor_polling();  // опрос датчика температуры

  timing();  // отсчет времени

  cycle_started();  // цикл запущен


  if (pusk == 0 && pu == 1) {
    etap = 5;
    stopr();
  }



  //-----------------------замок закрылся---------------------------------------
  //state_zamok = analogRead(lock_closed);
  state_zamok = 460;
  if (pusk == 1 && (state_zamok > 300 && state_zamok < 712)) {

    //-----------------------------------------этап стирки------------------------

    if (stirka == 1) {

      zapolnenie();  // залив воды


      //----------------контроль температуры--------------------------------
      if (bak < 300)
        if (temperature < temperatureUst - 2) digitalWrite(ten_pin, 1);
      if (bak > 700) digitalWrite(ten_pin, 0);
      if (temperature >= temperatureUst) digitalWrite(ten_pin, 0);
      //---------------------------------------------------------------------



      //----------вращение барабана во время стирки----------------------------------------------------------------------------------

      if (rotation == 0) {
        digitalWrite(rele_motor_1, 0);
        digitalWrite(rele_motor_2, 0);
      }

      if (rotation == 1 && rotation1 == 0) {
        timerotation = millis();
        rotation1 = 1;
      }

      if (millis() - timerotation > 2000 && millis() - timerotation <= 2500 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_1, 0);  // установка направления вращения
      if (millis() - timerotation > 2500 && millis() - timerotation <= 4000 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_2, 1);

      if (millis() - timerotation > 4000 && millis() - timerotation <= 24000 && rotation == 1) {
        //val = 0;  //подать ШИМ на симмистр
        val_temp = MaxObt;
        height = true;
        //PID_motor(val);
      }
      if (millis() - timerotation > 24000 && millis() - timerotation <= 26000 && rotation == 1) {
        //val = 0;  //снять ШИМ с симмистора
        val_temp = 0;
        height = false;
        //delay(2000);
        //PID_motor(val);
      }


      if (millis() - timerotation > 26000 && millis() - timerotation <= 27000 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_2, 0);  // установка направления вращения
      if (millis() - timerotation > 27000 && millis() - timerotation <= 27500 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_1, 1);

      if (millis() - timerotation > 28000 && millis() - timerotation <= 48000 && rotation == 1) {
        //val = 0;  //подать ШИМ на симмистр
        val_temp = MaxObt;
        height = true;
        //PID_motor(val);
      }
      if (millis() - timerotation > 48000 && millis() - timerotation <= 49000 && rotation == 1) {
        //val = 0;  //снять ШИМ с симмистора
        val_temp = 0;
        height = false;
        //delay(2000);
        // PID_motor(val);
      }
      if (millis() - timerotation > 49000 && rotation == 1) rotation1 = 0;

      //-----------------------------------------------------------------------------------------------------------------------------




      if (tst <= 0) {  // время стирки истекло
        tst = 0;
        tpol1 = 3;
        totg = 3;
        val_temp = 0;
        val = 0;
        height = false;
        digitalWrite(ten_pin, 0);
        digitalWrite(rele_motor_1, 0);
        digitalWrite(rele_motor_2, 0);
        stirka = 0;
        rotation = 0;
        otgimsmal = 1;
        slivst = 0;
        sl = 0;
        zah = 0;
        startzaliv = 0;
        timerun = 0;
      }
    }
    //------------------------------------------------------------------------------------------------------------------------------







    //------------------------ процесс полоскания----------------------------------------------------------------------------------

    if (poloskanije == 1 && otgimsmal == 0 && stirka == 0 && otgim == 0 && sliv == 0) {

      zapolnenie();  //---залив воды----



      //----------вращение барабана во время полоскания-------------

      if (rotation == 0) {
        digitalWrite(rele_motor_1, 0);
        digitalWrite(rele_motor_2, 0);
      }

      if (rotation == 1 && rotation1 == 0) {
        timerotation = millis();
        rotation1 = 1;
      }

      if (millis() - timerotation > 2000 && millis() - timerotation <= 2500 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_1, 0);  // установка направления вращения
      if (millis() - timerotation > 2500 && millis() - timerotation <= 4000 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_2, 1);

      if (millis() - timerotation > 4000 && millis() - timerotation <= 24000 && rotation == 1) {
        //val = 0;
        val_temp = MaxObtPolosk;
        height = true;
        //PID_motor(val);
      }
      if (millis() - timerotation > 24000 && millis() - timerotation <= 26000 && rotation == 1) {
        //val = 0;
        val_temp = 0;
        height = false;
        //delay(2000);
        //PID_motor(val);
      }


      if (millis() - timerotation > 26000 && millis() - timerotation <= 27000 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_2, 0);  // установка направления вращения
      if (millis() - timerotation > 27000 && millis() - timerotation <= 27500 && rotation == 1 && prOb < 100) digitalWrite(rele_motor_1, 1);

      if (millis() - timerotation > 28000 && millis() - timerotation <= 48000 && rotation == 1) {
        //val = 0;
        val_temp = MaxObtPolosk;
        height = true;
        //PID_motor(val);
      }
      if (millis() - timerotation > 48000 && millis() - timerotation <= 49000 && rotation == 1) {
        //val = 0;  //снять ШИМ с симмистора
        val_temp = 0;
        height = false;
        //delay(2000);
        //PID_motor(val);
      }
      if (millis() - timerotation > 49000 && rotation == 1) rotation1 = 0;


      //--------------------------------------------------------




      if (tpol1 <= 0) {  // время полоскания истекло
        polosk = polosk - 1;
        val_temp = 0;
        val = 0;
        height = false;
        rotation = 0;
        digitalWrite(rele_motor_1, 0);
        digitalWrite(rele_motor_2, 0);

        if (polosk <= 0) {  //перейти на основной отжим
          poloskanije = 0;
          tpol1 = 0;
          startzaliv = 0;
          slivst = 0;
          sl = 0;
          timerun = 0;
          totg = 1;  //время первого этапа отжима
          zah = 0;
          timeMin = 7;
          timeHour = 0;
          oetap = 0;
          otgim = 1;
        }
        if (polosk > 0) {  //перейти на короткий отжим
          poloskanije = 1;
          val_temp = 0;
          val = 0;
          height = false;
          rotation = 0;
          tpol1 = 3;
          totg = 3;
          startzaliv = 0;
          otgimsmal = 1;
          slivst = 0;
          sl = 0;
          timerun = 0;
          zah = 0;
        }
      }
    }
    //-------------------------------------- полоскание --------------------------------------------------





    //--------------------------программа просто слив воды-------------------------------------------------
    if (sliv == 1) {
      digitalWrite(drain_pump, 1);
      if (bak < 300 && slivst == 0) {
        slivst = 1;
        mssls = millis();
      }
      if (bak > 700 && sl == 0) {
        sl = 1;
        timerotation = millis();
      }
      //if (millis() - mssls > 180000 && bak < 300) error = 5;
      if (millis() - timerotation > 30000 && bak > 700) {
        digitalWrite(drain_pump, 0);
        sliv = 0;
      }
    }
    //-----------------------------------------------------------------------------------------------------------------


    //------------------------- короткий отжим ----------------------------------------

    if (otgimsmal == 1) {


      digitalWrite(drain_pump, 1);
      if (bak < 300 && slivst == 0) {
        slivst = 1;
        mssls = millis();
      }
      if (bak > 700 && sl == 0) {
        timerun = 1;
        sl = 1;
        timerotation = millis();
      }
      //if (millis() - mssls > 180000 && bak > 700) error = 5;

      //--------------------вращение барабана при коротком отжиме---------------------
      if (millis() - timerotation > 20000 && millis() - timerotation <= 22000 && bak > 700) {
        
        digitalWrite(rele_motor_1, 0);  //установить направление вращения
        digitalWrite(rele_motor_2, 1);
        
      }
      if (millis() - timerotation > 22000 && bak > 700 && zah == 0) {
        zah = 1;  // подать ШИМ на двигатель
        totgm = millis();
        val_temp = 50;
        height = true;
      }

      //PID_motor(val);

      if (millis() - totgm > 500 && val > 0 && otgimsmal == 1 && totg > 0) {
        //val = val + 5;
        val_temp = maxTraska;
        height = true;
        //if (val > maxTraska) val = maxTraska;
        totgm = millis();
      }


      if (totg <= 0) {  // время истекло
        val = 0;
        val_temp = 0;
        height = false;
        digitalWrite(rele_motor_1, 0);
        digitalWrite(rele_motor_2, 0);

        if (pulseIn(8, LOW) < 100) {
          otgimsmal = 0;  //------выполнить после полного останова двигателя---------
          zah = 0;
          rotation = 1;
          digitalWrite(drain_pump, 0);
        }
      }
    }
    //--------------------------Конец короткий отжим----------------------------------------------------------
    main_spin();  // основной отжим
  }


  //-----------------------замок закрылся----------------------------------------

  if (error > 0) stopr();
  //if (digitalRead(tank_overflow) == 0) error = 9;  //сработал перелив


  finishing_the_wash();  // цикл завершен
}

//-------------------Режимы----Новое-----------------------------------------
bool kontrol_btn = true;
void check_button() {
  if (analogRead(btn_start_pin) > 600) btnArray[0] = 1;
  else if (analogRead(btn_start_pin) < 200) {
    btnArray[0] = 0;
    pusk = 0;
    timeMin = 0;
    timeHour = 0;
    polosk = 0;
    poloskanije = 0;
    sliv = 0;
    otgim = 0;
    start = 0;
    oetap = 0;
    val = 0;
    val_temp = 0;
    height = false;
    digitalWrite(led, 0);
  }
  if (analogRead(btn1_pin) > 600) btnArray[1] = 1;
  else if (analogRead(btn1_pin) < 200) btnArray[1] = 0;
  if (analogRead(btn2_pin) > 600) btnArray[2] = 1;
  else if (analogRead(btn2_pin) < 200) btnArray[2] = 0;

  if (btnArray[0] == 0) kontrol_btn = true;

  if (pusk == 0 && kontrol_btn) {
    int sum = btnArray[0] * 4 + btnArray[1] * 2 + btnArray[2];


    //Serial.println(sum);
    //is_start = true;
    // код выбора режима
    switch (sum) {
      case 6:  //старт + полосканике
        pusk = 1;
        kontrol_btn = false;
        temperatureUst = 40;
        timeHour = 0;
        timeMin = 20;
        polosk = 2;
        stirka = 1;
        poloskanije = 1;
        sliv = 0;
        otgim = 0;
        maxTraska = turnover_calculation();
        digitalWrite(led, 1);
        break;

      case 5:  //старт + экономия времени
               //--------------слив
        pusk = 1;
        kontrol_btn = false;
        temperatureUst = 10;
        timeHour = 0;
        timeMin = 1;
        polosk = 0;
        stirka = 0;
        poloskanije = 0;
        sliv = 1;
        otgim = 0;
        maxTraska = turnover_calculation();
        digitalWrite(led, 1);
        break;

      case 7:  //долгая стирка
        pusk = 1;
        kontrol_btn = false;
        temperatureUst = 40;
        timeHour = 0;
        timeMin = 48;
        polosk = 2;

        stirka = 1;
        poloskanije = 1;
        sliv = 0;
        otgim = 1;
        maxTraska = turnover_calculation();
        digitalWrite(led, 1);
        break;

      case 4:  //старт + полосканике
        pusk = 1;
        kontrol_btn = false;
        temperatureUst = 0;
        timeHour = 0;
        timeMin = 6;
        polosk = 0;
        stirka = 0;
        poloskanije = 0;
        sliv = 0;
        otgim = 1;
        maxTraska = turnover_calculation();
        digitalWrite(led, 1);
        break;

      default:  //стоп
        pusk = 0;
        timeMin = 0;
        timeHour = 0;
        polosk = 0;
        poloskanije = 0;
        sliv = 0;
        otgim = 0;
        start = 0;
        oetap = 0;
        digitalWrite(led, 0);
        break;
    }
  }
}

//------------------Расчет траски (оборотов отжима)--------ноаое------------
int turnover_calculation() {
  int kruto = analogRead(krutila_pin);
  int mxTrsk = 500;
  if (kruto <= 85) mxTrsk = 300;
  else if (85 < kruto && kruto <= 226) mxTrsk = 400;
  else if (227 < kruto && kruto <= 509) mxTrsk = 500;
  else if (510 < kruto && kruto <= 629) mxTrsk = 600;
  else if (630 < kruto && kruto <= 775) mxTrsk = 700;
  else if (776 < kruto && kruto <= 878) mxTrsk = 800;
  else if (878 < kruto && kruto <= 981) mxTrsk = 900;
  else if (982 < kruto) mxTrsk = 1000;

  return (mxTrsk * 18);  //коэф передачи к барабану 18
}


//------------------- цикл запущен--------------------------------------------
void cycle_started() {
  if (pusk == 1 && error == 0 && pu == 0) {
    tpol1 = 3;
    tpol = polosk * 3;

    kolotg = polosk - 1;
    totg = 1;  // 1мин +2мин+4мин время основного отжима - 3 этапа
    if (stirka == 1) kolotg = kolotg + 1;
    tst = (timeMin + timeHour * 60) - tpol - 8 - 3 * kolotg;  //вычисление времени для этапа стирки
    if (tst < 5) tst = 5;
    zah = 0;





    digitalWrite(lock_pin, LOW);  // подать импульс на замок
    //state_zamok = analogRead(lock_closed);
    state_zamok = 460;
    izamok = 0;
    while ((state_zamok < 300 || state_zamok > 512) && izamok < 50) {
      //state_zamok = analogRead(lock_closed);
      digitalWrite(lock_pin, LOW);
      //delay(1000);
      izamok++;
    }
    //state_zamok = analogRead(lock_closed);
    if (state_zamok > 300 && state_zamok < 712) {
      rotation = 1;  // разрешить вращение двигателя
      mstime = millis();
      izamok = 0;
    }
    pu = 1;  // чтобы сюда больше не заходить
  }
}

//-------------------опрос датчика температуры -------------------------------------------------
void temperature_sensor_polling() {
  if (millis() - ms > 1000) {
    t = analogRead(temp_pin);
    if (t < 50 || t > 990) error = 3;  // датчик температуры за пределами нормы
    temperature = datchik(t);

    ms = millis();
  }
}

//-------------------отсчет времени---------------------------------------------------------------------------
void timing() {
  if (millis() - mstime >= 60000 && timerun == 1) {

    timeMin = timeMin - 1;

    if (stirka == 1) tst = tst - 1;
    if (poloskanije == 1 && stirka == 0 && otgimsmal == 0) tpol1 = tpol1 - 1;
    if (otgimsmal == 1 && sl == 1) totg = totg - 1;
    if (otgim == 1 && sl == 1) totg = totg - 1;



    if (timeMin < 0) {

      timeHour = timeHour - 1;
      if (timeHour < 0) {  //время истекло
        timeMin = 1;
        timeHour = 0;


      } else timeMin = 59;
    }
    if (timeHour == 0 && timeMin == 0) {  //время истекло

      timeMin = 1;
      timeHour = 0;
    }

    mstime = millis();
  }
}

//------------------------- основной отжим -----------------------------------------------------------
void main_spin() {
  if (otgim == 1 && stirka == 0 && sliv == 0 && poloskanije == 0 && otgimsmal == 0) {


    digitalWrite(drain_pump, 1);
    if (bak < 300 && slivst == 0) {
      slivst = 1;
      mssls = millis();
    }
    if (bak > 700 && sl == 0) {
      timerun = 1;
      sl = 1;
      timerotation = millis();
    }
    //if (millis() - mssls > 180000 && bak > 700) error = 5;

    //--------------------вращение барабана при основном отжиме  три этапа---------------------

    if (millis() - timerotation > 20000 && millis() - timerotation <= 22000 && bak > 700) {
      if (oetap == 0) {
        
        digitalWrite(rele_motor_1, 0);
        digitalWrite(rele_motor_2, 1);

      } else {
  
        digitalWrite(rele_motor_1, 1);
        digitalWrite(rele_motor_2, 0);

      }
    }  //установить направление вращения
    if (millis() - timerotation > 22000 && bak > 700 && zah == 0) {
      zah = 1;
      if (oetap == 0) totg = 1;
      val = 200;
      totgm = millis();

      e2 = millis();
    }  // подать ШИМ на двигатель



    if (millis() - totgm > 500 && val > 0 && otgim == 1 && totg > 0) {
      if (val < 500) val = val + 40;
      else val = val + 80;

      if (oetap == 0 && val > MaxObt) val = MaxObt;
      if (oetap == 1 && val > MaxObt) val = MaxObt;
      if (oetap == 2 && val > maxTraska) val = maxTraska;

      totgm = millis();
    }
    //PID_motor(val);


    if (totg <= 0 && oetap == 0) {  // время первого этапа истекло

      val = 0;
      digitalWrite(rele_motor_1, 0);
      digitalWrite(rele_motor_2, 0);

      if (pulseIn(8, LOW) < 100) {
        oetap = 1;  //------выполнить после полного останова двигателя---------
        zah = 0;
        totg = 2;
        timerotation = millis();
      }
    }

    if (totg <= 0 && oetap == 1) {  // время второго этапа истекло

      val = 0;
      digitalWrite(rele_motor_1, 0);
      digitalWrite(rele_motor_2, 0);

      if (pulseIn(8, LOW) < 100) {
        oetap = 2;  //------выполнить после полного останова двигателя---------
        totg = 4;
        zah = 0;
        timerotation = millis();
      }
    }

    if (totg <= 0 && oetap == 2) {  // время третьего этапа истекло

      val = 0;
      digitalWrite(rele_motor_1, 0);
      digitalWrite(rele_motor_2, 0);

      if (pulseIn(8, LOW) < 100) {
        otgim = 0;  //------выполнить после полного останова двигателя---------
        rotation = 0;
        totg = 0;
        digitalWrite(drain_pump, 0);
        digitalWrite(led, 0);
      }
    }
  }
}

//--------------------------------цикл завершен----------------------------------------
void finishing_the_wash() {
  if (stirka == 0 && poloskanije == 0 && otgim == 0 && sliv == 0 && otgimsmal == 0) {

    digitalWrite(lock_pin, 1);  // выключить замок

    //-----------тут обнулить все переменные------------
    pusk = 0;
    val_temp = 0;
    val = 0;
    digitalWrite(sliv_1_pin, 1);
    digitalWrite(sliv_2_pin, 1);
    digitalWrite(ten_pin, 0);
    digitalWrite(rele_motor_1, 0);
    digitalWrite(rele_motor_2, 0);

    obnulenie();
  }
}

void stopr() {
  val = 0;
  val_temp = 0;
  height = false;
  digitalWrite(sliv_1_pin, 1);
  digitalWrite(sliv_2_pin, 1);
  digitalWrite(ten_pin, 0);
  digitalWrite(rele_motor_1, 0);
  digitalWrite(rele_motor_2, 0);
  //digitalWrite(4,0);
  if (error == 9) {  // error == 9 && digitalRead(tank_overflow) == 0
    digitalWrite(drain_pump, 1);
  } else {
    digitalWrite(drain_pump, 0);

    //------открыть замок-------------------
    digitalWrite(lock_pin, 1);
    izamok = 0;
    obnulenie();
  }
}

void obnulenie() {  //обнуление переменных
  rotation = 0;
  timeHour = 0;
  timeMin = 0;
  polosk = 0;
  temperatureUst = 20;
  zap = 0;
  startzaliv = 0;
  sl = 0;
  timerun = 0;
  otgimsmal = 0;
  slivst = 0;
  rotation1 = 0;
  zah = 0;
  oetap = 0;
  t = 0;
  temperature = 20;
  tst = 0;
  tpol = 0;
  kolotg = 0;
  tpol1 = 0;
  totg = 0;
}

void zapolnenie() {
  if (bak > 700 && error == 0) {
    digitalWrite(sliv_2_pin, 0);
    timerun = 0;
    if (stirka == 0 && polosk == 1) digitalWrite(sliv_1_pin, 0);
  }
  if (bak < 300) startzaliv = 0;
  if (bak > 700 && startzaliv == 0) {
    digitalWrite(sliv_2_pin, 0);  // бак не заполнен - включить клапан 2, выключить ТЭН
    mszaliv = millis();
    startzaliv = 1;
    digitalWrite(ten_pin, 0);
  }
  if (bak < 300 && zap == 0) {
    zap = 1;  // бак заполнен - дать добрать воды ещё 10 секунд
    mszap = millis();
    startzaliv = 0;
    timerun = 1;
  }
  if (zap == 1 && millis() - mszap > 10000) {
    digitalWrite(sliv_2_pin, 1);
    digitalWrite(sliv_1_pin, 1);
    zap = 0;
  }
  if (bak > 700 && startzaliv == 1 && millis() - mszaliv > 200000) error = 4;  // за 200 секунд бак не заполнился , уйти в ошибку
}

//--------------------------------------------------------------------------------------------
ISR(TIMER1_CAPT_vect) {                    //прерывание захвата сигнала на входе ICP1
  tic = ((uint32_t)int_tic << 16) | ICR1;  //подсчёт тиков
  delayMicroseconds(1);
  ICR1 = 0;
  int_tic = 0;
  delayMicroseconds(1);
  TCNT1 = 0;
  delayMicroseconds(1);
  if (tic > 15000) tic = 15000;
  prOb = 1400000 / (tic + 1);
  if (prOb > 20000 | prOb < 20) prOb = 2;
  //prOb = prOb * 10;
}  // после каждого срабатывания датчика холл+1


ISR(TIMER1_OVF_vect) {  //прерывание для счёта по переполнению uint
  int_tic++;            //считать переполнения через 65536 тактов
}

// детектор нуля
void zero_crosss_int() {
  if ((time - tims) > 0) tims = time;
  //Serial.println(String(val) + ' ' + String(bak) + ' ' + String(otgimsmal) + ' ' + String(timeMin) + ' ' + String(otgim));
}
//-----------------------------------------------------------------------------------


int datchik(int t) {  //расчет температуры
  int temper;
  temper = (float)(t - 235) / 7.433;
  return temper;
}
