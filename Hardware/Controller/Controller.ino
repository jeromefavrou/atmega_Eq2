#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#include "PID.hpp" //https://github.com/jeromefavrou/PID
#include "Stepper.hpp" //https://github.com/jeromefavrou/Stepper

#ifndef nullptr
  #define nullptr 0x00
#endif

#define  VERBOSE_ARDUINO_PROG float(.01f)
#define  VERBOSE_SERIAL_TRAME float(.01f)
#define  ID_MASTER byte(0x01)
#define  ID_SLAVE byte(0x02)
#define  REGULATOR_PERIODE int(200)
#define  MOTOR_STEP_PER_REV int(48)
#define  REDUCTOR_FACTOR uint32_t(120.0*141.0*(140.0/27.0))
#define  MIN_FREQ int(0)
#define  MAX_FREQ int(250)

uint8_t const Ai_size{4};
uint8_t const Do_size{3};
uint8_t const Ao_size{4};
uint8_t const AM2320_size{2};
uint8_t const Step_ctrl_size{5};
uint8_t const Rdy_LED_pin{13};
uint8_t const Thread_inst_size{Ai_size};
uint8_t const MFPI_inst_size{Ai_size};
uint8_t const Stepper_ctr_size{2};

byte const AM2320_address{0x5C} ;

struct Pin{uint8_t const number;float value;};

struct Pin AM2320[AM2320_size]{{A4,.0},{A5,.0}};
struct Pin Ai[Ai_size]{{A0,.0},{A1,.0},{A2,.0},{A3,.0}};
struct Pin Do[Do_size]{{4,LOW},{12,LOW},{Rdy_LED_pin,HIGH}};
struct Pin Ao[Ao_size]{{3,LOW},{9,LOW},{10,LOW},{11,LOW}};//3,9,10,11 pwm


short const Stepper_pin_A[Stepper_ctr_size]{8,6};
short const Stepper_pin_B[Stepper_ctr_size]{7,5};

float Ai_offset[Ai_size]{2.65,2.0,.0,2.5};
float AM2320_offset[AM2320_size]{.0,.0};

unsigned long Thread_inst[Thread_inst_size]{0};
unsigned long Ready_inst{0};
unsigned long FPS{0};

float Ai_order{32};

Adafruit_AM2320 am2320 = Adafruit_AM2320();

Stepper BM_stepper(MOTOR_STEP_PER_REV,REDUCTOR_FACTOR);
PID Reg_Ai[Ai_size];


byte num_error;

// simple static array
class VCHAR
{
  public:
      VCHAR(unsigned int const & n):_data(new byte[n]),_size(n){}
      ~VCHAR(void)
      {
        this->free();
      }
      byte & operator[](unsigned int const & n){return this->_data[n];}
      unsigned int const size(void) const {return this->_size;}
      byte * data(void){return this->_data;}
      void free(void)
      {
        if(this->_data!=nullptr)
        {
          for(auto i=0;i>this->_size;i++)
            this->_data[i]=0;
          
          delete[] this->_data;
        }
      }
      
  private:

  byte *_data;
  unsigned int _size;
};

template<typename T> VCHAR  cast_to_vchar(T const & _type)
{

    VCHAR tps(sizeof(T));

    long fl=*((long *) &_type);

    for(auto i=0u;i<sizeof(T);i++)
        tps[i]= fl >> 8*(sizeof(T)-1-i) & 0xFF ;

    return tps;
}

template<typename T> T cast_to_type(VCHAR & _data)
{
    if(_data.size()!=sizeof(T)){return 0;}
      
      
    T tps(0);

    long fl(0);

    for(auto i=0;i<sizeof(T);i++)
        fl = fl << 8 | _data[i];

    tps  = * ( T * ) &fl;

    return tps;
}

template<uint8_t N> inline void update_Ai(struct Pin lst[N])
{
  for(auto i=0u;i<N;i++)
    lst[i].value=analogRead(lst[i].number);
}

template<uint8_t N> inline void calibrate_termistor(struct Pin lst[N])
{
  float tempK(0);
  for(auto i=0u;i<N;i++)
  {
    tempK = log(10000.0 * ((1024.0 / lst[i].value - 1)));
    tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );
    lst[i].value = tempK - 273.15;
  }
}

template<uint8_t N> inline void offset(struct Pin lst[N],float offset[N])
{
  for(auto i=0u;i<N;i++)
    lst[i].value += offset[i];
}

template<uint8_t N> inline void update_Do(struct Pin lst[N])
{
  for(auto i=0u;i<N;i++)
    digitalWrite(lst[i].number,lst[i].value);
}

template<uint8_t N> void update_Ao(struct Pin lst[N])
{
  for(auto i=0u;i<N;i++)
    analogWrite(lst[i].number,lst[i].value);
}

template<uint8_t N,uint8_t D> void initializer_list(struct Pin lst[N])
{
  for(auto i=0u;i<N;i++)
    pinMode(lst[i].number,D);

  update_Do<N>(lst);
}

void AM2320_Read(struct Pin lst[AM2320_size])
{
    AM2320[0].value=am2320.readTemperature();
    AM2320[1].value=am2320.readHumidity();
}


template<int periode> void thread_regulator(unsigned long & time_inst, PID& _regulator ,float & do_value)
{
  if(millis()-time_inst>=periode)
  { 
    time_inst=millis();
    update_Ai<Ai_size>(Ai); //lecture des analoges
    calibrate_termistor<Ai_size>(Ai); //calibrage
    offset<Ai_size>(Ai,Ai_offset); //offset
  
    do_value=_regulator.corrector(-255,0)*-1.0;

    update_Do<Do_size>(Do); //ecriture des sorties digital general
  } 
}

template<typename T> void send_to_uart(byte const & addr,T const& var)
{
    Serial.write(addr);
    Serial.write(sizeof(T));
    Serial.write(cast_to_vchar<T>(var).data(),sizeof(T));
}

long availableMemory() 
{
  long size = 2048;
  byte *buf;

  while ((buf = (byte *) malloc(--size)) == NULL)
    ;

  free(buf);

  return size;
}

void setup()
{
  initializer_list<Do_size,OUTPUT>(Do); //initialisation des sorties digital genaral
  
  update_Ao<Ao_size>(Ao);//ecriture des sortie analogique
  update_Do<Do_size>(Do); //ecriture des sorties digital general

  for(auto i=0;i<Ai_size;i++)
  {
    Reg_Ai[i].lock_sensor(Ai[i].value);
    Reg_Ai[i].lock_order(Ai_order);
    Reg_Ai[i].parameter(float(REGULATOR_PERIODE)/1000.0,10,0.5,1);
    Reg_Ai[i].direction(-1);
  }
 
  Serial.begin(9600);//uart

  while (!Serial) {
    delayMicroseconds(2000);
  }

  am2320.begin();

  for(auto i=0;i<Thread_inst_size;i++)
    Thread_inst[i]=millis();

  Ready_inst=millis();
  FPS=millis();

  num_error=0x00;

  BM_stepper.max_freq(MAX_FREQ);
  BM_stepper.min_freq(MIN_FREQ);
  BM_stepper.speed_rpm(1.0/(60.0*23.933));
  BM_stepper.init<Stepper_ctr_size,Stepper_ctr_size>(Stepper_pin_A,Stepper_pin_B,2);
  
  BM_stepper.direction(Stepper::DIRECTION::FORWARD);
  BM_stepper.move_steps(100);
  
  delay(500);
  
  BM_stepper.direction(Stepper::DIRECTION::BACKWARD);
  BM_stepper.move_steps(100);
  
  delay(50);
  
  BM_stepper.stop();
}
inline void free_uart(void)
{
  while(Serial.available () > 0){ Serial.read();}    
}

template<typename T> T read_var_uart(void)
{
  VCHAR data(sizeof(T));
  
  for(auto i=0; i< sizeof(T);i++)
    data[i]=Serial.read();
    
  return cast_to_type<T>(data);
}

void read_uart(void)
{ 
  long av=Serial.available();
  
  if(Serial.available() < 10){ free_uart(); num_error=0x01; return ;}
  else { if(Serial.read()!=0x00){ free_uart(); num_error=0x02; return ;} }// the first byte is 0x00 it's used to control the integrity ( SOT)

  //read header
  
  if(read_var_uart<float>() > VERBOSE_SERIAL_TRAME){ free_uart(); num_error=0x03; return ;}//version
  if(read_var_uart<byte>() != ID_MASTER){ free_uart(); num_error=0x04; return ;}//src
  if(read_var_uart<byte>() != ID_SLAVE){ free_uart(); num_error=0x05; return ;}//dest
  if(read_var_uart<long>() != (av-2)){ free_uart(); num_error=0x010+av; return ;}// size
  
  //read data
  byte _addr,_size;
  
  bool send_data;

  while(Serial.available() > 1) // the last byte is 0x04 it's ignored but can be used to control the integrity (EOT)
  {
    _addr=Serial.read();
    _size=Serial.read();

    if(_size==sizeof(byte))
    {
      byte b_value=read_var_uart<byte>();

      if((int)b_value==0x00 && (int)_addr==0x01)
      {
        BM_stepper.stop();
      }
      else if((int)b_value==0x01 && (int)_addr==0x01)
      {
        BM_stepper.start();
      }
      else if( (int)_addr==0x02 )
      {
        if(BM_stepper.stat())
        {
          BM_stepper.stop();
          delay(100);
          
          BM_stepper.direction((int)b_value==0x00 ? Stepper::DIRECTION::FORWARD : Stepper::DIRECTION::BACKWARD);
          
          BM_stepper.start();
        }
        else
          BM_stepper.direction((int)b_value==0x00 ? Stepper::DIRECTION::FORWARD : Stepper::DIRECTION::BACKWARD);
        
      }
      else if( (int)_addr==0x7F )
      {
        send_data= (int)b_value == 0x01 ? true : false;
      }
      else
      {
        for(auto i=0u; i< _size ; i++)
          Serial.read();
      }
    }
    else
    {
      for(auto i=0u; i< _size ; i++)
        Serial.read();
    }
    
  }

  free_uart();

  if(!send_data)
    return;

  send_data=false;
    
  AM2320_Read(AM2320);
  
  Serial.flush();

  ///start byte SOT
  Serial.write((byte)0x00);  

  /// get verbose serial tram
  Serial.write(cast_to_vchar<float>(VERBOSE_SERIAL_TRAME).data(),sizeof(float));

  //get id src
  Serial.write(ID_SLAVE); 

  //get dest
  Serial.write(ID_MASTER); 

  /// get size data
  Serial.write(cast_to_vchar<long>(61l).data(),sizeof(long)); 

  /// get version arduino programme
  send_to_uart<float>(0x01,VERBOSE_ARDUINO_PROG);
  
  /// num error byte
  send_to_uart<byte>(0x02,num_error);

  /// get avaible memory
  send_to_uart<long>(0x03,availableMemory());

  /// get temperate and humidity of module AM2320
  send_to_uart<float>(0x04,AM2320[0].value);
  send_to_uart<float>(0x05,AM2320[1].value);

  /// get temperatures externals
  for(auto i=0l;i< Ai_size ; i++)
    send_to_uart<float>(0x06+i,Ai[i].value);
  
  /// end byte EOT
  Serial.write((byte)0x04); 
}

void loop()
{
  if(millis()-Ready_inst>=1000)
  {
      read_uart();

      Ready_inst=millis();
  
      Do[2].value=!Do[2].value;
      num_error=0x00;
  }
  
  for(auto i=0;i<Ao_size;i++)
    thread_regulator<REGULATOR_PERIODE>(Thread_inst[i], Reg_Ai[i],Ao[i].value);

  BM_stepper.move_async(Stepper::MOVE_TYPE::HIGH_TORQUE);

}
