/*
	Name:       Tank.ino
	Created:	03-Jan-21 11:28:49
	Author:     NTNET\ROMANL
*/

#include "SPI.h"
#include "RF24.h"
#include "nRF24L01.h"

#include "DFRobotDFPlayerMini.h"
#include "Adafruit_PWMServoDriver.h"
#define  ENABLE_LOGGER

#include "Constants.h"
//#include "BTS7960.h"
#include "TA6586.h"

#include "RFcom.h"
#include "Led.h"
#include "ScopedGuard.h"


#include "SerialOutput.h"
#include "TimeManager.h"

#include "Stepper.h"

#include "Wire.h"

#include "MPU6050.h"

#include "XT_DAC_Audio.h"
#include "0003idle_motor.mp3.h"
//#include "0005start_moving.wav.h"
//#include "0004moving.wav.h"
/*
0	pulled up	OK	outputs PWM signal at boot
1   TX
2	OK	OK	M2
3   RX
4	OK	OK  M2
5	OK	OK	outputs PWM signal at boot
6	x	x	connected to the integrated SPI flash
7	x	x	connected to the integrated SPI flash
8	x	x	connected to the integrated SPI flash
9	x	x	connected to the integrated SPI flash
10	x	x	connected to the integrated SPI flash
11	x	x	connected to the integrated SPI flash
12	OK	OK	boot fail if pulled high  - M2
13	OK	OK  SCN
14	OK	OK	outputs PWM signal at boot  - HL
15	OK	OK	outputs PWM signal at boot  - BL
16	OK	OK  TX2
17	OK	OK  RX2
18	OK	OK  SCK
19	OK	OK  MISO
21	OK	OK  SDA
22	OK	OK  SCL
23	OK	OK  MOSI
25	OK	OK  M1
26	OK	OK  M1
27	OK	OK  M1
32	OK	OK  TR
33	OK	OK  TL
34	OK		input only  Speed1
35	OK		input only  Speed2
36	OK		input only
39	OK		input only
*/

// SPI
constexpr PIN _MOSI = 23;
constexpr PIN _MISO = 19;
constexpr PIN _SCK = 18;
constexpr PIN _SS = 5;   //CE  in general can be connected to high as only recv is used , but leaving it
constexpr PIN _SCN = 15; //CSN

constexpr PIN _SCL = 22; //I2C
constexpr PIN _SDA = 21; //I2C


constexpr PIN motorLFrdPin = 26;
constexpr PIN motorLBwdPin = 27;
constexpr PIN motorLSpdReadPin = 35;

constexpr PIN motorRFrdPin = 2;
constexpr PIN motorRBwdPin = 4;
constexpr PIN motorRSpdReadPin = 34;

//constexpr PIN 14; high on boot-

constexpr PIN audioOutputPin = 25; //DAC

constexpr PIN turret1Pin = 32;
constexpr PIN turret2Pin = 33;
constexpr PIN turret3Pin = 13;
constexpr PIN turret4Pin = 12;

unsigned char address[][6] = { "1Node" }; // pipe address

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

PCA_Led< Adafruit_PWMServoDriver> headLight(14, driver);
PCA_Led< Adafruit_PWMServoDriver> backLight(15, driver);

arduino::utils::Timer scheduler("timer");

hw_timer_t * hr_timer = NULL;

//RF24 radio(_SS , _SCN );
//ce, csn
RF24 radio(_SS, _SCN, _SCK, _MISO, _MOSI);

SemaphoreHandle_t xSemaphore = NULL;
TaskHandle_t Task1;

long map(const long x, const long in_min, const long in_max, const long out_min, const long out_max)
{
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

	else if (in_max == in_min)
		return min(abs(in_max), abs(in_min));

	// map the input to the output range.
	// round up if mapping bigger ranges to smaller ranges
	else  if ((in_max - in_min) > (out_max - out_min))
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
	// round down if mapping smaller ranges to bigger ranges
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class Turret : public Stepper
{
public:
	Turret(int motor_pin_1, int motor_pin_2,
		int motor_pin_3, int motor_pin_4) : Stepper( 1028, motor_pin_1, motor_pin_2,
			motor_pin_3, motor_pin_4)
	{
		setSpeed(35);
	}

	void right() {
		if ( pdTRUE == xSemaphoreTake(xSemaphore, portMAX_DELAY )) {
			m_direction = 1;
			xSemaphoreGive(xSemaphore);
		}
	}

	void left( int16_t degree = -1 ) {
		if ( pdTRUE == xSemaphoreTake(xSemaphore, portMAX_DELAY )) {
			m_direction = -1;
			xSemaphoreGive(xSemaphore);
		}
	}

	void setDegree( int16_t degree ) {
		if ( pdTRUE == xSemaphoreTake(xSemaphore, portMAX_DELAY )) {

			ScopedGuard guard = makeScopedGuard(
				[]() { xSemaphoreGive(xSemaphore); });
			
			if ( degree < 0 )
				degree += 360 ;

			if ( degree > 360 )
				degree -= 360;

			int16_t delta_degree = degree - azimut();

			m_direction = ( delta_degree > 0 ) ? 1 : -1;

			delta_degree = abs( delta_degree );

			if ( delta_degree > 180 ) {
				m_direction = -m_direction;
				
				delta_degree = ( 360 - delta_degree ) ;
			}

			LOG_MSG("Current azimut " << (float)azimut() << " target " << (short)degree << " delta_degree " << (short)delta_degree );

			m_angle_steps = ( delta_degree  * number_of_steps / 360 ) * ( 160 / 12 );
		}
	}

	//move the motor X degrees + or -
	void move ( int16_t degrees ) {
			setDegree( azimut() + degrees);
	}

	void stop() {
		if (pdTRUE == xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
			LOG_MSG("stop turret " << azimut() );
			m_direction = 0;
			m_angle_steps = 0;
			xSemaphoreGive(xSemaphore);
		}
	}

	int16_t azimut() const {
		return m_azimut; //27 = 360 * 12 / 160
	}

	void  run() {
		
		if ( m_direction != 0 )
		{
			unsigned long now = micros();
			// move only if the appropriate delay has passed:
			if (now - this->last_step_time >= this->step_delay) {
				// get the timeStamp of when you stepped:
				this->last_step_time = now;

				if (pdTRUE == xSemaphoreTake(xSemaphore, portMAX_DELAY))
				{
					ScopedGuard guard = makeScopedGuard(
						[]() { xSemaphoreGive(xSemaphore); });

					if ( m_direction == 1 ) {
						this->step_number++;
						
						m_azimut += 0.35 * 12 / 160  ;
						
						if (m_azimut > 360)
							m_azimut = m_azimut - 360;

						if (this->step_number == this->number_of_steps) {
							this->step_number = 0;
						}
					}
					else if ( m_direction == -1 ) {
						if (this->step_number == 0) {
							this->step_number = this->number_of_steps;
						}
						
						m_azimut -= 0.35 * 12 / 160 ;
						
						if (m_azimut < 0)
							m_azimut = 360 + m_azimut;

						this->step_number--;
					}
					else
						return;

					if ( 1 == m_angle_steps )
						m_direction = 0;

					if ( m_angle_steps > 0 )
						m_angle_steps--;

				}

				stepMotor( this->step_number % 4 );
			}
		}
	}

private:
	int16_t m_direction = 0;
	int16_t m_angle_steps = 0;
	float m_azimut = 0;
};

Turret turret(turret1Pin, turret2Pin, turret3Pin, turret4Pin);

/*
нешний диаметр выходной передачи: 7 мм (12 зубьев 0,5 модуля)


Коэффициент уменьшения коробки передач: 1: 51,4 (приблизительно)

(Приблизительно 1028 импульсных выходных валов вращаются на 360 градусов)


Шаговый угол: двигатель 18 градусов, а выходной вал около 0,35 градусов.

Фазовое сопротивление: 31,3 Ом (6 в ток короткого замыкания 0,19 а)
*/

DFRobotDFPlayerMini dfPlayer;

//26,13,15,12,23,27,28,9,10,11,7,8
//volatile uint64_t counterR = 0 ;
//volatile uint64_t counterL = 0;
int varibale = 1;

portMUX_TYPE mux_l = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_r = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

class encCounter {
public:
	encCounter(PIN i_pin) : m_pin(i_pin) {
	}


	unsigned long calcRPM() volatile {

		unsigned int average_dt = 0, size = 0, rpm = 0;

		for (auto val : m_series_dt) {
			average_dt += val;

			if (val != 0)
				size++;
		}

		if (average_dt != 0) {
			average_dt /= size;
			rpm = (60000 / (average_dt * 4));
		}

		return constrain(rpm, 0, 1500);
	}

	uint64_t update(bool force = false) volatile {

		if (millis() - t > 5) {
			int curState = digitalRead(m_pin);
			m_index++;

			if (LOW == curState && curState != m_lastState) {
				m_series_dt[m_index % SIZE_OF_ARR(m_series_dt)] = millis() - t;
				++m_counter;
			}

			if (millis() - t > 350 && curState == m_lastState) {
				m_counter = 0;
				m_index = 0;
				memset((void*)m_series_dt, 0x0, sizeof(m_series_dt));
			}

			t = millis();
			m_lastState = curState;
		}

		return m_counter;
	}

	uint16_t get() volatile {
		return m_counter;
	}

	String toString() volatile {
		String res;

		res += "m_counter=";
		res += (long)m_counter;
		res += " rpm ";
		res += calcRPM();

		return res;
	}

	int m_lastState = HIGH;

	uint16_t m_series_dt[8];
	uint16_t m_counter = 0;
	uint32_t m_index = 0;

	uint64_t t = 0;

private:
	PIN  m_pin;
};

volatile encCounter counterL(motorLSpdReadPin);
volatile encCounter counterR(motorRSpdReadPin);


XT_DAC_Audio_Class DacAudio(audioOutputPin, 1);

enum SOUND_EFFECT_ID {
	SE_START_ENGINE = 0,
	SE_IDLE = 1,
	SE_MOVING = 2,
	SE_STOP_ENGINE = 3,
	//	SE_BREAK,
	//	SE_TURN_TURRET,
	//	SE_MOVE_BARREL,
	//	SE_FIRE ,
	SE_LAST
};

static struct XT_SoundEffect
{
	SOUND_EFFECT_ID id;
	XT_Wav_Class sound;
	XT_SoundEffect(SOUND_EFFECT_ID i_id, const unsigned char* i_wav, bool i_is_loop = false) : id(i_id), sound(XT_Wav_Class(i_wav)) { sound.RepeatForever = i_is_loop; }
} SoundEffects[] = {
	 XT_SoundEffect(SE_START_ENGINE , idle_motor , false) , // 0
	 XT_SoundEffect(SE_IDLE , idle_motor , true), // 1
	 XT_SoundEffect(SE_MOVING , idle_motor , true),
	 XT_SoundEffect(SE_STOP_ENGINE , idle_motor , false),//3
	 XT_SoundEffect(SE_LAST , idle_motor , false)	// 4
};

class SoundEffect {

public:
	SoundEffect() {
		next = []() {};
	}

	void run() {
		DacAudio.FillBuffer();

		// 		if ( dfPlayer.available() ) {
		// 			switch ( dfPlayer.readType() ) {
		// 			   case DFPlayerPlayFinished:
		// 				   next();
		// 				   break;
		// 			}				
		// 		}
	}

	void mute() {
		DacAudio.StopAllSounds();
	}

	void playSound(SOUND_EFFECT_ID i_sound, bool i_is_mix = true) {
		int16_t sound_id = constrain(i_sound, 0, SOUND_EFFECT_ID::SE_LAST);
		DacAudio.Play(&SoundEffects[sound_id].sound, i_is_mix);
	}

private:
	typedef std::function<void()> NextPlay;
	NextPlay next;
} soundEffect;

void breakHook();

constexpr int16_t motor_dead_zone = 25;

//BTS7960_1PWM   motorR( motorRFrdPin, motorRBwdPin, motorRSpdPin, breakHook, motor_dead_zone);
//BTS7960_1PWM   motorL( motorLFrdPin, motorLBwdPin, motorLSpdPin, []() {}, motor_dead_zone);

TA6586   motorR(motorRFrdPin, motorRBwdPin, breakHook, motor_dead_zone);
TA6586   motorL(motorLFrdPin, motorLBwdPin, []() {}, motor_dead_zone);


void breakHook()
{
	if (motorR.getSpeed() == motorL.getSpeed()) {
		//	soundEffect.playSound(SE_IDLE_RUN);
		backLight.turn_on();
	}
}

void IRAM_ATTR encMotorR() {

	portENTER_CRITICAL_ISR(&mux_r);
	ScopedGuard guard = makeScopedGuard(
		[]() { portEXIT_CRITICAL_ISR(&mux_r); });

	counterR.update();
}

void IRAM_ATTR encMotorL() {

	portENTER_CRITICAL_ISR(&mux_l);
	ScopedGuard guard = makeScopedGuard(
		[]() { portEXIT_CRITICAL_ISR(&mux_l); });

	counterL.update();
}

void IRAM_ATTR onMyTimer() {
	// Increment the counter and set the time of ISR
	/*portENTER_CRITICAL_ISR(&timerMux);
	isrCounter++;
	LOG_MSG("Timer " << isrCounter);
	portEXIT_CRITICAL_ISR(&timerMux);*/
	// Give a semaphore that we can check in the loop
	xSemaphoreGiveFromISR(timerSemaphore, NULL);
	// It is safe to use digitalRead/Write here if you want to toggle an output
}

/*
input from speed
setpoint needed value
kp any value
ki any value
kd any value
dt - time period the function called
minout - min
maxOut - max
*/
int32_t computePIDR(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
	float err = setpoint - input;
	static float integral = 0, prevErr = 0;
	integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
	float D = (err - prevErr) / dt;
	prevErr = err;
	return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

int32_t computePIDL(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
	float err = setpoint - input;
	static float integral = 0, prevErr = 0;
	integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
	float D = (err - prevErr) / dt;
	prevErr = err;
	return constrain(err * kp + integral + D * kd, minOut, maxOut);
}



struct {
	bool enabled = false;
	MPU6050 module;

	void initialize()
	{
		Wire.beginTransmission(0x68);
		if ( 0 == Wire.endTransmission() ) {
			LOG_MSG("Found MPU");
			module.initialize();
			enabled = true;
		}
		else
		{
			LOG_MSG("NOT Found MPU");
		}
	}

	MPU6050* operator->(){
		return &module;
	}
} gyro;


void setup()
{
	//Setup radio
	radio.begin();
	radio.setAutoAck(1);
	radio.setRetries(0, 3 /*0*/);
	radio.enableAckPayload();
	radio.setPayloadSize(sizeof(arduino::utils::Payload)/*32*/);
	radio.enableDynamicPayloads();
	radio.openReadingPipe(1, (const uint8_t *)address[0]);
	radio.setChannel(0x64);
	radio.setPALevel(RF24_PA_MIN); //RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
	radio.setDataRate(RF24_250KBPS); //RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
	radio.powerUp();
	radio.startListening();


	// Create semaphore to inform us when the timer has fired
	timerSemaphore = xSemaphoreCreateBinary();

	// Use 1st timer of 4 (counted from zero).
	// Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
	// info).
	hr_timer = timerBegin(0, 80, true);

	// Attach onTimer function to our timer.
	timerAttachInterrupt(hr_timer, &onMyTimer, true);

	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter)
	timerAlarmWrite(hr_timer, 500000, true); //0.1 sec

	// Start an alarm
	timerAlarmEnable(hr_timer);

	//If slot is closed high if open low.
	pinMode(motorLSpdReadPin, INPUT_PULLUP);
	pinMode(motorRSpdReadPin, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(motorLSpdReadPin), encMotorL, CHANGE);
	attachInterrupt(digitalPinToInterrupt(motorRSpdReadPin), encMotorR, CHANGE);
	/*
	timer.addRecuringTask( TIME.getEpochTime() , 1, [](long&) {
		portENTER_CRITICAL_ISR(&mux);



		portEXIT_CRITICAL_ISR(&mux); }
	);
	*/

	motorR.begin();
	motorL.begin();

	//setup i2C

	Wire.begin();

	driver.begin();
	driver.setPWMFreq(1600);
	driver.setOutputMode(false);// + and pwm   if true ( - and pwm ) 

	gyro.initialize();

	//Init CORE 1
	xSemaphore = xSemaphoreCreateBinary();//created locked

	disableCore0WDT();

	xTaskCreatePinnedToCore(
		[](void *) {
		try {
			while (true) {
				/*motorR.run();
				motorL.run();
				soundEffect.run();*/
				turret.run();
				handleGYRO();
			}
		}
		catch (...) {
			LOG_MSG("Critical error");
			}
		}
		, "Task1"   // A name just for humans
		, 4 * 1024  // This stack size can be checked & adjusted by reading the Stack Highwater
		, NULL
		, 2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
		, &Task1
		, 0);

	xSemaphoreGive(xSemaphore);
	////////////////////////////////////////////////
	
	delay(1000);

	LOG_MSG("Ready");
}


void alg1(int16_t nJoyX, int16_t nJoyY, int16_t &o_m1 /*L*/, int16_t &o_m2 /*R*/)
{
	float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
	float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
	int32_t nPivSpeed;      // Pivot Speed                          (-128..+127)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
	float   fPivYLimit = 50; //should be more or less equal to dead zone of motor 40 tested

	// INPUTS
	//int32_t     nJoyX;              // Joystick X input                     (-128..+127)
	//int32_t     nJoyY;              // Joystick Y input                     (-128..+127)

	// OUTPUTS
	//int32_t     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
	//int32_t     nMotMixR;           // Motor (right) mixed output           (-128..+127)

	// Calculate Drive Turn output due to Joystick X input
	if (nJoyY >= 0) {
		// Forward
		nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
		nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
	}
	else {
		// Reverse
		nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
		nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
	}

	// Scale Drive output due to Joystick Y input (throttle)
	nMotPremixL = nMotPremixL * nJoyY / 127.0;
	nMotPremixR = nMotPremixR * nJoyY / 127.0;

	// Now calculate pivot amount
	// - Strength of pivot (nPivSpeed) based on Joystick X input
	// - Blending of pivot vs drive (fPivScale) based on Joystick Y input
	nPivSpeed = constrain(nJoyX, -20, 20); // higher the higher speed of spot rotation this is for 64
	fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

	// Calculate final mix of Drive and Pivot
	o_m1 = (1.0 - fPivScale)*nMotPremixL + fPivScale * (nPivSpeed);
	o_m2 = (1.0 - fPivScale)*nMotPremixR + fPivScale * (-nPivSpeed);
}

void handlePID() {

	float kp = 0.5; //2.0;
	float ki = 0.9; ////0.9;
	float kd = 0.1;// 0.1;
	portENTER_CRITICAL_ISR(&mux_l);
	static int trendLeft = 0;
	static int trendRight = 0;

	ScopedGuard guard = makeScopedGuard([]() {
		if (counterL.get() != 0) {
			LOG_MSG("L: " << counterL.toString());
		}
		counterL.update(true);

		portEXIT_CRITICAL_ISR(&mux_l);
	});

	portENTER_CRITICAL_ISR(&mux_r);
	ScopedGuard guard1 = makeScopedGuard([]() {
		if (counterR.get() != 0) {
			LOG_MSG("R: " << counterR.toString());
		}
		counterR.update(true);

		portEXIT_CRITICAL_ISR(&mux_r);

	});

	static int prev_counterR = 0;
	static int prev_counterL = 0;

	if (motorR.getSpeed() != motorL.getSpeed() || motorR.getSpeed() == 0 && motorL.getSpeed() == 0) {
		counterL.m_counter = 0;
		counterR.m_counter = 0;
		return;
	}

	unsigned int leftRPM = counterL.calcRPM();
	unsigned int rightRPM = counterR.calcRPM();

	int32_t pwmR = computePIDR(rightRPM, leftRPM, kp, ki, kd, 0.5, 0, 2000);
	//pwmR = map(pwmR, -10, 10, -20, 20);

	int32_t pwmL = computePIDL(leftRPM, rightRPM, kp, ki, kd, 0.5, 0, 2000);
	//	pwmL = map(pwmL, -10, 10, -20, 20);

		//motorR.adjust( motorR.getSpeed() + pwmR );
		//motorL.adjust( motorL.getSpeed() + pwmL );



	// 	if ( speed != 0 && motorL.getSpeed() == motorR.getSpeed() )
	// 	{
	// 		if ( 0 == counterR.m_counter ) {
	// 			LOG_MSG("Boost right");
	// 		
	// 			if (motorR.getDirection() == Motor::Direction::FORWARD)
	// 				motorR.forward( speed + 0.1 * speed );
	// 			if (motorR.getDirection() == Motor::Direction::BACKWARD)
	// 				motorR.backward( speed + 0.1 * speed );
	// 
	// 			delay(25);
	// 
	// 			if (motorR.getDirection() == Motor::Direction::FORWARD)
	// 				motorR.forward(speed);
	// 			if (motorR.getDirection() == Motor::Direction::BACKWARD)
	// 				motorR.backward(speed);
	// 
	// 		}
	// 
	// 		if ( 0 == counterL.m_counter ) {
	// 
	// 			LOG_MSG("Boost left");
	// 		
	// 			if (motorL.getDirection() == Motor::Direction::FORWARD)
	// 				motorL.forward(speed + 0.1 * speed);
	// 			if (motorL.getDirection() == Motor::Direction::BACKWARD)
	// 				motorL.backward(speed + 0.1 * speed);
	// 
	// 			delay(25);
	// 
	// 			if (motorL.getDirection() == Motor::Direction::FORWARD)
	// 				motorL.forward(speed);
	// 			if (motorL.getDirection() == Motor::Direction::BACKWARD)
	// 				motorL.backward(speed);
	// 		}
	// 	}
	// 
	// 		

		//float rouds_per_secR = (float) ( counterR.m_counter / 4.0 ) / 0.7;
		//float rouds_per_secL = (float) ( counterL.m_counter / 4.0 ) / 0.7;


		//int32_t pwmR = computePIDR( rouds_per_secR, rouds_per_secL , kp, ki, kd, 0.5 , 0 , 255 );
		//int32_t pwmL = computePIDL( rouds_per_secL, rouds_per_secR , kp, ki, kd, 0.5 , 0 , 255 );

		//LOG_MSG( "L : " << (float)rouds_per_secL << " R: " << (float)rouds_per_secR );

}

void handleGYRO()
{
	static unsigned long last_step_time = 0;
	unsigned long now = millis();
	// move only if the appropriate delay has passed:
	if (now -  last_step_time >= 1000) {
		// get the timeStamp of when you stepped:
		last_step_time = now;
		if (gyro.enabled) {
		
			LOG_MSG(gyro->getRotationX() );
			LOG_MSG(gyro->getAccelerationX() );
		}
	}
}

enum MODE { IDLE, MOVING, SECURED };

MODE mode = SECURED;

void stop_all() {

	if (mode != SECURED)
		soundEffect.playSound(SE_STOP_ENGINE, false);

	mode = SECURED;

	motorL.stop();
	motorR.stop();
}

void loop()
{
	using namespace arduino::utils;
	static uint64_t t = millis();
	byte pipeNo;
	static unsigned long lastRecievedTime = millis();
	static Payload recieved_data;
	static Payload pre_recieved_data;

	int16_t rMotor = 0;
	int16_t lMotor = 0;

	if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
		//adjust the speed
		handlePID();
		TIME.run();
		scheduler.run();
	}

	while (radio.available(&pipeNo))
	{
		radio.read(&recieved_data, sizeof(recieved_data));

		if (!recieved_data.isValid()) {
			LOG_MSG(F("Garbage on RF"));
			continue;
		}

		lastRecievedTime = millis();

		if (pre_recieved_data == recieved_data) {
			continue;
		}

		LOG_MSG(F("RCV : ")
			<< F(" j0 ") << recieved_data.m_j[0]
			<< F(" j1 ") << recieved_data.m_j[1]
			<< F(" j2 ") << recieved_data.m_j[2]
			<< F(" j3 ") << recieved_data.m_j[3]
			<< F(" Bits ") << recieved_data.m_b1
			<< F(" ") << recieved_data.m_b2
			<< F(" ") << recieved_data.m_b3
			<< F(" ") << recieved_data.m_b4);

		if (recieved_data.m_b3) {
			headLight.turn_on(Led::Brightness::_100);
			backLight.turn_on(Led::Brightness::_100);
		}
		else
		{
			headLight.turn_off();
			backLight.turn_off();
		}

		/* Calculate motor PWM */
		alg1(recieved_data.m_steering, recieved_data.m_speed, lMotor, rMotor);

		( lMotor > 0 ) ? motorL.backward(map(lMotor, 0, 127, motor_dead_zone - 3, 255))
			: motorL.forward(map(lMotor, -127, 0, 255, motor_dead_zone - 3));

		( rMotor > 0 ) ? motorR.backward(map(rMotor, 0, 127, motor_dead_zone - 3, 255))
			: motorR.forward(map(rMotor, -127, 0, 255, motor_dead_zone - 3));

		if (SECURED == mode)
			soundEffect.playSound(SE_START_ENGINE, false);

		if (((uint8_t)motorR.getDirection() | (uint8_t)motorL.getDirection())
			!= (uint8_t)Motor::Direction::STOPED) {
			soundEffect.playSound(SE_MOVING, true);
			mode = MOVING;
		}
		else {
			soundEffect.playSound(SE_IDLE, true);
			mode = IDLE;
		}

		if ( recieved_data.m_b1 ) {
			turret.setDegree(90);
		}

		if ( recieved_data.m_b2 ) {
			turret.setDegree(-90);
		}

		if ( 0 == ( recieved_data.m_b1 | recieved_data.m_b2 ) )
		{
			/************************************************************************/
			if (recieved_data.m_j4 >= 5) {
				//turret.right( map(recieved_data.m_j4, 0, 127, 0, 180) );
				turret.right();
			}
			else if (recieved_data.m_j4 <= -5) {
				turret.left();
				//turret.left( map(recieved_data.m_j4, -127, 0, 180, 0) );
			}
			else {
				turret.stop();
			}
		}
		


		// Send ack
		payLoadAck.speed = lMotor;
		payLoadAck.batteryLevel = lMotor;
		radio.writeAckPayload(pipeNo, &payLoadAck, sizeof(payLoadAck));

		pre_recieved_data = recieved_data;
	}

	if (lastRecievedTime < millis() - 3 * arduino::utils::RF_TIMEOUT_MS) {
		LOG_MSG("Lost connection");

		stop_all();

		headLight.fade(750);
		backLight.fade(750);

		lastRecievedTime = millis();
	}
}


/*

void alg2(int16_t nJoyX, int16_t nJoyY, int16_t &o_m1, int16_t &o_m2)
{
	float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
	float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
	int32_t nPivSpeed;      // Pivot Speed                          (-128..+127)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
	float fPivBearLimit = 75.0;


	// Calculate Drive Turn output due to Joystick X input
	if (nJoyY >= 0) {
		// Forward
		nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
		nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
	}
	else {
		// Reverse
		nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
		nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
	}

	// Scale Drive output due to Joystick Y input (throttle)
	nMotPremixL = nMotPremixL * nJoyY / 127.0;
	nMotPremixR = nMotPremixR * nJoyY / 127.0;

	nPivSpeed = nJoyX;
	float fBearMag;
	if (nJoyY == 0) {
		// Handle special case of Y-axis=0
		if (nJoyX == 0) {
			fBearMag = 0;
		}
		else {
			fBearMag = 90;
		}
	}
	else {
		// Bearing (magnitude) away from the Y-axis is calculated based on the
		// Joystick X & Y input. The arc-tangent angle is then converted
		// from radians to degrees.
		fBearMag = atan((float)abs(nJoyX) / (float)abs(nJoyY)) * 90.0 / (3.14159 / 2.0);
	}

	// Blending of pivot vs drive (fPivScale) based on Joystick bearing
	fPivScale = (fBearMag < fPivBearLimit) ? 0.0 : (fBearMag - fPivBearLimit) / (90.0 - fPivBearLimit);

	// Calculate final mix of Drive and Pivot
	o_m1 = (1.0 - fPivScale)*nMotPremixL + fPivScale * (nPivSpeed);
	o_m2 = (1.0 - fPivScale)*nMotPremixR + fPivScale * (-nPivSpeed);
}

*/