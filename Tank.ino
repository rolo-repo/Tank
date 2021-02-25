/*
    Name:       Tank.ino
    Created:	03-Jan-21 11:28:49
    Author:     NTNET\ROMANL
*/
#define  ENABLE_LOGGER
#include "SPI.h"
#include "RF24.h"
#include "nRF24L01.h"

#include "Constants.h"
#include "BTS7960.h"
#include "RFcom.h"
#include "Led.h"
#include "ScopedGuard.h"


#include "SerialOutput.h"
#include "TimeManager.h"

/*
0	pulled up	OK	outputs PWM signal at boot
1	TX pin	OK	debug output at boot
2	OK	OK	connected to on-board LED                 
3	OK	RX pin	HIGH at boot
4	OK	OK  M2
5	OK	OK	outputs PWM signal at boot  -  SS
6	x	x	connected to the integrated SPI flash
7	x	x	connected to the integrated SPI flash
8	x	x	connected to the integrated SPI flash
9	x	x	connected to the integrated SPI flash
10	x	x	connected to the integrated SPI flash
11	x	x	connected to the integrated SPI flash
12	OK	OK	boot fail if pulled high
13	OK	OK  SCN
14	OK	OK	outputs PWM signal at boot  - HL
15	OK	OK	outputs PWM signal at boot  - BL
16	OK	OK  M2
17	OK	OK  M2
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
constexpr PIN _SCK = 22;
constexpr PIN _SS = 5;
constexpr PIN _SCN = 13; //any pin

constexpr PIN _SCL = 22; //I2C
constexpr PIN _SDA = 21; //I2C

constexpr PIN motorEnablePin = 26;
constexpr PIN motorLFrdPin = 25; //INT0
constexpr PIN motorLSpdPin = 26; //PWM //INT1
constexpr PIN motorLBwdPin = 27;
constexpr PIN motorLSpdReadPin = 35;

constexpr PIN motorRFrdPin = 16; //INT0
constexpr PIN motorRSpdPin = 4;; //PWM //INT1 
constexpr PIN motorRBwdPin = 17;
constexpr PIN motorRSpdReadPin = 34;

constexpr PIN headLightPin = 14;
constexpr PIN backLightPin = 15;

constexpr PIN turretLeftPin = 32;
constexpr PIN turretRightPin = 33;
constexpr PIN ledPin = 2;

unsigned char address[][6] = { "1Node" }; // pipe address

Led headLight(headLightPin);
Led backLight(backLightPin);
Led stsLed(ledPin);

BTS7960_1PWM   motorR( motorRFrdPin, motorRBwdPin, motorRSpdPin, []() { backLight.turn_on(Led::Brightness::_100); } , 110 );
BTS7960_1PWM   motorL( motorLFrdPin, motorLBwdPin, motorLSpdPin, []() { backLight.turn_on(Led::Brightness::_100); } , 110 );

arduino::utils::Timer timer("timer");

hw_timer_t * hr_timer = NULL;

//RF24 radio(_SS , _SCN );
//ce, csn
RF24 radio( _SS , _SCN, _SCK, _MISO, _MOSI );

//26,13,15,12,23,27,28,9,10,11,7,8
//volatile uint64_t counterR = 0 ;
//volatile uint64_t counterL = 0;
int varibale = 1;

portMUX_TYPE mux_l = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_r = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

class encCounter {
public:
	encCounter( PIN i_pin ) : m_pin ( i_pin ) {
	}

	unsigned long calcRPM() volatile {
		if ( m_counter != 0) {
			return (60000 / ( dt * 4 ) );
		}
		else {
			return 0;
		}
	}

	uint64_t update() volatile {
	//	bool curState = digitalRead( m_pin ) ;
		
		if ( millis() - t > 10 ) {
			dt = millis() - t;
			t  = millis();

		//	m_lastState = curState;
			m_counter++;
		}
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

	bool m_lastState = 0;
	uint64_t m_counter = 0;
	
	uint64_t t =  0;
	uint64_t dt = 0;

private :
	PIN  m_pin;
};

volatile encCounter counterL( motorLSpdReadPin );
volatile encCounter counterR( motorRSpdReadPin );

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

void IRAM_ATTR onTimer() {
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
int32_t computePIDR( float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut ) {
	float err = setpoint - input;
	static float integral = 0, prevErr = 0;
	integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
	float D = (err - prevErr) / dt;
	prevErr = err;
	return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

int32_t computePIDL( float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut ) {
	float err = setpoint - input;
	static float integral = 0, prevErr = 0;
	integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
	float D = (err - prevErr) / dt;
	prevErr = err;
	return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void setup()
{
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
	stsLed.turn_on();

	// Create semaphore to inform us when the timer has fired
	timerSemaphore = xSemaphoreCreateBinary();

	// Use 1st timer of 4 (counted from zero).
	// Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
	// info).
	hr_timer = timerBegin(0, 80, true);

	// Attach onTimer function to our timer.
	timerAttachInterrupt( hr_timer, &onTimer, true );

	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter)
	timerAlarmWrite( hr_timer, 100000 , true ); //0.1 sec

	// Start an alarm
	timerAlarmEnable( hr_timer );

	//analogWriteFrequency(1000);
	//analogWriteResolution(8);
	/*ledcSetup( 0, 1000, 8 );
	ledcAttachPin( motorRSpdPin, 0 );*/
	
	motorR.begin();
	motorL.begin();

	delay(1000);

	//If slot is closed high if open low.
	pinMode( motorLSpdReadPin, INPUT_PULLUP );
	pinMode( motorRSpdReadPin, INPUT_PULLUP );

	attachInterrupt( digitalPinToInterrupt(motorLSpdReadPin), encMotorL, FALLING);
	attachInterrupt( digitalPinToInterrupt(motorRSpdReadPin), encMotorR, FALLING);
	/*
	timer.addRecuringTask( TIME.getEpochTime() , 1, [](long&) {
		portENTER_CRITICAL_ISR(&mux);
		
		
		
		portEXIT_CRITICAL_ISR(&mux); }
	);
	*/
	/*
	timer.addRecuringTask(TIME.getEpochTime(), 1, [](long&) {
		static uint64_t dt = millis();
		float kp = 0.3; //2.0;
		float ki = 1 ; ////0.9;
		float kd = 2 ;// 0.1;
		portENTER_CRITICAL_ISR(&mux);

		LOG_MSG( "L : " << (unsigned  long)counterL.get() << " R: " << (unsigned long)counterR.get() );

		if ( motorR.getSpeed() == motorL.getSpeed() )
		{
			if (  counterL.update()  > 0 )
			{
			//	uint16_t speed = motorR.getSpeed() * (counterL.get() - counterR.get()) / (counterL.get() + counterR.get());

				uint16_t speed = computePIDR( counterR.get() , counterL.get(), kp, ki, kd, (float)( millis() - dt ) / 1000.0 , counterL.get() , 255) ;
				
				speed = motorR.getSpeed() * ( float )( speed / counterL.get() );
				
				if (motorR.getDirection() == Motor::Direction::FORWARD)
					motorR.forward(speed);
				if (motorR.getDirection() == Motor::Direction::BACKWARD)
					motorR.backward(speed);
				
				LOG_MSG("1 "  << speed);
			}
			else if (counterR.get() > 0 )
			{
				//	uint16_t speed = motorR.getSpeed() * (counterR.get() - counterL.get()) / (counterL.get() + counterR.get());

				uint16_t speed = computePIDL( counterL.get(), counterR.get(), kp, ki, kd, (float)(millis() - dt) / 1000.0, counterR.get(), 255);

				speed = motorL.getSpeed() * (float) ( speed /  counterR.get() );

				if (motorL.getDirection() == Motor::Direction::FORWARD)
					motorL.forward(speed);
				if (motorL.getDirection() == Motor::Direction::BACKWARD)
					motorL.backward(speed);

				LOG_MSG("2 " << speed);
			}
		}

		counterL.reset();
		counterR.reset();

		portEXIT_CRITICAL_ISR(&mux);

		dt = millis();
	}
	);
	*/

	LOG_MSG("Ready");
}

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

void alg1( int16_t nJoyX, int16_t nJoyY, int16_t &o_m1 /*L*/ , int16_t &o_m2 /*R*/)
{
	float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
	float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
	int32_t nPivSpeed;      // Pivot Speed                          (-128..+127)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
	float   fPivYLimit = 35; //should be more or less equal to dead zone of motor

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
	nPivSpeed = constrain(nJoyX, -64, 64 );
	fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

	// Calculate final mix of Drive and Pivot
	o_m1 = (1.0 - fPivScale)*nMotPremixL + fPivScale * (nPivSpeed);
	o_m2 = (1.0 - fPivScale)*nMotPremixR + fPivScale * (-nPivSpeed);
}

void handlePID() {
	
	float kp = 0.05; //2.0;
	float ki = 0.01; ////0.9;
	float kd = 0.02;// 0.1;
	portENTER_CRITICAL_ISR(&mux_l);

	ScopedGuard guard = makeScopedGuard([]() { 
		if ( counterL.m_counter != 0 ){
			LOG_MSG("L: " << (long)counterL.dt  << " "  << counterL.toString())
			
		}
		counterL.m_counter = 0;
	
		portEXIT_CRITICAL_ISR(&mux_l);
	});

	portENTER_CRITICAL_ISR(&mux_r);
	ScopedGuard guard1 = makeScopedGuard([]() {
		if (counterR.m_counter != 0) {
			LOG_MSG("R: " << (long)counterR.dt << " " << counterR.toString() );
			counterR.m_counter = 0;
		}
      portEXIT_CRITICAL_ISR(&mux_r);

	});

	if ( !counterL.m_counter && !counterR.m_counter ) {
		return;
	}

	if ( motorL.getSpeed() != 0 && motorL.getSpeed() == motorR.getSpeed() )
	{
		if ( 0 == counterR.m_counter ) {
			LOG_MSG("Boost right");
			uint16_t speed = motorR.getSpeed();

			if (motorR.getDirection() == Motor::Direction::FORWARD)
				motorR.forward( speed + 0.1 * speed );
			if (motorR.getDirection() == Motor::Direction::BACKWARD)
				motorR.backward( speed + 0.1 * speed );

			delay(25);

			if (motorR.getDirection() == Motor::Direction::FORWARD)
				motorR.forward(speed);
			if (motorR.getDirection() == Motor::Direction::BACKWARD)
				motorR.backward(speed);

		}

		if ( 0 == counterL.m_counter ) {

			LOG_MSG("Boost left");
			uint16_t speed = motorL.getSpeed();

			if (motorL.getDirection() == Motor::Direction::FORWARD)
				motorL.forward(speed + 0.1 * speed);
			if (motorL.getDirection() == Motor::Direction::BACKWARD)
				motorL.backward(speed + 0.1 * speed);

			delay(25);

			if (motorL.getDirection() == Motor::Direction::FORWARD)
				motorL.forward(speed);
			if (motorL.getDirection() == Motor::Direction::BACKWARD)
				motorL.backward(speed);
		}
	}

		

	//float rouds_per_secR = (float) ( counterR.m_counter / 4.0 ) / 0.5;
	//float rouds_per_secL = (float) ( counterL.m_counter / 4.0 ) / 0.5;


	//int32_t pwmR = computePIDR( rouds_per_secR, rouds_per_secL , kp, ki, kd, 0.5 , 0 , 255 );
	//int32_t pwmL = computePIDL( rouds_per_secL, rouds_per_secR , kp, ki, kd, 0.5 , 0 , 255 );

	//LOG_MSG( "L : " << (float)rouds_per_secL << " R: " << (float)rouds_per_secR );
	
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

	TIME.run();
	timer.run();
	motorR.run(); 
	motorL.run();

	if ( xSemaphoreTake( timerSemaphore, 0 ) == pdTRUE ) {
		//adjust the speed
		handlePID();
	}

	while (  radio.available(&pipeNo) )
	{
		radio.read(&recieved_data, sizeof(recieved_data));

		if (!recieved_data.isValid()) {
			LOG_MSG(F("Garbage on RF") );
			continue;
		}

		lastRecievedTime = millis();
		
        if ( pre_recieved_data == recieved_data ) {
			continue;
		}
		
		pre_recieved_data = recieved_data;

		LOG_MSG( F("RCV : ")  
			<< F(" j0 ") << recieved_data.m_j[0]
			<< F(" j1 ") << recieved_data.m_j[1]
			<< F(" j2 ") << recieved_data.m_j[2]
			<< F(" j3 ") << recieved_data.m_j[3]
			<< F(" Bits ") << recieved_data.m_b1
			<< F(" ") << recieved_data.m_b2
			<< F(" ") << recieved_data.m_b3
			<< F(" ") << recieved_data.m_b4);

		if ( recieved_data.m_b3 ) {
			headLight.turn_on(Led::Brightness::_100);
			backLight.turn_on(Led::Brightness::_100);
		}
		else
		{
			headLight.turn_off();
			backLight.turn_off();
		}

		if (recieved_data.m_b1) {
			motorL.stop();
			motorR.stop();
		}

		/* Calculate motor PWM */
		alg1( -recieved_data.m_steering , recieved_data.m_speed, lMotor, rMotor );

		if (motorL.getSpeed() != lMotor || rMotor != motorR.getSpeed() ) {
			( lMotor > 0 ) ? motorL.backward( map( lMotor, 0, 127, 100, 255) )
				: motorL.forward( map( lMotor, -127, 0, 255, 100) );
			( rMotor > 0 ) ? motorR.backward( map(rMotor, 0, 127, 100, 255) )
				: motorR.forward( map( rMotor, -127, 0, 255, 100) );
		}

	/************************************************************************/
		/*
		if (recieved_data.m_j4 > 0)
		{
			turret.right(map(recieved_data.m_j4, 0, 127, 0, 255));
		}
		else
		{
			turret.left(map(recieved_data.m_j4, -127, 0, 255, 0));
		}
		*/

		// Send ack
		payLoadAck.speed = lMotor;
		payLoadAck.batteryLevel = lMotor;
		radio.writeAckPayload(pipeNo, &payLoadAck, sizeof(payLoadAck));
	}

	if ( lastRecievedTime < millis() - 3 * arduino::utils::RF_TIMEOUT_MS ) {
		LOG_MSG("Lost connection");
		stsLed.blynk();

		motorR.stop(); motorL.stop();

		headLight.fade(500);

		lastRecievedTime = millis();
	}

	/*if ( ! radio.isChipConnected() ) {
		RESET();
	}*/
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