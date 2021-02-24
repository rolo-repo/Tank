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
constexpr PIN motorLSpdReadPin = 34;

constexpr PIN motorRFrdPin = 16; //INT0
constexpr PIN motorRSpdPin = 4;; //PWM //INT1 
constexpr PIN motorRBwdPin = 17;
constexpr PIN motorRSpdReadPin = 35;

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

//RF24 radio(_SS , _SCN );
//ce, csn
RF24 radio( _SS , _SCN, _SCK, _MISO, _MOSI );

//26,13,15,12,23,27,28,9,10,11,7,8
//volatile uint64_t counterR = 0 ;
//volatile uint64_t counterL = 0;
int varibale = 1;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

class encCounter {
public:
	encCounter( PIN i_pin) : m_pin ( i_pin ) {
		dt = millis();
	}



	void reset() volatile  {
		m_counter = 0; 
		
	}

	uint64_t get() volatile const {
		return m_counter;
	}

	void filter(int newVal) volatile {
		
		m_buf[ ++m_count % SIZE_OF_ARR(m_buf)] = newVal;

		uint32_t middle = 0;
		
		for ( uint16_t i = 0 ; i < SIZE_OF_ARR(m_buf) ; i++ ) {
			middle += m_buf[i];
		}

		m_counter = middle / SIZE_OF_ARR(m_buf);
	}

	uint64_t update(int i_direction = 1)  volatile {
		bool curState = digitalRead(m_pin);

		if ( millis() - dt  > 10 && m_lastState != curState) {
			dt = millis();
			m_lastState = curState;
			if (!curState) {
				filter( m_counter += i_direction );
			}
		}

		return m_counter;
	}

private:
	bool m_lastState = 0;
	uint64_t m_counter = 0;
	uint16_t m_buf[3] = { 0,0,0 };
	uint16_t m_count = 0;
	PIN  m_pin;
	uint64_t dt = 0;

	
};

volatile encCounter counterR( motorRSpdPin );
volatile encCounter counterL( motorLSpdPin );

void IRAM_ATTR encMotor() {
	portENTER_CRITICAL_ISR(&mux);
	counterR.update();
	counterL.update();
	portEXIT_CRITICAL_ISR(&mux);
}

/*
int filter(int newVal) {
	_buf[_count] = newVal;
	if (++_count >= 3) _count = 0;
	int middle = 0;
	if ((_buf[0] <= _buf[1]) && (_buf[0] <= _buf[2])) {
		middle = (_buf[1] <= _buf[2]) ? _buf[1] : _buf[2];
	}
	else {
		if ((_buf[1] <= _buf[0]) && (_buf[1] <= _buf[2])) {
			middle = (_buf[0] <= _buf[2]) ? _buf[0] : _buf[2];
		}
		else {
			middle = (_buf[0] <= _buf[1]) ? _buf[0] : _buf[1];
		}
	}
	_middle_f += (middle - _middle_f) * 0.7;
	return _middle_f;
}
*/

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
int32_t computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
	float err = setpoint - input;
	static float integral = 0, prevErr = 0;
	integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
	float D = (err - prevErr) / dt;
	prevErr = err;
	return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void setup()
{
	//LOG_MSG_BEGIN(115200);
	//delay(1000);

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

	
	//analogWriteFrequency(1000);
	//analogWriteResolution(8);
	/*ledcSetup( 0, 1000, 8 );
	ledcAttachPin( motorRSpdPin, 0 );*/
	
	motorR.begin();
	motorL.begin();

	delay(1000);

	//If slot is closed high if open low.
	pinMode(motorLSpdReadPin, INPUT );
	pinMode(motorRSpdReadPin, INPUT );

	attachInterrupt( digitalPinToInterrupt(motorLSpdReadPin), encMotor, FALLING );
	attachInterrupt( digitalPinToInterrupt(motorRSpdReadPin), encMotor, FALLING);

	timer.addRecuringTask( TIME.getEpochTime() , 1, [](long&) {
		portENTER_CRITICAL_ISR(&mux);
		LOG_MSG("L : " << ( unsigned  long )counterL.get() << " R: " << (unsigned long) counterR.get());
		
		counterL.reset();
		counterR.reset();

		portEXIT_CRITICAL_ISR(&mux); }
	);


	timer.addRecuringTask(TIME.getEpochTime(), 1, [](long&) {
		static uint64_t dt = millis();
		float kp = 2.0;
		float ki = 0.9;
		float kd = 0.1;
		portENTER_CRITICAL_ISR(&mux);
		if ( motorR.getSpeed() == motorL.getSpeed() )
		{
			if ( counterR.get() < counterL.get() )
			{
				uint16_t speed = computePID(0, motorL.getSpeed() / counterL.get(), kp, ki, kd, (millis() - dt) / 1000, 0, 255) *  counterL.get();
				if (motorR.getDirection() == Motor::Direction::FORWARD)
					motorR.forward(speed );
				if (motorR.getDirection() == Motor::Direction::BACKWARD)
					motorR.backward( speed );
				LOG_MSG("1 " << speed);
			}
			else if ( counterR.get() > counterL.get() )
			{
				uint16_t speed = computePID(0, motorL.getSpeed() / counterR.get(), kp, ki, kd, (millis() - dt) / 1000, 0, 255) *  counterR.get();

				if (motorL.getDirection() == Motor::Direction::FORWARD)
					motorL.forward(speed );
				if (motorL.getDirection() == Motor::Direction::BACKWARD)
					motorL.backward(speed );
				LOG_MSG("2 "<<speed);
			}
		}
		portEXIT_CRITICAL_ISR(&mux);

		dt = millis();
	}
	);
	

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


void alg2( int16_t nJoyX, int16_t nJoyY, int16_t &o_m1, int16_t &o_m2 )
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

void loop()
{
	using namespace arduino::utils;
	static uint64_t t = millis();
	byte pipeNo;
	static unsigned long lastRecievedTime = millis();
	static Payload recieved_data;

	int16_t rMotor = 0;
	int16_t lMotor = 0;

	TIME.run();
	timer.run();

	while (  radio.available(&pipeNo) )
	{
		radio.read(&recieved_data, sizeof(recieved_data));

		if (!recieved_data.isValid())
		{
			LOG_MSG(F("Garbage on RF") );
			continue;
		}

		stsLed.blynk();
		
		LOG_MSG( F("RCV : ")  
			<< F(" j0 ") << recieved_data.m_j[0]
			<< F(" j1 ") << recieved_data.m_j[1]
			<< F(" j2 ") << recieved_data.m_j[2]
			<< F(" j3 ") << recieved_data.m_j[3]
			<< F(" Bits ") << recieved_data.m_b1
			<< F(" ") << recieved_data.m_b2
			<< F(" ") << recieved_data.m_b3
			<< F(" ") << recieved_data.m_b4);
		
		

		/*
		LOG_MSG(F("RCV  Speed:") << recieved_data.m_speed
			<< F(" Direction: ") << ((recieved_data.m_speed > 0) ? F("BACKWARD") : F("FORWARD"))
			<< F(" Steering: ") << recieved_data.m_steering
			<< ((recieved_data.m_steering > 0) ? F(" LEFT") : F(" RIGHT"))
			<< F(" Bits ") << recieved_data.m_b1
			<< F(" ") << recieved_data.m_b2
			<< F(" ") << recieved_data.m_b3
			<< F(" ") << recieved_data.m_b4);*/

		
		
		lastRecievedTime = millis();
	
		if ( recieved_data.m_b3 )
		{
			headLight.turn_on(Led::Brightness::_100);
			backLight.turn_on(Led::Brightness::_100);
		}
		else
		{
			headLight.turn_off();
			backLight.turn_off();
		}

		if (recieved_data.m_b1)
		{
			motorL.stop();
			motorR.stop();
		}

		if ( ! ( recieved_data.m_b2 | recieved_data.m_b1 ) )
		{
			/* Calculate motor PWM */
			alg1( recieved_data.m_steering, recieved_data.m_speed, lMotor, rMotor );

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

	if ( lastRecievedTime < millis() - 3 * arduino::utils::RF_TIMEOUT_MS )
	{
		LOG_MSG("Lost connection");
		stsLed.blynk();

		motorR.stop(); motorL.stop();

		headLight.fade(500);

		lastRecievedTime = millis();
	}

	/*if ( ! radio.isChipConnected() ) {
		RESET();
	}*/

	motorR.run(); motorL.run();

	

}