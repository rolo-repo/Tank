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

BTS7960_1PWM   motorR( motorRFrdPin, motorRBwdPin, motorRSpdPin, []() { backLight.turn_on(Led::Brightness::_100); });
BTS7960_1PWM   motorL( motorLFrdPin, motorLBwdPin, motorLSpdPin, []() { backLight.turn_on(Led::Brightness::_100); });

//RF24 radio(_SS , _SCN );
//ce, csn
RF24 radio( _SS , _SCN, _SCK, _MISO, _MOSI );

//26,13,15,12,23,27,28,9,10,11,7,8

void setup()
{
	//LOG_MSG_BEGIN(115200);
	delay(1000);

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

	delay(1000);
	//analogWriteFrequency(1000);
	//analogWriteResolution(8);
	/*ledcSetup( 0, 1000, 8 );
	ledcAttachPin( motorRSpdPin, 0 );*/
	
	motorR.begin(0);
	motorL.begin(1);

	LOG_MSG("Ready");
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
int32_t computePID( float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut ) {
	float err = setpoint - input;
	static float integral = 0, prevErr = 0;
	integral = constrain( integral + (float)err * dt * ki, minOut, maxOut );
	float D = (err - prevErr) / dt;
	prevErr = err;
	return constrain( err * kp + integral + D * kd, minOut, maxOut );
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


void loop()
{
	using namespace arduino::utils;
	
	byte pipeNo;
	static unsigned long lastRecievedTime = millis();
	static Payload recieved_data;

	static int i = 3000;
	float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
	float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
	int     nPivSpeed;      // Pivot Speed                          (-128..+127)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
	float   fPivYLimit = 10.0;



	// INPUTS
	int     nJoyX;              // Joystick X input                     (-128..+127)
	int     nJoyY;              // Joystick Y input                     (-128..+127)

	// OUTPUTS
	int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
	int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

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

		nJoyY = recieved_data.m_speed;
		nJoyX = recieved_data.m_steering;

		// Calculate Drive Turn output due to Joystick X input
		if ( nJoyY >= 0 ) {
			// Forward
			nMotPremixL = ( nJoyX >= 0 ) ? 127.0 : (127.0 + nJoyX);
			nMotPremixR = ( nJoyX >= 0 ) ? (127.0 - nJoyX) : 127.0;
		}
		else {
			// Reverse
			nMotPremixL = ( nJoyX >= 0 ) ? (127.0 - nJoyX) : 127.0;
			nMotPremixR = ( nJoyX >= 0 ) ? 127.0 : (127.0 + nJoyX);
		}

		// Scale Drive output due to Joystick Y input (throttle)
		nMotPremixL = nMotPremixL * nJoyY / 128.0;
		nMotPremixR = nMotPremixR * nJoyY / 128.0;

		// Now calculate pivot amount
		// - Strength of pivot (nPivSpeed) based on Joystick X input
		// - Blending of pivot vs drive (fPivScale) based on Joystick Y input
		nPivSpeed = nJoyX;
		fPivScale = ( abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit );

		// Calculate final mix of Drive and Pivot
		nMotMixL = (1.0 - fPivScale)*nMotPremixL + fPivScale * (nPivSpeed);
		nMotMixR = (1.0 - fPivScale)*nMotPremixR + fPivScale * (-nPivSpeed);

		(nMotMixL > 0) ? motorL.backward( map( nMotMixL, 0, 127, 0, 255 )) 
			: motorL.forward(map(nMotMixL, -127, 0, 255, 0));
		(nMotMixR > 0) ? motorR.backward( map( nMotMixR, 0, 127, 0, 255)) 
			: motorR.forward(map(nMotMixR, -127, 0, 255, 0));
		
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
		payLoadAck.speed = 10;
		payLoadAck.batteryLevel = 10;
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