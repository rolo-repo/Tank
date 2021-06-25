// SoundEffect.h

#ifndef _SOUNDEFFECT_h
#define _SOUNDEFFECT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

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
class SoundEffect {

public:
	virtual void begin() {}
	virtual void run() {}
	virtual void mute() {}
	virtual void unmute() {}
	virtual void play( SOUND_EFFECT_ID i_sound, bool i_is_mix = true ) {}
};

#endif

