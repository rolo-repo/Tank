// SoundEffectI2S.h

#ifndef _SOUNDEFFECTI2S_h
#define _SOUNDEFFECTI2S_h

#include "SoundEffect.h"

class SoundEffectI2S : public SoundEffect {

public:
	SoundEffectI2S() {
	
	}

	virtual void run();
	virtual void mute(bool sts = true);
	void play( SOUND_EFFECT_ID i_sound, bool i_is_mix = true );
	void begin( uint8_t i_volume = 10 );
	void stop();
private :
	uint64_t m_timer = 0;
	
	float m_volume = 1.0;

	SOUND_EFFECT_ID m_cSE = SOUND_EFFECT_ID::SE_LAST;
};
#endif

