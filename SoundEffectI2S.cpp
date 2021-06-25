// 
// 
// 
#include "Constants.h"
#include "SoundEffectI2S.h"

#include "driver/i2s.h"

#include "0003idle_motor.h"

#include "BufferAndSize.h"
#define ENABLE_LOGGER
#include "SerialOutput.h"

#include <driver/dac.h>

struct {
    SOUND_EFFECT_ID id;
    int32_t rate;
    bool loop;
    BufferAndSize_t<unsigned char> track;
} soundEffects[] = { 
       { .id = SE_START_ENGINE , .rate = 44100  ,.loop = false , .track = BufferAndSize_t<unsigned char>(idle_motor,SIZE_OF_ARR(idle_motor)) },
       { .id = SE_IDLE ,         .rate = 16000  ,.loop = true ,  .track = BufferAndSize_t<unsigned char>(idle_motor,SIZE_OF_ARR(idle_motor)) },
       { .id = SE_MOVING ,       .rate = 44100  ,.loop = false , .track = BufferAndSize_t<unsigned char>(idle_motor,SIZE_OF_ARR(idle_motor)) },
       { .id = SE_STOP_ENGINE ,  .rate = 44100  ,.loop = false , .track = BufferAndSize_t<unsigned char>(idle_motor,SIZE_OF_ARR(idle_motor)) },
};


int i2s_num = 0; // i2s port number


int i2s_write_sample_nb( uint8_t sample ) {
    return i2s_write_bytes((i2s_port_t)i2s_num, (const unsigned char*)&sample, sizeof(uint8_t), 100);
}

/*
//Main function to play samples from PROGMEM
void playPROGMEMsample( const uint8_t* audioSample , uint32_t size ) {

    uint32_t sampleSize = sizeof(audioSample) * 4;

    uint32_t counter = 0;
    //initialize i2s with configurations above
    i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
    i2s_set_pin((i2s_port_t)i2s_num, 0);
    //set sample rates of i2s to sample rate of wav file
    i2s_set_sample_rates((i2s_port_t)i2s_num, 8000);


    uint8_t readData;
    uint8_t sendData;
    while (audioSample) {
        readData = pgm_read_word(&audioSample[counter++]);

        if (counter == size)
            break;
        //volume_control_changeVolume(&readData, &sendData, 2, 15);
        i2s_write_sample_nb(readData);
    }
    Serial.println("Stop");
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
}
*/


void SoundEffectI2S::begin( uint8_t i_volume )
{
    i_volume = constrain( i_volume, 0, 100 );

    static  i2s_config_t i2s_config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
            .sample_rate = 44100,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, /* the DAC module will only take the 8bits from MSB */
            .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,//I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
            .intr_alloc_flags = 0, // default interrupt priority
            .dma_buf_count = 8,
            .dma_buf_len = 96,
            .use_apll = 0
    };



    i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);

  //  i2s_set_pin((i2s_port_t)i2s_num, &pin_config);

   i2s_set_pin((i2s_port_t)i2s_num, 0);
   dac_output_disable((dac_channel_t)DAC_CHANNEL_2);
  
   m_volume = ( i_volume / 100.0 )  ;

   dac_output_voltage((dac_channel_t)DAC_CHANNEL_1, m_volume * 255.0 );
}

void SoundEffectI2S::stop(){
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
}

void SoundEffectI2S::run()
{
    static uint32_t counter = 0;

    if ( m_cSE == SE_LAST )
        return;

    //LOG_MSG("Playing " << (int)m_cSE << " " << soundEffects[m_cSE].track.m_size);

    i2s_write_sample_nb( soundEffects[m_cSE].track[counter++] * m_volume );

    if (soundEffects[m_cSE].loop)
        counter %= soundEffects[m_cSE].track.m_size;
    else
        m_cSE = SE_LAST;   
}


void SoundEffectI2S::play( SOUND_EFFECT_ID i_sound, bool i_is_mix )
{
    if ( m_cSE != i_sound && i_sound < SE_LAST ) {
        LOG_MSG("Starting to play " << (int)i_sound << " " << soundEffects[i_sound].track.m_size );
        m_timer = millis();
        m_cSE = i_sound;
        i2s_set_sample_rates( (i2s_port_t)i2s_num, soundEffects[i_sound].rate );
    }
}

void SoundEffectI2S::mute(bool sts){
   (sts)? dac_output_disable((dac_channel_t)DAC_CHANNEL_1) : dac_output_enable((dac_channel_t)DAC_CHANNEL_1);
}