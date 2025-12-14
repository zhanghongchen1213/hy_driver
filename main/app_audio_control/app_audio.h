#ifndef __APP_AUDIO_H__
#define __APP_AUDIO_H__

#include "all_include.h"
#include "esp_vfs.h"
#include "esp_http_client.h"
#include "freertos/ringbuf.h"
#include "audio_player.h"
#include <fcntl.h>

void rtos_audio_control_init(void);

#endif