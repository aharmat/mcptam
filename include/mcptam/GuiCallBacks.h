#ifndef GUI_CALLBACKS_H
#define GUI_CALLBACKS_H

#include <mcptam/ThresholdConfig.h>
#include <dynamic_reconfigure/server.h>

int level0 = 10;
int level1 = 15;
int level2 = 15;
int level3 = 10;

int threshold_state = 0;

void threshold_gui_callback(mcptam::ThresholdConfig &config, uint32_t level);

#endif
