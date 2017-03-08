#ifndef GUI_CALLBACKS_H
#define GUI_CALLBACKS_H

#include <mcptam/ThresholdConfig.h>
#include <dynamic_reconfigure/server.h>

extern int level0;
extern int level1;
extern int level2;
extern int level3;
 
extern int threshold_state;

void threshold_gui_callback(mcptam::ThresholdConfig &config, uint32_t level);

#endif
