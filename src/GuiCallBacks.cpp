#include <mcptam/GuiCallBacks.h>

void threshold_gui_callback(mcptam::ThresholdConfig &config, uint32_t level)
{
    //ROS_ERROR_STREAM("threshold gui callback: state " << config.groups.thresh.state);

    level0 = config.level_0;
    level1 = config.level_1;
    level2 = config.level_2;
    level3 = config.level_3;

    threshold_state = config.groups.thresh.state;
} 
