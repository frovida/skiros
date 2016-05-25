#ifndef COMMON_DEF_H
#define COMMON_DEF_H

namespace skiros_primitive
{
    //Primitive system topics
    static const char STATUS_TOPIC[] = "monitor/status";
    static const char EVENT_TOPIC[] = "monitor/event";
    static const char TRIGGER_TOPIC[] = "trigger";
    static const char SET_CONFIG_TOPIC[] = "set_config";
    static const char GET_CONFIG_TOPIC[] = "get_config";
    static const char SCHED_START_TOPIC[] = "scheduler/start_stop";
    static const char SCHED_SET_TOPIC[] = "scheduler/set_scheduling";
    //Primitive events
    static const char CONFIG_EVENT[] = "configured";
    static const char BAD_CONFIG_EVENT[] = "badConfigured";

}
#endif // COMMON_DEF_H
