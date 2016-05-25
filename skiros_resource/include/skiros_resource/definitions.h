#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

namespace skiros_resource
{
    namespace communication_state
    {
        enum states
        {
            NOT_ADVERTISED = -1,
            INITIALISATION = 0,
            ADVERTISED = 1,
            ADVERTISED_UNKNOWN = 2
        };
    }

    namespace operational_state
    {
        enum states
        {
            ERROR = -4,
            NOT_CONNECTED = -3,
            NOT_INITIALIZED = -2,
            STOPPED = -1,
            IDLE = 0,               //ready
            BUSY = 1

        };
    }
}
#endif /* DEFINITIONS_HPP */
