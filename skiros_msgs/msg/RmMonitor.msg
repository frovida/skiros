## Msg of the device manager monitor
## Event type constants
##
byte STARTED=1
byte TERMINATED=2
byte PAUSED=4
byte CONTINUED=8
##
## Fields
##
Header header
byte event_type
string resource_address
#string driver
