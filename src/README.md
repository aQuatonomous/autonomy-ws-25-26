# Competition message sending node

This node subscribes to various topics to get most of the data it needs to send the heartbeat and other required messages itself. 

Heartbeat message is automatically publised at 1 Hz.

It can be made to send other messages by publishing to topics as follows: 

### GatePass Message

A message about passing either the gate at the start of the course or in the speed challenge can be sent by sending a std_msgs/String.msg type message to the topic /messages/gatepass with the following contents: 
- data: start, end, speed_start, or speed_end

### ObjectDetetcted Message

A message about detecting an object can be sent by sending an ObjectDetection message (defined in this package) to the topic messages/object_detected with the following contents
- object_type: boat, light, or buoy
- colour: red, green, yellow, black
- latitude: the object's latitude
- longitude: the object's longitude
- object_id: a stable integer id for the object

### ObjectDelivery Message

A messsage about delivering water (the balls part of the spec is not implemented becuase the boat doesn't do that) to a target can be sent by sending a std_msgs/Integer.msg to the topic /messages/objec_delivered The contents of the message are not used becuase we are only going to deliver water to yellow boats this year. 

### Docking Message

A message about docking can be sent by sending a sending a Dock message (defined in this package) to the topic /messages/docking with the following contents:
- dock: "N" or "S" for the North or South dock
- slip: "1", "2", or "3" (as a string) for the dock number

### SoundSignal Message

A message about a sound signal interupt will automatically send if one is detetced.
