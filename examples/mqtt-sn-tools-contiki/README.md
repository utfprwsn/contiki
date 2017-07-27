Adaptation of mqtt-sn tools to be used on Contiki devices.

Supported Features
------------------

- QoS 0 and -1
- Keep alive pings
- Publishing retained messages
- Publishing empty messages
- Subscribing to named topic
- Clean / unclean sessions
- Manual and automatic client ID generation
- Displaying topic name with wildcard subscriptions
- Pre-defined topic IDs and short topic names


Limitations
-----------

- Currently there is no mqtt-sn gateway that supports IPv6, use NAT64
- Packets must be 255 or less bytes long
- No Last Will and Testament
- No QoS 1 or 2
- No Automatic gateway discovery


Building
--------

Standard Contiki build instructions apply


Example Client
--------------
The example client demonstrates a client connects, registers a topic, 
published a topic, and subscribes to a topic.

It used the first byte in the recieved payload of the subscription topic
to change the interval (0 to 256 seconds) between publishing messages.


License
-------

MQTT-SN Tools is licensed under the [MIT License].



[MIT License]: http://opensource.org/licenses/MIT
