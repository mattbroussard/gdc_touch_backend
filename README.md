# About

This is the backend for the [gdc_touch](https://github.com/mattbroussard/gdc_touch) touch screen interface for GDC. This part will be communicated with via [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) websocket using [roslibjs](https://github.com/RobotWebTools/roslibjs) on the front end.

# Functionality

* More reliably pass messages back and forth (receipts are sent and expected; messages are retransmitted if not received).
* Accept event tracking messages and store the data in a local MySQL database.

# Customizations

Due to security reasons, the following will not be included, but may be necessary to run:

* SSL keys/certificates
* MySQL database credentials

There is a sample launchfile in launch/ showing how these parameters would need to be set.
