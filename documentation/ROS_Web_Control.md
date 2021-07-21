# ROS Worker Control #

The platform includes the ability to control a ROS based robot using a joystick in the browser and navigation by setting goals from a map visualized in the browser.   you will need to setup secure websocket connection certification files on your robot and refer to them in the `rosbridge_server rosbridge_websocket.launch`
![image info](./documentation/pictures/![image info](./documentation/pictures/2.png))

Here are the the steps to configuring the key files and cert files:
- openssl genrsa -out server.key 2048
- openssl rsa -in server.key -out server.key
- openssl req -sha256 -new -key server.key -out server.csr -subj '/CN=localhost'
> - replace 'localhost' with the url you are using which normally is the ipaddress:port.  For example: `192.168.2.27:9090`
- openssl x509 -req -sha256 -days 365 -in server.csr -signkey server.key -out server.crt
