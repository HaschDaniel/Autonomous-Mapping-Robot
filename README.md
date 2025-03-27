# Autonomous Mapping Robot

The **Autonomous Mapping Robot** is my project for autonomous indoor environment mapping. It collects distances with ultrasonic sensors and sends the data to a server for 2D map generation using other application (not done yet).

## How It Works

1. **Ultrasonic sensors:** Robot uses three ultrasonic sensors (front, right, left) to measure distances,
2. **Data collection:** Robot is autonomously moving in the enviroment and collects the data,
3. **Communication:** Communication is done using MQTT, where it sends the data, so it does not save it in its own memory,
4. **Creating the map:** Collected data are visualized in other application (not done yet).

## Download

### Needed

- Python,
- Robot with ultrasonic sensors and a MQTT server. Car scheme: (not done yet)

### Steps

1. Clone the repository:

```bash
git clone https://github.com/HaschDaniel/autonomous-mapping-robot.git
```

2. Navigate to the project directory:

```bash
cd autonomous-mapping-robot
```

3. Install Python libraries required for the project and my libraries for L298N, HC-020K, HC-SR04 and QMC5883L. For MPU, I recommend finding one on GitHub.

4. Set your MQTT server IP, port, username and password in `secret` file:

```python
MQTT_SERVER=<server-ip>
MQTT_PORT=<port>
USERNAME=<username>   
PASSWORD=<password>   
```

if you want to use the username and password, edit lines from `477` to:
```python
client = MQTTClient(
        client_id="MAPPING_ROBOT_1",
        server=MQTT_SERVER,
        port=MQTT_PORT,
        user=USERNAME,
        password=PASSWORD,
        keepalive=60
    )
```

## Usage

1. Save the `main.py` file and required libraries into the robot,

2. The robot will begin moving autonomously, measure distances, and sending data to the MQTT server
(or you can use my app (not done yet)),

3. Open the mapping application to visualize the 2D map from the collected data. (not done yet)

## License

This project is open-source and available under the [Apache License 2.0](LICENSE). Feel free to use, modify, and distribute it as you want.
