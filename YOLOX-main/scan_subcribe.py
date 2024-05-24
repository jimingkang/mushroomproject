# python3.6

import random
import subprocess
from paho.mqtt import client as mqtt_client


broker = '192.168.254.42'
port = 1883
topic = "/flask/scan"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    #client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        subprocess.call("C:/Users/jkang7/Downloads/uh/project/MUSHROOM/YOLOX-main/venv/Scripts/python.exe demo_ip.py webcam -f ./exps/example/yolox_voc/yolox_voc_s.py --trt --conf 0.25 --nms 0.45 --tsize 640 --save_result ")
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()