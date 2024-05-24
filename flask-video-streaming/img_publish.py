# python 3.6

import random
import time

from paho.mqtt import client as mqtt_client
import xyz_publish

#broker = '192.168.254.18'
broker = '10.0.0.133'
port = 1883
topic = "/flask/img"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
# username = 'emqx'
# password = 'public'

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            i=1
            #print("img publish Connected to MQTT Broker!")
        else:
            i=0
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client):
    with open("./image/detection.jpg", 'rb') as file:
        filecontent = file.read()
        byteArr = bytearray(filecontent)
        #print(byteArr)
        publish_result = client.publish(topic, byteArr,2)
        msg_status = publish_result[0]
        if msg_status == 0:
            print(f"message sent to topic {topic}")
        else:
            print(f"Failed to send message to topic {topic}")




def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)


if __name__ == '__main__':
    run()
