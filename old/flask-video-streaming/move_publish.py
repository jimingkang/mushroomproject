# python 3.6

import random
import time

from paho.mqtt import client as mqtt_client



broker=''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)-1]
except:
    pass

#broker = "172.26.52.46"
port = 1883
topic = "/flask/serial"
print("BROKER: ", broker)
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 10000)}'
# username = 'emqx'
# password = 'public'


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            i=1
            #print("move publish Connected to MQTT Broker!")
        else:
            i=0
            #print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client



def publish(client,camera_xyz_list):
    publish_result = client.publish(topic, camera_xyz_list,qos=0)






def run(camera_xyz_list):
    client = connect_mqtt()
    client.loop_start()
    publish(client,camera_xyz_list)
    client.loop_stop()


if __name__ == '__main__':
    run('1,2,3;')
