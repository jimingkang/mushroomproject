# python 3.6

import random
import time

from paho.mqtt import client as mqtt_client



broker=''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)]
except:
    pass
broker=broker.replace("\n","").replace("\r\n","")
print(broker)

port = 1883
topic = "/flask/xyz"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 10000)}'
# username = 'emqx'
# password = 'public'


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            i=1
            #print("xyx publish Connected to MQTT Broker!")
        else:
            i=0
            #print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    print("connect broker",broker)
    client.connect(broker, port)
    return client



def publish(client,camera_xyz_list):
    #finalxyx = camera_xyz_list[0:len(camera_xyz_list) - 1]
    #print(finalxyx)
    publish_result = client.publish(topic, camera_xyz_list,qos=1)






def run(camera_xyz_list):
    client = connect_mqtt()
    client.loop_start()
    publish(client,camera_xyz_list)
    client.loop_stop()


if __name__ == '__main__':
    run('1,2,3;')
