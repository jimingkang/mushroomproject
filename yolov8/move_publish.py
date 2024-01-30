# python 3.6

import random
import time

from paho.mqtt import client as mqtt_client


broker=''
try:
    for line in open("ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:-1]
except:
    pass

broker = '192.168.254.42'
#broker = '10.0.0.134'
port = 1883
tpoic_flask_serial = "/flask/serial"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
# username = 'emqx'
# password = 'public'


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            i=1
            print("get serial publish Connected to MQTT Broker!")
        else:
            i=0
            #print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client,topic,cmd):
    publish_result = client.publish(topic, cmd,qos=1,retain=True)
    cmd= ""
    return publish_result





def run(topic,cmd):
    client = connect_mqtt()
    client.loop_start()
    ret=publish(client,topic,cmd)
    client.loop_stop()
    return ret


if __name__ == '__main__':
    run('G21 G91 G1 X134 F2540')
