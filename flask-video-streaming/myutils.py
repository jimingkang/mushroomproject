
def getbroker():
    broker = ''
    redis_server = ''
    try:
        for line in open("../ip.txt"):
            if line[0:6] == "broker":
                broker = line[9:len(line) - 1]
            if line[0:6] == "reddis":
                redis_server = line[9:len(line) - 1]
    except:
        pass
    print(broker)
    print(redis_server)
    return (broker,redis_server)