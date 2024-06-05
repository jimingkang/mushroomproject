import redis

redis_server='192.168.254.26'
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)
r.delete("detections")
r.delete("detections_index_x")
r.delete("detections_index_y")
r.delete("global_camera_xy")
print(r.hgetall("detections"))
print(r.set("global_camera_xy","0,0"))
print(r.set("old_x","0"))
print(r.set("old_y","0"))
print(r.set("pre_trackid","0"))
print(r.get("old_x"))
print(r.get("old_y"))