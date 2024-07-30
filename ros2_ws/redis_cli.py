import redis
redis_server='172.27.34.65'
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)
print(r.get("mode"))
print(r.get("global_camera_xy"))
print(r.set("mode","camera_ready"))
