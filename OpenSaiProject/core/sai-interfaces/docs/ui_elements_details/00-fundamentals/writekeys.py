import redis 

r = redis.Redis()
r.flushall()
r.set('sai::redis::apples', 1)
r.set('sai::redis::bananas', 2)
r.set('sai::redis::oranges', 3)
