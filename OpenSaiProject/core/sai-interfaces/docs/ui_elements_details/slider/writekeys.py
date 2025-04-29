import redis 

r = redis.Redis()
r.set('sai::interfaces::tutorial::scalar_key', '5')
r.set('sai::interfaces::tutorial::vector_key', '[3,4]')
