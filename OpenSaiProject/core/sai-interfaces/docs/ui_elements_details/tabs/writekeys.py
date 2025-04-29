import redis 

r = redis.Redis()
r.set('sai::interfaces::tutorial::scalar_key', '1')
r.set('sai::interfaces::tutorial::vector_key', '[2,3]')
