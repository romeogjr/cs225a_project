import redis
import math
import time

SCALAR_KEY = 'sai::interfaces::scalar_key'
VECTOR_KEY = 'sai::interfaces::vector_key'
WRITE_PERIOD = 0.01

r = redis.Redis()

print('Writing keys...')

t_start = time.time()
while True:
    t = time.time() - t_start 
    t_second = int(t // 1)
    t_fraction = t % 1

    # sawtooth from [0, 1] for scalar key
    if t_second % 2 == 0:
        r.set(SCALAR_KEY, t_fraction)
    else:
        r.set(SCALAR_KEY, 1 - t_fraction)

    # sine and cosine waves for vector keys
    r.set(VECTOR_KEY, str([math.sin(t), math.cos(t)]))

    time.sleep(WRITE_PERIOD)