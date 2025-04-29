import redis
import numpy as np 
import time

SCALAR_KEY = 'sai::interfaces::tutorial::scalar_key'
VECTOR_KEY = 'sai::interfaces::tutorial::vector_key'
MATRIX_KEY = 'sai::interfaces::tutorial::matrix_key'

r = redis.Redis()

print('Writing keys...')

while True:
    r.set(SCALAR_KEY, str(10 * np.random.random()))
    r.set(VECTOR_KEY, str((10 * np.random.random((4,))).tolist()))
    r.set(MATRIX_KEY, str((10 * np.random.random((4,4))).tolist()))
    time.sleep(0.1)
