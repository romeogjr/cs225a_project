import redis
import json

KEY = 'sai::interfaces::tutorial::simple_data_structure'

values = {
    "string_key": "str",
    "number_key": 5.0,
    "bool_key": True,
}

redis_client = redis.Redis()
redis_client.set(KEY, json.dumps(values))
