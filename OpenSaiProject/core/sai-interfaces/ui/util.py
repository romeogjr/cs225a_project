import json


def get_redis_key(key, redis_client):
    ''' 
    Retrieves a key from redis and attempts JSON parsing.
    We don't want to send a JSON object hiding in a string to the
    frontend, who would then be forced to double unwrap.
    '''
    return try_parse_json(redis_client.get(key))


def try_parse_json(s):
    try:
        return json.loads(s)
    except:
        return s
