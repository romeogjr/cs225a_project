from flask import Flask, jsonify, request, Response, send_file, send_from_directory
from flask_socketio import SocketIO
from redis_cache import RedisCache
from online_plot import OnlinePlotManager
import json
import click
import redis
import os
import sys
import subprocess

# determine full, absolute path to web/
static_folder_path = os.path.join(
    os.path.dirname(os.path.realpath(sys.argv[0])), 'web')

# bypass Flask templating engine by serving our HTML as static pages
example_to_serve = None
app = Flask(__name__, static_folder=static_folder_path, static_url_path='')
app.config['UPLOAD_FOLDER'] = '/tmp'
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
socketio = SocketIO(app)

#### global variables, initialized in server start ####
example_to_serve = None
redis_client = None
redis_cache = None
trajectory_runner = None

########### ROUTE HANDLING ################


@app.route('/')
def get_home():
    ''' Gets the home page "/", which is the example passed in the CLI. '''
    return send_file(example_to_serve)


@app.route('/<string:filename>', methods=['GET'])
def get_file(filename):
    ''' 
    Pulls a local file from either the launched directory or the static web
    folder. 
    '''
    cwd = os.getcwd()
    file_path_in_cwd = os.path.join(cwd, filename)

    # try looking in cwd. since it's the user's data, it has priority.
    # if the file isn't there, it might be in the web/ folder.
    if os.path.exists(file_path_in_cwd):
        return send_from_directory(cwd, filename)
    return send_from_directory(static_folder_path, filename)


@socketio.on('redis')
def handle_socket_redis_call(data):
    keys = json.loads(data)
    if type(keys) == list:
        return jsonify({key: redis_cache.key_cache.get(key) for key in keys})
    return jsonify({keys: redis_cache.key_cache.get(keys)})


@app.route('/redis', methods=['GET', 'POST'])
def handle_redis_call():
    ''' 
    Handles get/set of Redis keys. Supports multiple getting of keys.

    Front-end documentation:
    To get a key, GET /redis with { 'key': 'your_redis_key_here' }.
    To get multiple keys, query /redis with { 'key': ['key1', 'key2' , ...] }

    To set a key, POST to /redis with a JSON object { 'key': 'key1', 'val': val }.
        `val` must be a valid JSON object or string or number.
    '''
    if request.method == 'GET':
        key_list = json.loads(request.args.get('key'))
        if isinstance(key_list, str):
            response_json = json.dumps(redis_cache[key_list])
            return Response(response=response_json,
                            status=200,
                            mimetype='application/json')
        else:
            response_json = json.dumps(
                {key: redis_cache[key]
                 for key in key_list})
            return Response(response=response_json,
                            status=200,
                            mimetype='application/json')
    elif request.method == 'POST':
        data = request.get_json()
        if isinstance(data['val'], list):
            redis_client.set(data['key'], json.dumps(data['val']))
        else:
            redis_client.set(data['key'], data['val'])

        return Response(status=200)


@app.route('/redis/keys', methods=['GET'])
def handle_get_all_redis_keys():
    '''
    Gets all SAI redis keys.
    
    Frontend documentation:
    GET to /redis/keys
        > Response is a sorted JSON list of keys.
    '''
    keys = list(redis_cache.key_cache.keys())
    keys.sort()
    return jsonify(keys)


@app.route('/plot/offline', methods=['POST'])
def trigger_open_csv_plotter():
    csv_plotter_path = static_folder_path + '/../csv_plotter.py'
    subprocess.Popen(['python', csv_plotter_path])
    return Response(status=200)


@app.route('/plot/start', methods=['POST'])
def handle_plot_start():
    data = request.get_json()
    keys = data['keys']
    rate = data['rate']
    return jsonify({'plot_id': plot_manager.start_plot(keys, rate)})


@app.route('/plot/data', methods=['POST'])
def handle_plot_get():
    plot_id = request.get_json()['plot_id']
    avail = plot_manager.get_available_data_from_plot(plot_id)
    return jsonify({
        'plot_id': plot_id,
        'running': plot_manager.is_plot_running(plot_id),
        'data': avail
    })


@app.route('/plot/stop', methods=['POST'])
def handle_plot_stop():
    plot_id = request.get_json()['plot_id']
    if plot_manager.stop_plot(plot_id):
        return Response(status=200)
    else:
        return Response(status=500)


############ CLI + Server Init ##############
@click.command()
@click.option("-hp",
              "--http_port",
              help="HTTP Port (default: 8000)",
              default=8000,
              type=click.INT)
@click.option("-rh",
              "--redis_host",
              help="Redis hostname (default: localhost)",
              default="localhost",
              type=click.STRING)
@click.option("-rp",
              "--redis_port",
              help="Redis port (default: 6379)",
              default=6379,
              type=click.INT)
@click.option("-rd",
              "--redis_db",
              help="Redis database number (default: 0)",
              default=0,
              type=click.INT)
@click.option("-rate",
              "--cache-refresh-rate",
              help="How often to load keys from Redis (default: 0.1)",
              default=0.1,
              type=click.FLOAT)
@click.argument('example',
                type=click.Path(exists=True,
                                file_okay=True,
                                dir_okay=False,
                                readable=True))
def server(http_port, redis_host, redis_port, redis_db, cache_refresh_rate,
           example):
    global redis_client, redis_cache, example_to_serve, plot_manager
    example_to_serve = os.path.realpath(example)
    redis_client = redis.Redis(host=redis_host,
                               port=redis_port,
                               db=redis_db,
                               decode_responses=True)
    redis_cache = RedisCache(redis_client, refresh_rate=cache_refresh_rate)
    plot_manager = OnlinePlotManager(redis_cache)

    redis_cache.start()
    socketio.run(app, port=http_port, debug=True)


if __name__ == "__main__":
    server()
