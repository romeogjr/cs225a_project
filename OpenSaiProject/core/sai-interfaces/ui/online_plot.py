import time
import uuid
import queue
from threading import Thread

TIME_KEY = "Time"


class OnlinePlot(object):

    def __init__(self, redis_cache):
        self.id = str(uuid.uuid4())
        self.redis_cache = redis_cache
        self.running = False
        self.queue = queue.Queue()

    def _gather_plot_data(self, window_size=10):
        start_time = time.time()

        while self.running:
            current_time = time.time() - start_time
            keys = self.keys.copy()
            if TIME_KEY in keys:
                keys.remove(TIME_KEY)
            values = [self.redis_cache[key] for key in keys]
            queue_item = {
                TIME_KEY: current_time
            } if TIME_KEY in self.keys else {}
            for key, val in zip(keys, values):
                queue_item[key] = val

            self.queue.put_nowait(queue_item)

            if self.queue.qsize() > window_size:
                self.queue.get_nowait()

            time.sleep(self.rate)

    def get_available(self):
        if self.queue.empty():
            return []
        items = []
        while not self.queue.empty() and len(items) < 250:
            item = self.queue.get_nowait()
            if item is None:
                break
            items.append(item)
            self.queue.task_done()
        return items

    def start(self, keys, rate):
        if self.running:
            return False

        self.rate = rate
        self.keys = keys

        # start worker thread
        self.running = True
        self.thread = Thread(target=self._gather_plot_data,
                             daemon=True)  # XXX: may not clean up correctly
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        self.queue = queue.Queue()  # empty the old queue


class OnlinePlotManager(object):

    def __init__(self, redis_client):
        self._plots = {}
        self.redis_client = redis_client

    def start_plot(self, keys, rate):
        plot = OnlinePlot(self.redis_client)
        self._plots[plot.id] = plot
        plot.start(keys, rate)
        return plot.id

    def is_plot_running(self, plot_id):
        if plot_id not in self._plots:
            return False
        else:
            return self._plots[plot_id].running

    def get_available_data_from_plot(self, plot_id):
        if plot_id not in self._plots:
            return []
        else:
            data = self._plots[plot_id].get_available()
            return data

    def stop_plot(self, plot_id):
        if plot_id not in self._plots:
            return False

        plot = self._plots[plot_id]
        if plot.running:
            plot.stop()
        return True
