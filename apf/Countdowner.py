import time


class Countdowner:
    def __init__(self, countdown):
        self._countdown_time = countdown
        self._start_time = None

    def start(self):
        self._start_time = time.time()

    def running(self):
        return time.time() - self._start_time < self._countdown_time
