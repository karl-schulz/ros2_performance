import time
from time import sleep


class Interval:
    """
    Helper for stuff that you want to be executed only once in a while.
    Example:

        if interval.tick():
            do_some_rare_task()

    """
    def __init__(self, seconds):
        self._seconds = seconds
        self._last_reset = time.time()

    @property
    def seconds(self):
        return self._seconds

    def remaining(self):
        return self._seconds + self._last_reset - time.time()

    def wait(self) -> "Interval":
        sleep(max(0, self.remaining()))
        return self

    def expired(self):
        if self._seconds is None:
            return
        return time.time() > self._seconds + self._last_reset

    def reset(self):
        self._last_reset = time.time()

    def untick(self):
        self._last_reset = time.time() - self._seconds

    def tick(self):
        if self._seconds is None:
            return False
        if self.expired():
            self.reset()
            return True
        else:
            return False

