"""
Imagine you have to run multiple functions within a deadline. Running them sequentially will be the easiest thing to do,
but if one of them behaves badly and takes too much time, this might cause some trouble.
This code, instead of running functions sequentially, runs them all in separate threads in parallel. This allows to
consistently call the functions that meet their deadline in time, while ignoring those functions that miss the cycle
rate, yet still call them at their maximum rate.
Example:
             plugin1       plugin2         plugin3
start    ----------------------------------------------
              start        start           start

              finish

                           finish
loop rate ----------------------------------------------  (deadline)
              start again  start again
                                           finish
                                           start again
                                        .
                                        .
                                        .  etc...
In this example plugin 1 and 2 meet their deadline (and therefore are "in time"), while plugin3 takes more time than
the desired call frequency (misses the loop rate), yet it is still being called as fast as it can.
"""

import sys
import time
from threading import Thread


class FuncExecManager:
    """
    Class to keep track of synchronous execution of multiple functions with deadline.
    """

    def __init__(self, list_of_plugins, stop_condition, exec_after_each_loop, pause_execution, log_info=print,
                 log_warn=print, log_debug=print):
        """
        Constructor of a function execution manager.

        :param list_of_plugins: List of plugins that have an function called 'execute'
        :param stop_condition: Function that decides if simulation should be stopped or not
        :param exec_after_each_loop: Function to execute after each loop
        :param pause_execution: Function to pause the simulation
        :param log_info: Function for info logging
        :param log_warn: Function for warning logging
        :param log_debug: Function for debug logging
        :type list_of_plugins: list of object
        :type stop_condition: T
        :type exec_after_each_loop: T
        :type pause_execution: T
        :type log_info: T
        :type log_warn: T
        :type log_debug: T
        """
        self._on_time_functions = []
        self._late_functions = []
        self._late_threads = []
        self._is_loop_finished = False
        # to differentiate between each cycle (and keep track of functions that finish on time)
        self._cycle_unique_id = None
        self._list_of_plugins = list_of_plugins
        self._stop_condition = stop_condition
        self._exec_after_each_loop = exec_after_each_loop
        self._pause_execution = pause_execution

        self._log_warn = log_warn
        self._log_debug = log_debug
        log_info("[FuncExecManager::init] Started synchronous plugin execution manager.")

    def _time_control(self, cycle_unique_id, obj):
        class_name = str(obj.__class__)
        start_time = time.time()
        obj.execute()
        # compare id to check if we are still on time
        if self._cycle_unique_id == cycle_unique_id:
            self._log_debug("[FuncExecManager::time_control] Finished {class_name} in time")
            self._on_time_functions.append(obj)
        else:
            end_time = time.time()
            self._log_warn(
                f"[FuncExecManager::time_control] {class_name}: Missed loop rate, took "
                f"{str(round(end_time - start_time - (1.0 / self._loop_rate), 3))} sec longer than expected")
            self._late_functions.append(obj)

    def _loop_thread(self):
        time.sleep(1.0 / self._loop_rate)
        self._log_debug('==== loop rate! ====')
        self._is_loop_finished = True

    def start_synchronous_execution(self, loop_rate=0.25):
        self._loop_rate = loop_rate
        self._on_time_functions = self._list_of_plugins
        self._late_functions = []
        cycle_unique_id = 0

        while not self._stop_condition():
            cycle_unique_id = cycle_unique_id + 1
            if cycle_unique_id > sys.maxsize:
                cycle_unique_id = 0
            self._cycle_unique_id = cycle_unique_id
            self._is_loop_finished = False
            thread_list = list()

            # start all functions that are on time in a separate thread
            for func in self._on_time_functions:
                thread_list.append(Thread(target=self._time_control, args=(cycle_unique_id, func,)))
            self._on_time_functions = []
            for t in thread_list:
                t.start()

            # wait for one loop duration    
            Thread(target=self._loop_thread).start()
            while not self._is_loop_finished:
                # if there is a late thread that finished, run it again right away
                if self._late_functions:
                    self._late_threads = []
                    for func in self._late_functions:
                        self._late_threads.append(Thread(target=self._time_control, args=(cycle_unique_id, func,)))
                        self._late_threads[-1].start()
                    self._late_functions = []
                # sleep to reduce computational load
                time.sleep(0.0001)
                while self._pause_execution():
                    time.sleep(0.1)
            self._exec_after_each_loop()

        for thread in self._late_threads:
            thread.join()
