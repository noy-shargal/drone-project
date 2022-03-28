import time
from typing import Tuple

from shapely.geometry import Point

from Config import config
from SmartAgent_v1 import SmartAgent_v1
import logging
from timeit import default_timer as timer



class Test:
    def __init__(self, source:Point, destination: Point, height):
        self._source = source
        self._destination = destination
        self._height = height

class Tests:

    def __init__(self):

        self.test_pathes = list()

        logging.basicConfig(filename='tests.log', level=logging.DEBUG)

        #self.add_test((-1200.0, -1200), (0.0, -600.0), -50)
        self.add_test((-1000.0, -1000), (-830, -900), -50)

        # self.add_test((0.0, -600.0, -50), (-1200.0, -1200, -50))
        #
        # self.add_test((0.0, -1200.0, -50), (-1200.0, -1300, -50))
        # self.add_test((-1200.0, -1300, -50),(0.0, -1200.0, -50))
        #
        # self.add_test((-1200, -850.0, -50), (0, -850, -50))
        # self.add_test((0, -850, -50), (-1200, -850.0, -50))
        #
        # self.add_test((-600, -1200.0, -50), (-600, -400, -50))
        # self.add_test( (-600, -400, -50), (-600, -1200.0, -50))



    def add_test(self,  src:Tuple, dst:Tuple, height):
        source = Point(*src)
        destination = Point(*dst)
        self.test_pathes.append(Test (source, destination, height))


    def run_tests(self):

        i = 1
        for test in self.test_pathes:
            logging.info("Running Test number:  "+str(i))
            i += 1
            self.run_test(test)

    def run_test(self, test: Test):

        config.source = test._source
        config.destination = test._destination
        config.height = test._height

        agent = SmartAgent_v1()
        try:
            agent.connect_and_spawn()
            logging.info("start path from: " + str(config.source) + " -> " + str(config.destination))
            start = timer()

            agent.fly_to_destination()
            end = timer()
            elapsed_time = end - start
            logging.info("Finished path from: " + str(config.source) + " -> " + str(config.destination) + "at time: " + str(elapsed_time))
        finally:
            agent.client.reset()



