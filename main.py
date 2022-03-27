import time
from Config import config

from Agent import Agent
from SmartAgent_v1 import SmartAgent_v1
from tests import Tests

if __name__ == "__main__":
    # drone_agent = Agent()
    # agent = None
    # if config.fly_with_full_lidar:
    #     agent = SmartAgent_v1()
    # else:
    #     agent = Agent()
    #
    # try:
    #     agent.connect_and_spawn()
    #     agent.fly_to_destination()
    # finally:
    #     agent.client.reset()
    # exit(1)

    tests = Tests()
    tests.run_tests()



