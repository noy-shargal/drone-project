import time

from Agent import Agent

if __name__ == "__main__":
    drone_agent = Agent()
    agent = Agent()
    try:
        agent.connect_and_spawn()
        for i in range(10):
            agent.client.rotateByAngle(90, 5)
            time.sleep(5)
        agent.fly_to_destination()
    finally:
        agent.client.reset()
    exit(1)


