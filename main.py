from Agent import Agent

if __name__ == "__main__":
    drone_agent = Agent()
    agent = Agent()
    try:
        agent.connect_and_spawn()
        agent.fly_to_destination()
    finally:
        agent.client.reset()
    exit(1)


