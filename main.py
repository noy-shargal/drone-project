from TangentBugAgent import TangentBugAgent

if __name__ == "__main__":
    drone_agent = TangentBugAgent()
    agent = TangentBugAgent()
    try:
        agent.connect_and_spawn()
        agent.fly_to_destination()
    finally:
        agent.client.reset()
    exit(1)


