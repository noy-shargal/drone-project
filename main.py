from Agent import Agent
import config


if __name__ == '__main__':
    current_config = config.finite_repulsion_config1
    agent = Agent()
    agent._drone_client.reset()
    agent.connect_and_spawn()
    try:
        agent.fly_to_destination()
    finally:
        agent._drone_client.reset()
    exit(1)
