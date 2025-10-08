from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from time import sleep

# Replace with ensp0 etc
NETWORK_DEVICE = "eth0"


def main():

    print("Starting")

    # Initialize communication channel
    print("Initializing communication channel...")
    ChannelFactoryInitialize(NETWORK_DEVICE)

    # Initialize the SportClient
    print("Initializing SportClient...")
    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # Stand
    sport_client.StandUp()
    sleep(2)

    # Sit
    sport_client.Sit()
    sleep(2)
    sport_client.RiseSit()  # TODO Is this the correct command?
    sleep(2)

    # Walk forward
    sport_client.Move(5.0, 0.0, 0.0)  # TODO What units are these?
    sleep(2)

    # Walk backward
    sport_client.Move(-5.0, 0.0, 0.0)
    sleep(2)

    # Adjust speed
    sport_client.SpeedLevel(123)  # TODO figure out correct values
    sport_client.Move(5.0, 0.0, 0.0)
    sleep(2)

    # Turn right
    sport_client.Euler(0.0, 0.0, 90)
    sleep(2)

    # Turn left
    sport_client.Euler(0.0, 0.0, -180)
    sleep(2)

    # Turn back to original
    sport_client.Euler(0.0, 0.0, 90)
    sleep(2)

    # Say hello
    sport_client.Hello()
    sleep(2)

    print("Finished")


if __name__ == "__main__":
    main()
