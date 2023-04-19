from pythonosc import udp_client
from sys import argv
import random
import time


def send_rand(client):
  vals = [random.random() * 5 for _ in range(3)]
  
  client.send_message("/controls", vals)
  client.send_message("/notcontrols", "hello there")

def main():
  host = argv[1]
  port = int(argv[2])
  client = udp_client.SimpleUDPClient(host, port)
  for _ in range(10):
    send_rand(client)
    print("sending...")
    time.sleep(1)

if __name__ == "__main__":
  main()