import romer.romer_publishers.src.camera_cartesian as client

client.initialize()
pose = client.get_cart()
print(pose)
client.finalize()