# Ownership example for Neuron SDK

This is a simple ROS 2 example for ownership QoS, which is supported in Neuron SDK.
Ownership is often used for redundancy nodes.
There are two attributes: kind and strength.
If there are several publishers and their kind are `shared`, these publishers share the ownership to publishe messages.
If their kind are exclusive with different strength, subscriber will only receive messages from higher strength publisher.

## Shared ownership example

* subscriber side

```bash
ros2 run ownership ownership_sub -k shared
```

* publisher side

  - One publisher is shared ownership and it's ID is 100

```bash
ros2 run ownership ownership_pub -k shared -i 100
```

  - One publisher is shared ownership and it's ID is 200

```bash
ros2 run ownership ownership_pub -k shared -i 200
```

## Exclusive ownership example

* subscriber side

```bash
ros2 run ownership ownership_sub -k exclusive
```

* publisher side

  - One publisher is shared ownership and it's ID is 100

```bash
ros2 run ownership ownership_pub -k exclusive -i 100 -s 100
```

  - One publisher is shared ownership and it's ID is 200

```bash
ros2 run ownership ownership_pub -k exclusive -i 200 -s 200
```

Now you can see subscriber only can receive message from node id 200.
If you stop the publisher which node id is 200, you will see the publisher which node id is 100 hands over.
