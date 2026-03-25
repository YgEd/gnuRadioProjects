

# GFSK Drone TX to GCS RX Simulator Demo

## This demo illustrates the functionality of this project where we fully simulate a drone passing messages over the air (OTA) to a ground control station (GCS). This simulator is constructed from all the code in this repository including packet construction methods, gnuradio SDR modules for TX and RX, metric collection and log formation, db writing, and GCS streaming. 
## This simulator allows us to control and collect detailed parameters at every step of the communication system from lower level rf data such as packet construction, SNR, jitter, modulation scheme, to higher level metrics such as drone state, latency, etc. The simulator is build in a modular fashion allowing the simulator to be drone, comms protocol, and modulation scheme agnostic allowing us to simulate any drone to GCS communication system. Eventually the simulator will feed into BNM models described [here](../stat_models) to help us understand casual relationships between physical medium rf changes and higher level cyber states and metrics of the drone.


### Picture of the Setup

![IMG_7002](https://github.com/user-attachments/assets/587a406e-570f-4242-a348-864cf8a4bc93)

The setup includes three machines:
1. Machine 1 (Center) runs the simulator drone and is connected to the TX SDR
2. Machine 2 (Furthest Left) connected to the RX SDR receives drone messages OTA and broadcasts messages to network
3. Machine 3 (Furthest Right) runs the GCS and receives drone messages broadcasted over the network from machine 2


### Below is a short video demo fo the simulator:

<video src="https://github.com/user-attachments/assets/10d3e058-0c3c-4e0d-8478-949b53aaf728" width="600" controls></video>




