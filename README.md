# Multithreaded Drone-Hub Simulator
## Project Definiton
- This project is a simulation that mimicks Senders, Receivers, Drones, and Hubs. Senders would like to send packages for receivers. In order to do that, they need to give their package to hubs. Hubs are responsible to distribute packages to the destination hubs in which receivers will pick-up packages. However, each hub has limited drone park space. Also, there are limited drones in the system.
- This similator aims to mimic a living system with hubs and drones to achieve successfull delivery system.
## Programming Details
- Each drone and each hub is represented as a seperate thread. Each hub limited space for incoming and outgoing packages and also drones. One must request space before using it. This is accomplished by creating semaphore locks. 
## How to run
```
make all
.simulation < sample_input.txt
```
- This will compile the project, and run with sample input.
- Input must be given in the format below.
```
<number_of_hubs>
<size_of_incoming_package_storage_for_i> <size_of_outgoing_package_storage_for_i> <size_of_charging_spaces_for_drones_for_i> <list_distances_to_other_hubs>
<time_sender_i_waits_between_two_packets> <assigned_hub_id_of_sender_i> <total_number_of_packages_sender_i_will_send>
<time_receiver_i_waits_between_two_packets> <assigned_hub_id_of_receiver_i>
<travel_speed_of_the_drone_i> <starting_hub_id_of_drone_i> <max_range_of_drone_i>
```
