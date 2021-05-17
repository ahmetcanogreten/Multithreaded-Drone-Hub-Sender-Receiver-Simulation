#include "simulation.h"

#define DEBUG 1

sem_t active_hubs_mutex;
int active_hubs;

sem_t active_senders_mutex;
int active_senders;

int num_of_hubs;
int num_of_drones;

Hub *hubs;
Drone *drones;
Sender *senders;
Receiver *receivers;

pthread_t *tid;
pthread_attr_t attr;

void initialize_hubs();
void initialize_senders();
void initialize_receivers();
void initialize_drones();
void initialize_world();
void destroy_world();

void* hub_thread();
void* sender_thread();
void* receiver_thread();
void* drone_thread();



int main(int argc, char *argv[]){
    int i = 0, j = 0;
    int total_received, total_delivered, total_sent, total_assigned;
    total_received = total_delivered = total_sent = total_assigned = 0;
    InitWriteOutput();
    initialize_world();

    pthread_attr_init(&attr);

    for(i = 0; i < num_of_hubs; i++){
        int *id = malloc(sizeof(int));
        *id = i;
        pthread_create(&tid[j++], &attr, hub_thread, (void *) id);
    }
    for(i = 0; i < num_of_hubs; i++){
        int *id = malloc(sizeof(int));
        *id = i;
        pthread_create(&tid[j++], &attr, sender_thread, (void *) id);
    }
    for(i = 0; i < num_of_hubs; i++){
        int *id = malloc(sizeof(int));
        *id = i;
        pthread_create(&tid[j++], &attr, receiver_thread, (void *) id);
    }
    for(i = 0; i < num_of_drones; i++){
        int *id = malloc(sizeof(int));
        *id = i;
        pthread_create(&tid[j++], &attr, drone_thread, (void *) id);
    }
    for(i = 0; i < (num_of_hubs) * 3 + num_of_drones; i++){
        pthread_join(tid[i], NULL);
    }

    

    for(i = 0; i < num_of_hubs; i++){
        total_assigned += hubs[i].total_assigned;
        total_received += receivers[i].total_received;
        total_sent += senders[i].total_sent;
    }
    for(i = 0; i < num_of_drones; i++){
        total_delivered += drones[i].total_delivered;
    }

    destroy_world();

    if(DEBUG){
        printf("TOTAL ASSIGNED : %d\n", total_assigned);
        printf("TOTAL SENT : %d\n", total_sent);
        printf("TOTAL RECEIVED : %d\n", total_received);
        printf("TOTAL DELIVERED : %d\n", total_received);  
    }
     


    return 0;
}

//**************** COMMON START ********************

int wait_for_one_sec(sem_t *sem){
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec++;   

    return sem_timedwait(sem, &ts);
}

void park_drone(Drone *drone, int hub_id){
    ChargingSpaceNode *current = hubs[hub_id - 1].charging_space.dummy;
    ChargingSpaceNode *new;
    while(current->next && drone->current_range < current->next->drone->current_range) current = current->next;
    new = malloc(sizeof(ChargingSpaceNode));
    new->drone = drone;
    new->next = current->next;
    current->next = new;
}


int is_any_hub_active(){
    int is_active = 0;
    sem_wait(&active_hubs_mutex);
        is_active = active_hubs;
    sem_post(&active_hubs_mutex);
    return is_active;
}

// *************** COMMON STOP  *********************

// *************** SENDER START ********************

int select_random_hub(int id){
    int to = (rand() % num_of_hubs) + 1;
    while(receivers[to -1].assigned_hub_id == senders[id - 1].assigned_hub_id) to = (rand() % num_of_hubs) + 1;
    return to;
}

void deposit_package(Hub *hub, PackageInfo *package){
    StorageNode *current;
    sem_wait(&hub->outgoing_storage_mutex);
        current = hub->outgoing_storage.dummy;
        while(current->next) current = current->next;

        current->next = malloc(sizeof(StorageNode));
        current->next->next = NULL;
        current->next->package = package;
    sem_post(&hub->outgoing_storage_mutex);
}

void* sender_thread(int *index){
    Sender *this = &senders[*index];
    Hub *my_hub = &hubs[this->assigned_hub_id - 1];

    SenderInfo sender_info;
    FillSenderInfo(&sender_info, *index + 1, this->assigned_hub_id, this->remaining_packages, NULL);
    WriteOutput(&sender_info, NULL, NULL, NULL, SENDER_CREATED);


    while(this->remaining_packages > 0) {
        int to = select_random_hub(*index+1);
        PackageInfo *sent = malloc(sizeof(PackageInfo));
        FillPacketInfo(sent, *index + 1, this->assigned_hub_id, to, receivers[to-1].assigned_hub_id);
        
        sem_wait(&my_hub->outgoing_storage.empty);   // Wait for an empty space
            deposit_package(my_hub, sent);
        sem_post(&my_hub->outgoing_storage.full); 
        
        FillSenderInfo(&sender_info, *index + 1, this->assigned_hub_id, this->remaining_packages, sent);
        WriteOutput(&sender_info, NULL, NULL, NULL, SENDER_DEPOSITED);

        this->remaining_packages--;
        this->total_sent++;
        wait(this->wait_time * UNIT_TIME);   
        //printf("SENDER : %d, REM : %d\n", *index+1, this->remaining_packages); 

    }
    sem_wait(&active_senders_mutex);
        active_senders--;
    sem_post(&active_senders_mutex);

    FillSenderInfo(&sender_info, *index + 1, this->assigned_hub_id, this->remaining_packages, NULL);
    WriteOutput(&sender_info, NULL, NULL, NULL, SENDER_STOPPED);

    //printf("SENDER DONE : %d, TOTAL SENT : %d\n", *index + 1, this->total_sent);
    free(index);
    pthread_exit(0);
}

//**************** SENDER STOP  ********************

// ************ RECEIVER START *********************

PackageInfo* receive_package(int receiver_id){
    StorageNode *current = hubs[receivers[receiver_id - 1].assigned_hub_id - 1].incoming_storage.dummy;
    PackageInfo *package;
    StorageNode *tmp;
    while(current->next){
        package = current->next->package;
        if(package->receiver_id == receiver_id){
            tmp = current->next;
            current->next = tmp->next;
            free(tmp);
            sem_post(&hubs[receivers[receiver_id - 1].assigned_hub_id - 1].incoming_storage.empty);
            return package;
        }
        current = current->next;
    }

}

void* receiver_thread(int *index){
    Receiver *this = &receivers[*index];
    Hub *my_hub = &hubs[*index];
    PackageInfo *received;

    ReceiverInfo receiver_info;
    FillReceiverInfo(&receiver_info, *index + 1, *index + 1, NULL);
    WriteOutput(NULL, &receiver_info, NULL, NULL, RECEIVER_CREATED);

    while(is_any_hub_active()){
        if(wait_for_one_sec(&this->have_package)){
            //printf("TIMEOUT HAVE PACKAGE RECEIVER : %d\n", *index+1);
            continue;
        }
        sem_wait(&hubs[this->assigned_hub_id - 1].incoming_storage_mutex);
            received = receive_package(*index+1);        
        sem_post(&hubs[this->assigned_hub_id - 1].incoming_storage_mutex);
        
        FillReceiverInfo(&receiver_info, *index+1, this->assigned_hub_id, received);
        WriteOutput(NULL, &receiver_info, NULL, NULL, RECEIVER_PICKUP);
        free(received);
        this->total_received++;
        wait(this->wait_time * UNIT_TIME);

    }
    //printf("RECEIVER DONE : %d, TOTAL RECEIVED : %d\n", *index + 1, this->total_received);
    FillReceiverInfo(&receiver_info, *index + 1, *index + 1, NULL);
    WriteOutput(NULL, &receiver_info, NULL, NULL, RECEIVER_STOPPED);
    free(index);
    pthread_exit(0);
}

// ************ RECEIVER STOP  *********************

// ************* DRONE START ************************

int find_distance(int from_id, int to_id){
    DistanceNode *current = hubs[from_id - 1].distance_dummy->next;
    while(current->id != to_id) current = current->next;
    return current->distance;
}

void* drone_thread(int *index){
    Drone *this = &drones[*index];

    DroneInfo drone_info;
    FillDroneInfo(&drone_info, *index + 1, this->current_hub_id, this->current_range, NULL, 0);
    WriteOutput(NULL, NULL, &drone_info, NULL, DRONE_CREATED);
    while(is_any_hub_active()){
        long long start;
        int travel_time;
        DroneOrderNode *tmp_order;
        int result = 0;
        //printf("DRONE WAITING FOR A JOB ID : %d\n", *index + 1);
        
        
        
        result = wait_for_one_sec(&this->order_ready);
        if(result == -1) {
            //printf("DRONE WAITING FOR A JOB AGAIN\n");
            continue;
        }
        else {
            ;
            //printf("DRONE RECEIVED JOB\n");
        }   
        tmp_order = this->order_dummy->next;
        switch(tmp_order->type){
            case DRONE_DELIVER_PACKAGE:
                
                travel_time = find_distance(this->order_package->sending_hub_id, this->order_package->receiving_hub_id);
                
                //printf("DRONE DELIVERY JOB RECEIVED\n");
                sem_wait(&hubs[this->order_package->receiving_hub_id - 1].charging_space.empty);
                //printf("DRONE DELIVERY JOB CHARGING RESERVED\n");

                sem_wait(&hubs[this->order_package->receiving_hub_id - 1].incoming_storage.empty);
                //printf("DRONE DELIVERY JOB INCOMING RESERVED\n");
                
                this->current_range = calculate_drone_charge(timeInMilliseconds() - this->last_timestamp, this->current_range, this->max_range);

                if(this->current_range < travel_time){
                    wait((travel_time - this->current_range) * UNIT_TIME);
                }

                sem_post(&hubs[this->current_hub_id - 1].charging_space.empty);
                sem_post(&hubs[this->current_hub_id - 1].outgoing_storage.empty);

                //printf("DRONE DELIVERY JOB FLEW\n");

                travel(travel_time, this->speed);
                this->current_range -= range_decrease(travel_time, this->speed);
                //printf("DRONE DELIVERY JOB ARRIVED\n");

                this->current_hub_id = this->order_package->receiving_hub_id;
                sem_wait(&hubs[this->order_package->receiving_hub_id - 1].incoming_storage_mutex);
                    StorageNode *new = malloc(sizeof(StorageNode));
                    new->package = this->order_package;
                    new->next = hubs[this->order_package->receiving_hub_id - 1].incoming_storage.dummy->next;
                    hubs[this->order_package->receiving_hub_id - 1].incoming_storage.dummy->next = new;
                sem_post(&hubs[this->order_package->receiving_hub_id - 1].incoming_storage_mutex);
                //printf("DRONE DELIVERY JOB UNLOADED\n");

                sem_post(&receivers[this->order_package->receiver_id - 1].have_package);
                //printf("DRONE DELIVERY JOB NOTIFIED\n");

                


                sem_wait(&hubs[this->order_package->receiving_hub_id - 1].charging_space_mutex);
                    park_drone(this, this->order_package->receiving_hub_id);
                    this->last_timestamp = timeInMilliseconds();
                sem_post(&hubs[this->order_package->receiving_hub_id - 1].charging_space_mutex);

                //printf("DRONE DELIVERY JOB PARKED\n");
                
                this->order_dummy->next = NULL;
                free(tmp_order);
                //printf("DRONE DELIVERY JOB DONE, CURRENT HUB : %d\n", this->current_hub_id);
                this->total_delivered++;

                break;
            case DRONE_HELP_HUB:
                //printf("DRONE HELP JOB RECEIVED\n");
                start = timeInMilliseconds();
                travel_time = find_distance(this->current_hub_id, this->order_hub);
                
                sem_wait(&hubs[this->order_hub - 1].charging_space.empty);

                this->current_range = calculate_drone_charge(timeInMilliseconds() - start, this->current_range, this->max_range);

                if(this->current_range < travel_time){
                    wait((travel_time - this->current_range) * UNIT_TIME);
                }

                sem_post(&hubs[this->current_hub_id - 1].charging_space.empty);
                travel(travel_time, this->speed);
                this->current_range -= range_decrease(travel_time, this->speed);
                this->current_hub_id = this->order_hub;

                this->order_dummy->next = tmp_order->next;
                free(tmp_order);

                sem_post(&this->order_ready);
                //printf("DRONE HELP JOB DONE, CURRENT HUB : %d\n", this->current_hub_id);

                break;
        }

    }
    //printf("DRONE DONE : %d, TOTAL DELIVERED : %d\n", *index+1, this->total_delivered);
    FillDroneInfo(&drone_info, *index + 1, this->current_hub_id, this->current_range, NULL, 0);
    WriteOutput(NULL, NULL, &drone_info, NULL, DRONE_STOPPED);
    free(index);
    pthread_exit(0);
}

// ************* DRONE STOP  ************************

// *************** HUB START ************************

int should_operate(){
    int i = 0;

    sem_wait(&active_senders_mutex);
        if(active_senders){
            sem_post(&active_senders_mutex);
            return 1;        
        }
    sem_post(&active_senders_mutex);
    //printf("NO SENDERS\n");

    for(i = 0; i < num_of_hubs; i++){
        sem_wait(&hubs[i].incoming_storage_mutex);
            if(hubs[i].incoming_storage.dummy->next){
                sem_post(&hubs[i].incoming_storage_mutex);       
                return 1; 
            }
        sem_post(&hubs[i].incoming_storage_mutex);
        //printf("HUB %d NO INCOMING\n", i+1);

        sem_wait(&hubs[i].outgoing_storage_mutex);
            if(hubs[i].outgoing_storage.dummy->next){
                sem_post(&hubs[i].outgoing_storage_mutex);
                return 1;        
            }
        sem_post(&hubs[i].outgoing_storage_mutex);
        //printf("HUB %d NO OUTGOING\n", i+1);
    }
    //printf("MUST END\n");
    return 0;
}

void print_distance_vector(int hub_id){
    DistanceNode *current = hubs[hub_id - 1].distance_dummy->next;
    while(current){
        printf("TO : %d, DISTANCE : %d\n", current->id, current->distance);
        current = current->next;
    }
}

int remaining_storage(int hub_id){
    StorageNode *current;
    int i = 0;
    sem_wait(&hubs[hub_id - 1].outgoing_storage_mutex);
        current = hubs[hub_id - 1].outgoing_storage.dummy->next;
        while(current){
            current = current->next;
            i++;
        }
    sem_post(&hubs[hub_id - 1].outgoing_storage_mutex);
    return i;
}

void* hub_thread(int *index){
    Hub *this = &hubs[*index];
    ChargingSpaceNode *current_charging_space;
    StorageNode *tmp_storage_node;
    Drone *delivery_drone;
    int i = 0;

    HubInfo hub_info;
    FillHubInfo(&hub_info, *index + 1);
    WriteOutput(NULL, NULL, NULL, &hub_info, HUB_CREATED);

    while(should_operate()){
        //if(*index == 2 || *index == 0) break;
        int result, found = 0;
        DistanceNode *current;
        ChargingSpaceNode *tmp;

        result = wait_for_one_sec(&this->outgoing_storage.full);
        //result = sem_trywait(&this->action_count);
        if(result == -1){
            //printf("NO PACKAGE LEFT HUB : %d\n", *index + 1);
            continue;
        }


        //printf("HUB JOB : %d\n", *index + 1);
        
        
        sem_wait(&this->charging_space_mutex);
            current_charging_space = this->charging_space.dummy;
            if(current_charging_space->next){
                tmp = current_charging_space->next;
                delivery_drone = tmp->drone;
                current_charging_space->next = tmp->next;
                free(tmp);
                sem_post(&this->charging_space_mutex);

                sem_wait(&this->outgoing_storage_mutex);
                    tmp_storage_node = this->outgoing_storage.dummy->next;
                    this->outgoing_storage.dummy->next = tmp_storage_node->next;
                sem_post(&this->outgoing_storage_mutex);                                    
                
                delivery_drone->order_package = tmp_storage_node->package;
                free(tmp_storage_node);

                delivery_drone->order_dummy->next = malloc(sizeof(DroneOrderNode));
                delivery_drone->order_dummy->next->type = DRONE_DELIVER_PACKAGE;
                delivery_drone->order_dummy->next->next = NULL;
                sem_post(&delivery_drone->order_ready);
                this->total_assigned++;
                //printf("JOB ASSIGNED TO DRONE FROM HUB : %d, REMAINING IN STORAGE : %d\n", *index+1, remaining_storage(*index + 1));
                continue;
            }   
        sem_post(&this->charging_space_mutex);

        //printf("LOOK FOR HELP NEARBY HUB : %d\n", *index+1);
        
        current = this->distance_dummy->next->next;
        
        while(current){          
            sem_wait(&hubs[current->id - 1].charging_space_mutex);
                tmp = hubs[current->id - 1].charging_space.dummy;         

                if(tmp->next != 0){
                    ChargingSpaceNode *closest = tmp->next;
                    tmp->next = closest->next;
                    sem_post(&hubs[current->id - 1].charging_space_mutex);

                    
                    delivery_drone = closest->drone;
                    free(closest);

                    sem_wait(&this->outgoing_storage_mutex);
                        tmp_storage_node = this->outgoing_storage.dummy->next;
                        this->outgoing_storage.dummy->next = tmp_storage_node->next;
                    sem_post(&this->outgoing_storage_mutex);

                    delivery_drone->order_package = tmp_storage_node->package;
                    free(tmp_storage_node);

                    delivery_drone->order_hub = *index+1;

                    delivery_drone->order_dummy->next = malloc(sizeof(DroneOrderNode));
                    delivery_drone->order_dummy->next->type = DRONE_HELP_HUB;                    
                    
                    delivery_drone->order_dummy->next->next = malloc(sizeof(DroneOrderNode));
                    delivery_drone->order_dummy->next->next->type = DRONE_DELIVER_PACKAGE;
                    delivery_drone->order_dummy->next->next->next = NULL;
                    sem_post(&delivery_drone->order_ready);
                    
                    found = 1;
                    this->total_assigned++;
                    //printf("DRONE CALLED FROM HUB : %d, REMAINING IN STORAGE : %d\n", *index+1, remaining_storage(*index + 1));
                    break;
                }          
            sem_post(&hubs[current->id - 1].charging_space_mutex);
            current = current->next; 
        }
        if(!found){
            //printf("NOT AVAILABLE DRONE HUB : %d\n", *index+1);
            //sleep(2);            
            wait(UNIT_TIME);
            sem_post(&this->outgoing_storage.full);
        }
        
              
    }
    //printf("HUB DONE : %d, TOTAL ASSIGNED : %d\n", *index + 1, this->total_assigned);
    sem_wait(&active_hubs_mutex);
        active_hubs--;
    sem_post(&active_hubs_mutex);
    FillHubInfo(&hub_info, *index + 1);
    WriteOutput(NULL, NULL, NULL, &hub_info, HUB_STOPPED);
    free(index);
    pthread_exit(0);
}

// *************** HUB STOP  ************************
void check_and_init_sem(sem_t *sem, int value, unsigned int id){
    int result = 0, expected = 0;
    result = sem_init(sem, 0, value);
}

void initialize_hubs(){
    DistanceNode *current;
    DistanceNode *tmp;
    int i = 0, j = 0;
    scanf("%d", &num_of_hubs);
    hubs = malloc(sizeof(Hub) * num_of_hubs);

    for(i = 0; i < num_of_hubs; i++){
        int incoming_package_storage;
        int outgoing_package_storage;
        int charging_space;

        scanf("%d %d %d", &incoming_package_storage,
                            &outgoing_package_storage,
                            &charging_space);
        
        
        sem_destroy(&hubs[i].incoming_storage_mutex);
        sem_destroy(&hubs[i].outgoing_storage_mutex);
        sem_destroy(&hubs[i].charging_space_mutex);

        sem_destroy(&hubs[i].incoming_storage.empty);
        sem_destroy(&hubs[i].outgoing_storage.empty);
        sem_destroy(&hubs[i].charging_space.empty);
        
        sem_destroy(&hubs[i].incoming_storage.full);
        sem_destroy(&hubs[i].outgoing_storage.full);
        sem_destroy(&hubs[i].charging_space.full);  
        

        
        check_and_init_sem(&hubs[i].incoming_storage.empty, incoming_package_storage, i + 1);
        check_and_init_sem(&hubs[i].incoming_storage.full, 0, i + 1);
        check_and_init_sem(&hubs[i].incoming_storage_mutex, 1, i + 1);

        check_and_init_sem(&hubs[i].outgoing_storage.empty, outgoing_package_storage, i + 1);
        check_and_init_sem(&hubs[i].outgoing_storage.full, 0, i + 1);
        check_and_init_sem(&hubs[i].outgoing_storage_mutex, 1, i + 1);

        check_and_init_sem(&hubs[i].charging_space.empty, charging_space, i + 1);
        check_and_init_sem(&hubs[i].charging_space.full, 0, i + 1);
        check_and_init_sem(&hubs[i].charging_space_mutex, 1, i + 1);
        

        /*
        sem_init(&hubs[i].incoming_storage.empty , 0, incoming_package_storage);
        sem_init(&hubs[i].incoming_storage.full, 0, 0);
        sem_init(&hubs[i].incoming_storage_mutex, 0, 1);

        sem_init(&hubs[i].outgoing_storage.empty , 0, outgoing_package_storage);
        sem_init(&hubs[i].outgoing_storage.full, 0, 0);
        sem_init(&hubs[i].outgoing_storage_mutex, 0, 1);

        sem_init(&hubs[i].charging_space.empty , 0, charging_space);
        sem_init(&hubs[i].charging_space.full, 0, 0);
        sem_init(&hubs[i].charging_space_mutex, 0, 1);

        */

        

        hubs[i].distance_dummy = malloc(sizeof(DistanceNode));
        hubs[i].distance_dummy->next = NULL;

        hubs[i].outgoing_storage.dummy = malloc(sizeof(StorageNode));
        hubs[i].outgoing_storage.dummy->next = NULL;

        hubs[i].incoming_storage.dummy = malloc(sizeof(StorageNode));
        hubs[i].incoming_storage.dummy->next = NULL;

        hubs[i].charging_space.dummy = malloc(sizeof(ChargingSpaceNode));
        hubs[i].charging_space.dummy->next = NULL;

        hubs[i].total_assigned = 0;

        for(j = 0; j < num_of_hubs; j++){
            int distance = 0;
            current = hubs[i].distance_dummy;
            tmp = malloc(sizeof(DistanceNode));

            scanf("%d", &distance);            

            while(current->next && distance > current->next->distance){
                current = current->next;
            } 
            
            tmp->next = current->next;
            tmp->id = j+1;
            tmp->distance = distance;
            current->next = tmp;            
        } 
    }
    active_hubs = num_of_hubs;
    
    
}

void initialize_senders(){
    int i = 0;
    senders = malloc(sizeof(Sender) * num_of_hubs);
    for(i = 0; i < num_of_hubs; i++){
        scanf("%d %d %d", &senders[i].wait_time, &senders[i].assigned_hub_id, &senders[i].remaining_packages);
        senders[i].total_sent = 0;
    }
    active_senders = num_of_hubs;
    
}

void initialize_receivers(){
    int i = 0;
    receivers = malloc(sizeof(Receiver) * num_of_hubs);
    for(i = 0; i < num_of_hubs; i++){
        scanf("%d %d", &receivers[i].wait_time, &receivers[i].assigned_hub_id);

        sem_destroy(&receivers[i].have_package);
        sem_init(&receivers[i].have_package, 0, 0);

        receivers[i].total_received = 0;
    }
}

void initialize_drones(){
    int i = 0;
    scanf(" %d ", &num_of_drones);
    drones = malloc(sizeof(Drone) * num_of_drones);
    for(i = 0; i < num_of_drones; i++){
        scanf("%d %d %d", &drones[i].speed, &drones[i].current_hub_id, &drones[i].max_range);

        park_drone(&drones[i], drones[i].current_hub_id);
        drones[i].order_dummy = malloc(sizeof(DroneOrderNode));
        drones[i].order_dummy->next = NULL;

        sem_destroy(&drones[i].order_ready);
        sem_init(&drones[i].order_ready, 0, 0);

        drones[i].total_delivered = 0;
    }
}

void initialize_world(){    
    int i = 0;
    initialize_hubs();   

    initialize_senders();

    initialize_receivers();

    initialize_drones();

    tid = malloc(sizeof(pthread_t) * (num_of_hubs * 3 + num_of_drones));

    sem_init(&active_hubs_mutex, O_CREAT, 1);
    sem_init(&active_senders_mutex, O_CREAT, 1);
}

void destroy_world(){
    int i = 0, j = 0;

    for(i = 0; i < num_of_hubs; i++){
        DistanceNode *current = hubs[i].distance_dummy;
        DistanceNode *tmp = current->next;

        

        ChargingSpaceNode *current_spot = hubs[i].charging_space.dummy;
        ChargingSpaceNode *tmp_spot = current_spot->next;



        
        while(tmp){
            current->next = tmp->next;
            free(tmp);
            tmp = current->next;
        }
        free(current);

        while(tmp_spot){
            current_spot->next = tmp_spot->next;
            free(tmp_spot);
            tmp_spot = current_spot->next;
        }
        free(current_spot);


        

        sem_destroy(&hubs[i].incoming_storage.full);
        sem_destroy(&hubs[i].incoming_storage.empty);
        sem_destroy(&hubs[i].incoming_storage_mutex);

        sem_destroy(&hubs[i].outgoing_storage.full);
        sem_destroy(&hubs[i].outgoing_storage.empty);
        sem_destroy(&hubs[i].outgoing_storage_mutex);

        sem_destroy(&hubs[i].charging_space.full);
        sem_destroy(&hubs[i].charging_space.empty);
        sem_destroy(&hubs[i].charging_space_mutex);

        sem_destroy(&receivers[i].have_package);
    }
    free(hubs);
    free(senders);
    free(receivers);
    free(drones);
    free(tid);

    for(i = 0; i < num_of_drones; i++){
        DroneOrderNode *current, *tmp;
        current = drones[i].order_dummy;
        tmp = current->next;
        while(tmp){
            current->next = tmp->next;            
            free(tmp);
            tmp = current->next;
        }
        free(current);

        sem_destroy(&drones[i].order_ready);
    }
}




