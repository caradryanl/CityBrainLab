# Reproducing Instruction

CBEngine is an efficient and scalable traffic simulator. In our paper, baselines are evaluated in docker. In this instruction, we provide the guidance to reproduce our results in the efficiency and scalability experiment. 

All the dataset mentioned in our paper is released on [Google Drive](https://drive.google.com/drive/folders/1e8wjEYFnDXluHaOxyAzJOOknNvJPZ4_r?usp=sharing). To reproduce our experiments, download the dataset and unzip it in the folder `/CBEngine/efficiency_scalability/` . We demonstrate the process of evaluating efficiency via the following case on the dataset of Changchun.

### CBEngine

#### Create the environment

1. Pull the docker image of CBEngine:

```bash
    docker pull citybrainlab/cbengine:latest
```

2. Run these commands at `/CBEngine/efficiency_scalability/`, moving the data to the script folder:

   ```
   	cp ./ExperimentalSetup/Changchun/CBEngine/data/flow.txt ./CBEngine/data/
   	cp ./ExperimentalSetup/Changchun/CBEngine/data/roadnet.txt ./CBEngine/data/
   ```

2. Create a docker container:

```
    docker run -it -v /path/to/your/CBLab/CBEngine/efficiency_scalability/CBEngine/:/CBEngine_experiment citybrainlab/cbengine:latest bash
```

#### Reproducing the experiments

In the docker container of CBEngine, run these commands.

```
    cd /CBEngine_experiment
    python3 run.py
```

You will see several folders within data files corresponding to an experiment.

Then CBEngine will run the simulation and results will be seen after a while.

During the running, the output will be printed. Output *t* represents the current time of the simulator, and *v* represents the current number of vehicles in transit.



### SUMO

The process of experiment reproduction of SUMO is similar to that of CBEngine.

#### Create the environment

1. Pull the docker image of SUMO.

```
    docker pull citybrainlab/sumo_docker:latest
```

2. Run these commands at `/CBEngine/efficiency_scalability/`, moving the data to the script folder:

   ```
   	cp ./ExperimentalSetup/Changchun/SUMO/data/flow.rou.xml ./SUMO/data/
   	cp ./ExperimentalSetup/Changchun/SUMO/data/roadnet.net.xml ./SUMO/data/
   ```

2. Create a docker container:

```
    docker run -it -v /path/to/your/CBLab/CBEngine/efficiency_scalability/SUMO:/SUMO_experiment citybrainlab/sumo_docker:latest bash
```

In the container, SUMO had already been installed in `/root/sumo`.

#### Reproducing the experiments

In the docker container of SUMO, execute these commands:

```
    cd /SUMO_experiment
    python run.py
```

During the running of the program, the output will be printed. Output *t* represents the current time of the simulator, and *v* represents the current number of vehicles in transit.

At the end of the program,  the runtime of the simulation will be printed.



### CityFlow

The process of experiment reproduction of CityFlow is similar to that of CBEngine.

#### Create the environment

1. Pull the docker image.

```
    docker pull cityflowproject/cityflow:latest
```

2. Run these commands at `/CBEngine/efficiency_scalability/`, moving the data to the script folder:

   ```
   	cp ./ExperimentalSetup/Changchun/CityFlow/data/flow.json ./CityFlow/data/
   	cp ./ExperimentalSetup/Changchun/CityFlow/data/roadnet.json ./CityFlow/data/
   ```

2. Create a docker container. `/path/to/your/CBLab` is the path of CBLab downloaded.

```
    docker run -it -v /path/to/your/CBLab/CBEngine/efficiency_scalability/CityFlow:/CityFlow_experiment cityflowproject/cityflow:latest bash
```

#### Reproducing the experiments

Take *Nanchang* as an example:

```
    cd /CityFlow_experiment
    python run.py
```

During the running of the program, the output will be printed. Output *t* represents the current time of the simulator, and *v* represents the current number of vehicles in transit.

At the end of the program, the runtime of the simulation will be printed.