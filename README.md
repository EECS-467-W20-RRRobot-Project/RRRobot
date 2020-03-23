# RRRobot!

## Contributors

### Project Team

- Sravan Balaji ([balajsra@umich.edu](mailto:balajsra@umich.edu))
- Chenxi Gu ([chenxgu@umich.edu](mailto:chenxgu@umich.edu))
- Jake Johnson ([thejakej@umich.edu](mailto:thejakej@umich.edu))
- Derek Witcpalek ([dwitcpa@umich.edu](mailto:dwitcpa@umich.edu))

### EECS 467 W20 Course Staff

- Prof. Chad Jenkins ([ocj@umich.edu](mailto:ocj@umich.edu))
- Xiaotong Chen ([cxt@umich.edu](mailto:cxt@umich.edu))
- Jana Pavlasek ([pavlasek@umich.edu](mailto:pavlasek@umich.edu))

## Documents

1. [Project Poster](Documents/1.%20Project%20Poster.pdf)
2. [Project Proposal](Documents/2.%20Project%20Proposal.pdf)

## Introduction

Repository for UMICH EECS 467: Autonomous Robotics (W20) RRRobot! project.

To get started, you will need to install Docker on your system. Information on what Docker is, how to install it, and how to use it can be found in [Getting Started with Docker](https://sravanbalaji.com/Web%20Pages/blog_docker.html).

## Running Containers

**NOTE:** If using VcXsrv for Windows to enable GUI applications, run XLaunch using [config.xlaunch](utils/config.xlaunch).

**NOTE:** To enable X forwarding, be sure to update the `IP_ADDRESS` variable in [.env](src/.env) with your computer's IP Address.

1. Start Docker Machine (default)
    - WSL: `docker-machine.exe start default`
    - Other: `docker-machine start default`
2. Set Docker Machine Environment
    - WSL: `docker-machine.exe env`
    - Other: `docker-machine env`
3. Navigate to src folder
    - `cd /PATH/TO/rrrobot/src`
4. Use Docker Compose to run a service (refer to [docker-compose.yml](src/docker-compose.yml) for a list of services)
   - `docker-compose run --rm <service_name>`
