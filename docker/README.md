# HACK-A-THON GIT REPO

The following guide describes how to set up the system and get it running on a Jetson. 

## Prerequisites
This system is designed for Linux only and requires Docker with Compose v2.

### Installing Docker and Docker Compose v2
1. Install Docker:
```bash
sudo apt update
sudo apt install docker.io
```

2. Add your user to the docker group (to run Docker without sudo):
```bash
sudo usermod -aG docker $USER
```

3. Install Docker Compose v2:
```bash
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-linux-x86_64" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
```

4. **Important:** Log out and log back in (or restart) for the docker group membership to take effect.

5. Verify installation:
```bash
docker --version
docker compose version
```

## Docker Setup
The whole system runs in a docker container. Follow these steps:

### 1. Build the Docker Image
Before running for the first time, build the Docker image:
```bash
docker build -t p7-drone-sit-awareness .
```

### 2. Start the Container
```bash
docker compose -f docker-compose-drone.yml up
```
Press d to detach.

### 3. Enable Display Support
To get GUI applications working in the container, run this command **after** the container is up:
```bash
xhost +local:root 
```

### Enter docker container:
To enter the docker container run:
```bash
docker exec -it p7-drone-sit-awareness bash
```

### 4. Test Display
To verify that the display is working, enter the container and test:
```bash
xeyes
```
This should show a pair of eyes. Press Ctrl+C to close it again.
