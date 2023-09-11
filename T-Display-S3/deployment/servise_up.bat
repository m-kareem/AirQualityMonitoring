@ECHO OFF
docker rmi mqttbridge:1.0.0
docker build -t mqttbridge:1.0.0 .
docker-compose -f YorkUFSC_monitoring.yaml up -d