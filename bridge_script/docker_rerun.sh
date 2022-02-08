echo "1. docker stop:"
docker stop mqttbridge

echo "2. docker rm:"
docker rm mqttbridge

echo "3. docker image remove:"
docker image remove raqm/mqttbridge

echo "4. docker build:"
docker build -t raqm/mqttbridge .

echo "5. docker run:"
docker run -d --restart always --name mqttbridge raqm/mqttbridge
#docker run --name mqttbridge raqm/mqttbridge

echo "5. docker ps:"
docker ps -a
