FROM python:3.7-alpine

LABEL maintainer="Nilhcem" \
      description="MQTT to InfluxDB Bridge"

ADD main_TrH_co2voc.py /

COPY requirements.txt /
RUN pip install -r /requirements.txt

COPY . /app
WORKDIR /app

CMD ["python3", "-u", "main_TrH_co2voc.py"]
